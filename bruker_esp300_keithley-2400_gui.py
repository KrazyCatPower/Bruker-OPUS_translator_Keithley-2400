"""
GUI для измерения спектров на Bruker IFS 125-HR (через OPUS)
с управлением транслятором UTS-150PP (ESP300 / SMC100)
и источником Keithley 2400 (RS-232).

Keithley-функции интегрированы из scontel_gui.py:
  - потокобезопасный доступ (lock)
  - фоновый опрос V/I/статуса
  - отдельное компактное окно настроек
  - индикатор compliance (COMPL V / COMPL I)
  - корректное переключение source/measure
  - измерение и сохранение ВАХ (режим 4 и кнопка в окне Keithley)

Начальное значение compliance: 40e-6
  → при источнике напряжения = 40 µA
  → при источнике тока       = 40 µV

Зависимости:
    pip install brukeropus pyserial numpy matplotlib psutil
"""

import os
import sys
import time
import math
import subprocess
import threading
import queue
from datetime import datetime

import serial
import serial.tools.list_ports
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, scrolledtext

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import psutil
except ImportError:
    psutil = None


K_POLL_INTERVAL_S = 0.5
DEFAULT_COMPLIANCE = 40e-6   # 40 µA / 40 µV


# ============================================================
#  ESP300
# ============================================================

class ESP300Error(Exception):
    pass


class ESP300:
    def __init__(self, port, baudrate=19200, timeout=3.0,
                 log_func=None, unit_scale=1000.0):
        self.log = log_func or print
        self.unit_scale = unit_scale
        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=5.0,
            xonxoff=True, rtscts=False, dsrdtr=False,
        )
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        try:
            pos = self.get_position_um(1)
            self.log(f"ESP300: связь OK, позиция = {pos:.2f} µm")
        except Exception as e:
            self.log(f"ESP300: первичный запрос позиции — {e}")

    def _drain(self, wait=0.1):
        time.sleep(wait)
        try:
            while self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
                time.sleep(0.02)
        except Exception:
            pass

    def _write(self, cmd):
        if not self.ser.is_open:
            raise ESP300Error("Порт не открыт")
        self._drain(0.05)
        try:
            self.ser.write(f"{cmd}\r".encode("ascii"))
        except serial.SerialTimeoutException:
            try:
                self.ser.reset_output_buffer()
                self.ser.reset_input_buffer()
            except Exception:
                pass
            time.sleep(0.3)
            self.ser.write(f"{cmd}\r".encode("ascii"))

    def _readline(self, wait=0.2):
        time.sleep(wait)
        try:
            resp = self.ser.read_until(b"\r\n")
        except serial.SerialException as e:
            raise ESP300Error(f"Read error: {e}")
        if not resp:
            return ""
        return resp.decode("ascii", errors="replace").strip()

    def _query(self, cmd, retries=3):
        for attempt in range(retries):
            self._write(cmd)
            resp = self._readline(0.2)
            if resp:
                return resp
            if attempt < retries - 1:
                time.sleep(0.3)
        raise ESP300Error(f"Пустой ответ на '{cmd}'")

    def _parse_number(self, resp):
        if not resp:
            raise ESP300Error("Пустой ответ")
        s = resp.replace(",", ".").strip()
        for token in reversed(s.split()):
            cleaned = ""
            for ch in token:
                if ch.isdigit() or ch in ".+-eE":
                    cleaned += ch
                elif cleaned:
                    break
            if cleaned:
                try:
                    return float(cleaned)
                except ValueError:
                    continue
        raise ESP300Error(f"Не число в ответе: {resp!r}")

    def get_position_um(self, axis=1):
        resp = self._query(f"{axis}TP?")
        return self._parse_number(resp) * self.unit_scale

    def move_absolute_um(self, position_um, axis=1):
        self._write(f"{axis}PA{position_um / self.unit_scale:.6f}")

    def wait_for_target(self, target_um, tolerance_um=5.0,
                        timeout=180.0, axis=1):
        time.sleep(0.5)
        t0 = time.time()
        last_pos = None
        while time.time() - t0 < timeout:
            try:
                pos = self.get_position_um(axis)
                last_pos = pos
                if abs(pos - target_um) <= tolerance_um:
                    self.log(f"ESP300: цель достигнута ({pos:.2f} µm)")
                    return True
            except Exception as e:
                self.log(f"ESP300 wait: {e}")
            time.sleep(1.0)
        if last_pos is not None:
            self.log(f"ESP300: таймаут. Текущая {last_pos:.2f} µm, "
                     f"цель {target_um:.2f} µm")
        return False

    def motor_on(self, axis=1):
        self._write(f"{axis}MO")

    def motor_off(self, axis=1):
        self._write(f"{axis}MF")

    def stop(self, axis=1):
        try:
            self._write(f"{axis}ST")
        except Exception:
            pass

    def set_velocity_um(self, v_um, axis=1):
        self._write(f"{axis}VA{v_um / self.unit_scale:.6f}")

    def set_acceleration_um(self, a_um, axis=1):
        self._write(f"{axis}AC{a_um / self.unit_scale:.6f}")

    def close(self):
        if self.ser.is_open:
            self.ser.close()


# ============================================================
#  SMC100
# ============================================================

class SMC100Error(Exception):
    pass


class SMC100:
    def __init__(self, port, baudrate=57600, timeout=2.0, axis=1):
        self.axis = axis
        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            xonxoff=False, rtscts=False, dsrdtr=False,
            timeout=timeout, write_timeout=5.0,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.1)
        try:
            self._query("ID?")
        except Exception:
            self._query("VE")

    def _write(self, command):
        if not self.ser.is_open:
            raise SMC100Error("Порт не открыт")
        self.ser.write(f"{self.axis}{command}\r\n".encode("ascii"))
        time.sleep(0.01)

    def _readline(self):
        resp = self.ser.readline()
        if not resp:
            raise SMC100Error("Таймаут чтения")
        return resp.decode("ascii", errors="replace").strip()

    def _query(self, command):
        self._write(command)
        return self._readline()

    def _strip_axis(self, resp):
        if resp and resp[0].isdigit():
            return resp[1:]
        return resp

    def _query_float(self, command):
        resp = self._strip_axis(self._query(command))
        if len(resp) > 2 and resp[:2].isalpha():
            resp = resp[2:]
        return float(resp)

    def get_position(self):
        return self._query_float("TP")

    def move_absolute(self, position):
        self._write(f"PA{position}"); self._readline()

    def move_relative(self, distance):
        self._write(f"PR{distance}"); self._readline()

    def stop(self):
        try:
            self._write("ST"); self._readline()
        except Exception:
            pass

    def home_search(self):
        self._write("OR"); self._readline()

    def get_status(self):
        return self._strip_axis(self._query("TS"))

    def set_velocity(self, v):
        try:
            self._write(f"VA{v}"); self._readline()
        except Exception:
            pass

    def set_acceleration(self, a):
        try:
            self._write(f"AC{a}"); self._readline()
        except Exception:
            pass

    def close(self):
        if self.ser.is_open:
            self.ser.close()


# ============================================================
#  Единый интерфейс подвижки
# ============================================================

class MotionController:
    def __init__(self, log_func=None):
        self.log = log_func or print
        self.device = None
        self.driver_name = ""

    def is_connected(self):
        return self.device is not None


class ESP300Motion(MotionController):
    def __init__(self, port, log_func=None, baudrate=19200,
                 unit_scale=1000.0, velocity_um=500.0, accel_um=500.0):
        super().__init__(log_func)
        self.driver_name = "ESP300"
        self.device = ESP300(port, baudrate=baudrate,
                             log_func=log_func, unit_scale=unit_scale)
        self._velocity_um = velocity_um
        self._accel_um = accel_um
        self._motor_on = False

    def enable(self):
        if not self._motor_on:
            try:
                self.device.motor_on(1)
                self._motor_on = True
            except Exception as e:
                self.log(f"ESP300: motor_on — {e}")

    def disable(self):
        if self._motor_on:
            try:
                self.device.motor_off(1)
                self._motor_on = False
            except Exception:
                pass

    def set_velocity(self, v): self._velocity_um = v
    def set_acceleration(self, a): self._accel_um = a

    def move_to(self, position_um, wait=True):
        self.enable()
        try:
            self.device.set_velocity_um(self._velocity_um, 1)
            self.device.set_acceleration_um(self._accel_um, 1)
        except Exception as e:
            self.log(f"ESP300: set VA/AC — {e}")
        self.device.move_absolute_um(position_um, 1)
        if wait:
            self.device.wait_for_target(position_um, tolerance_um=5.0,
                                        timeout=180.0)

    def move_by(self, distance_um, wait=True):
        self.move_to(self.get_position() + distance_um, wait=wait)

    def get_position(self):
        return self.device.get_position_um(1)

    def stop(self):
        self.device.stop(1)

    def home(self, mode=None, wait=True):
        self.log("ESP300: возврат в ноль (PA0)")
        self.enable()
        try:
            self.device.set_velocity_um(self._velocity_um, 1)
            self.device.set_acceleration_um(self._accel_um, 1)
        except Exception:
            pass
        self.device.move_absolute_um(0.0, 1)
        if wait:
            self.device.wait_for_target(0.0, tolerance_um=5.0, timeout=180.0)

    def disconnect(self):
        try:
            self.disable()
        except Exception:
            pass
        try:
            self.device.close()
        except Exception:
            pass
        self.device = None


class SMC100Motion(MotionController):
    def __init__(self, port, log_func=None, baudrate=57600):
        super().__init__(log_func)
        self.driver_name = "SMC100"
        self.device = SMC100(port, baudrate=baudrate)

    def enable(self):
        try:
            self.device.get_status()
        except Exception:
            pass

    def disable(self):
        pass

    def move_to(self, position, wait=True):
        self.device.move_absolute(position)
        if wait:
            self._wait_for_position(position)

    def move_by(self, distance, wait=True):
        self.device.move_relative(distance)

    def get_position(self):
        return self.device.get_position()

    def stop(self):
        self.device.stop()

    def _wait_for_position(self, target, tolerance=5.0, timeout=180.0):
        time.sleep(0.5)
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                pos = self.device.get_position()
                if abs(pos - target) <= tolerance:
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        return False

    def home(self, mode=None, wait=True):
        self.device.home_search()
        if wait:
            self._wait_for_position(0.0)

    def set_velocity(self, v): self.device.set_velocity(v)
    def set_acceleration(self, a): self.device.set_acceleration(a)

    def disconnect(self):
        try:
            self.device.close()
        except Exception:
            pass
        self.device = None


# ============================================================
#  Keithley 2400
# ============================================================

class Keithley2400:
    def __init__(self, port, baudrate=9600, timeout=1.0, log_func=None):
        self.log = log_func or (lambda m: print(m))
        self.ser = serial.Serial(
            port=port, baudrate=baudrate,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=timeout,
        )
        time.sleep(0.1)
        if not self.ser.is_open:
            raise ConnectionError(f"Не удалось открыть порт {port}")

        self._io_lock = threading.Lock()

        self._source_mode = 'VOLT'
        self._measure_mode = 'CURR'
        self._output_on = False
        self._has_data = False

        idn = self.query('*IDN?')
        if not idn:
            raise ConnectionError("Keithley 2400 не отвечает")
        self.log(f"Keithley 2400: {idn.strip()}")

        self._reset()
        try:
            self.write('*CLS')
        except Exception:
            pass

    def write(self, cmd):
        with self._io_lock:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()

    def query(self, cmd):
        with self._io_lock:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()
            return self.ser.readline().decode(errors='ignore').strip()

    def _reset(self):
        self.write('*RST')
        time.sleep(0.1)
        self.write(':SOUR:FUNC VOLT')
        self.write(':SOUR:VOLT:MODE FIXED')
        self.write(':SOUR:VOLT:LEV 0')
        self.write(':SENS:FUNC "CURR"')
        # Начальный compliance: 40 µA
        self.write(f':SENS:CURR:PROT {DEFAULT_COMPLIANCE}')
        self.write(':OUTP OFF')
        self._source_mode = 'VOLT'
        self._measure_mode = 'CURR'
        self._output_on = False

    def set_source_function(self, func):
        func = func.upper()
        if func not in ('VOLT', 'CURR'):
            raise ValueError("func должен быть 'VOLT' или 'CURR'")
        self.write(f':SOUR:FUNC {func}')
        self.write(f':SOUR:{func}:MODE FIXED')
        self._source_mode = func
        self.write(f':SOUR:{func}:LEV 0')
        if func == 'VOLT':
            self.write(':SENS:FUNC "CURR"')
            self._measure_mode = 'CURR'
        else:
            self.write(':SENS:FUNC "VOLT"')
            self._measure_mode = 'VOLT'
        return func

    def set_source_level(self, value):
        if self._source_mode is None:
            raise RuntimeError("Сначала задайте функцию источника")
        self.write(f':SOUR:{self._source_mode}:LEV {value}')

    def set_compliance(self, value):
        if self._source_mode == 'VOLT':
            self.write(f':SENS:CURR:PROT {value}')
            self._measure_mode = 'CURR'
        else:
            self.write(f':SENS:VOLT:PROT {value}')
            self._measure_mode = 'VOLT'

    def output_on(self):
        self.write(':OUTP ON')
        self._output_on = True

    def output_off(self):
        try:
            self.write(':OUTP OFF')
        except Exception:
            pass
        self._output_on = False

    def read(self):
        resp = self.query(':READ?')
        parts = [p.strip() for p in resp.split(',')]
        if len(parts) >= 2:
            try:
                v = float(parts[0])
                i = float(parts[1])
                self._has_data = True
                if self._measure_mode == 'VOLT':
                    return v, 'V'
                return i, 'A'
            except ValueError:
                pass
        return 0.0, '?'

    def read_v_and_i(self):
        resp = self.query(':READ?')
        parts = [p.strip() for p in resp.split(',')]
        if len(parts) >= 2:
            try:
                v = float(parts[0])
                i = float(parts[1])
                self._has_data = True
                return v, i
            except ValueError:
                pass
        return None, None

    def read_last(self):
        if not self._output_on:
            return None, None
        result = (None, None)
        try:
            resp = self.query(':READ?')
            parts = [p.strip() for p in resp.split(',')]
            if len(parts) >= 2:
                v = float(parts[0])
                i = float(parts[1])
                self._has_data = True
                result = (v, i)
        except Exception:
            result = (None, None)
        finally:
            try:
                self.write('*CLS')
            except Exception:
                pass
        return result

    def get_compliance_status(self):
        try:
            resp = self.query(':STAT:MEAS:COND?')
            st = int(float(resp))
        except Exception:
            return '?'
        bits = []
        if st & 0x02:
            bits.append('COMPL V')
        if st & 0x04:
            bits.append('COMPL I')
        return ' + '.join(bits) if bits else 'OK'

    def close(self):
        try:
            self.output_off()
            self.ser.close()
        except Exception:
            pass


# ============================================================
#  OPUS (Bruker)
# ============================================================

try:
    from brukeropus import Opus, read_opus
except ImportError:
    print("Не установлена библиотека brukeropus. Выполните: pip install brukeropus")
    sys.exit(1)


WAVENUMBER_TO_MEV = 1000.0 / 8065.544


class BrukerOPUS:
    def __init__(self, log_func=None):
        self.log = log_func or (lambda m: print(m))
        self.connected = False
        self._local = threading.local()
        self._generation = 0
        self._lock = threading.Lock()

    def _get_connection(self):
        with self._lock:
            cur_gen = self._generation
        if (not hasattr(self._local, 'opus')
                or self._local.opus is None
                or getattr(self._local, 'generation', -1) != cur_gen):
            old = getattr(self._local, 'opus', None)
            if old is not None:
                try:
                    old.disconnect()
                except Exception:
                    pass
            self._local.opus = Opus()
            self._local.generation = cur_gen
        return self._local.opus

    def _close_connection(self):
        if hasattr(self._local, 'opus') and self._local.opus is not None:
            try:
                self._local.opus.disconnect()
            except Exception:
                pass
            self._local.opus = None

    def _bump_generation(self):
        with self._lock:
            self._generation += 1

    def check_connection(self):
        try:
            version = self._get_connection().get_version()
            self.connected = True
            self.log(f"OPUS: подключение установлено ({version})")
            return True
        except Exception as e:
            self.log(f"OPUS: ошибка подключения — {e}")
            self.connected = False
            self._close_connection()
            return False

    def ping(self, timeout_ms=3000):
        try:
            v = self._get_connection().query('GET_VERSION_EXTENDED',
                                             timeout=timeout_ms)
            return bool(v)
        except Exception:
            return False

    def disconnect(self):
        self._close_connection()
        self.connected = False
        self.log("OPUS: отключено")

    def measure_sample(self):
        try:
            filepath = self._get_connection().measure_sample(unload=True)
            if filepath:
                self.log(f"OPUS: измерение выполнено — {filepath}")
                return filepath
            self.log("OPUS: измерение завершено, путь пуст")
            return None
        except Exception as e:
            self._close_connection()
            self.log(f"OPUS: ошибка измерения — {e}")
            return None

    def read_spectrum(self, filepath, retries=5, delay=0.5):
        last_error = None
        for _ in range(retries):
            try:
                opus_file = read_opus(filepath)
                if 'sm' in opus_file.data_keys:
                    data = opus_file.sm
                elif 'ab' in opus_file.data_keys:
                    data = opus_file.ab
                else:
                    key = opus_file.data_keys[0]
                    data = getattr(opus_file, key)
                return np.array(data.x), np.array(data.y)
            except PermissionError as e:
                last_error = e
                time.sleep(delay)
                continue
            except Exception as e:
                self.log(f"OPUS: ошибка чтения {filepath} — {e}")
                return None, None
        self.log(f"OPUS: файл заблокирован: {last_error}")
        return None, None


# ============================================================
#  OPUS Recovery
# ============================================================

class OPUSRecovery:
    def __init__(self, opus, exe_path=None,
                 ping_interval=10, ping_timeout_ms=3000,
                 fail_threshold=3, log_func=None):
        self.opus = opus
        self.exe_path = exe_path
        self.ping_interval = ping_interval
        self.ping_timeout_ms = ping_timeout_ms
        self.fail_threshold = fail_threshold
        self.log = log_func or print
        self.recovery_event = threading.Event()
        self.recovery_event.set()
        self._in_recovery = False
        self._recovery_lock = threading.Lock()
        self._stop_flag = threading.Event()
        self._watchdog_thread = None
        self._fail_count = 0
        self._enabled = True

    def start(self):
        if self._watchdog_thread is not None and self._watchdog_thread.is_alive():
            return
        self._stop_flag.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()
        self.log("OPUS watchdog: запущен")

    def stop(self):
        self._stop_flag.set()

    def set_enabled(self, enabled):
        self._enabled = bool(enabled)

    def wait_if_recovering(self, timeout=None):
        return self.recovery_event.wait(timeout=timeout)

    def trigger_recovery(self):
        with self._recovery_lock:
            if self._in_recovery:
                return
            self._in_recovery = True
            self.recovery_event.clear()
        threading.Thread(target=self._do_recovery, daemon=True).start()

    def _watchdog_loop(self):
        while not self._stop_flag.is_set():
            for _ in range(self.ping_interval):
                if self._stop_flag.is_set():
                    return
                time.sleep(1)
            if not self._enabled or self._in_recovery:
                continue
            if self.opus.ping(timeout_ms=self.ping_timeout_ms):
                self._fail_count = 0
            else:
                self._fail_count += 1
                self.log(f"OPUS watchdog: пинг не прошёл "
                         f"({self._fail_count}/{self.fail_threshold})")
                if self._fail_count >= self.fail_threshold:
                    self._fail_count = 0
                    self.trigger_recovery()

    def _do_recovery(self):
        try:
            if psutil is None:
                self.log("OPUS recovery: psutil не установлен")
                return
            exe = self.exe_path or self._find_opus_exe()
            if exe:
                self.log(f"OPUS recovery: путь — {exe}")
            killed = self._kill_opus()
            self.log(f"OPUS recovery: убито процессов — {killed}")
            time.sleep(3.0)
            self.opus._bump_generation()
            if exe:
                self._start_opus(exe)
            if self._wait_for_ready(timeout=90):
                self.log("OPUS recovery: OPUS готов")
            else:
                self.log("OPUS recovery: не удалось дождаться готовности")
        except Exception as e:
            self.log(f"OPUS recovery error: {e}")
        finally:
            with self._recovery_lock:
                self._in_recovery = False
            self.recovery_event.set()

    def _find_opus_exe(self):
        try:
            for p in psutil.process_iter(['name', 'exe']):
                try:
                    name = (p.info['name'] or '').lower()
                    exe = p.info['exe']
                    if 'opus' in name and exe and os.path.exists(exe):
                        return exe
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        except Exception:
            pass
        return None

    def _kill_opus(self):
        killed = 0
        try:
            for p in psutil.process_iter(['name']):
                try:
                    if 'opus' in (p.info['name'] or '').lower():
                        p.kill()
                        killed += 1
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        except Exception:
            pass
        return killed

    def _start_opus(self, exe_path):
        try:
            cwd = os.path.dirname(exe_path)
            subprocess.Popen([exe_path], cwd=cwd)
        except Exception as e:
            self.log(f"OPUS recovery: ошибка запуска — {e}")

    def _wait_for_ready(self, timeout=90):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._stop_flag.is_set():
                return False
            if self.opus.ping(timeout_ms=3000):
                self.opus._close_connection()
                try:
                    self.opus.check_connection()
                    return True
                except Exception:
                    pass
            time.sleep(3)
        return False


# ============================================================
#  Окно настроек Keithley — компактное, все параметры внутри
# ============================================================

class KeithleyWindow(tk.Toplevel):
    def __init__(self, master):
        super().__init__(master)
        self.app = master
        self.title("Keithley 2400")
        self.geometry("500x620")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.withdraw)
        self.withdraw()
        self._build_ui()

    def _build_ui(self):
        app = self.app
        pad = {"padx": 8, "pady": 4}

        # ---------- Подключение ----------
        conn = ttk.LabelFrame(self, text="Подключение", padding=6)
        conn.pack(fill="x", **pad)
        ttk.Label(conn, text="COM:").pack(side="left")
        self.combo = ttk.Combobox(conn, textvariable=app.keithley_port,
                                  width=10)
        self.combo.pack(side="left", padx=4)
        self.combo.bind("<Button-1>",
                        lambda e: app.refresh_keithley_ports())
        ttk.Button(conn, text="Подключить",
                   command=app.connect_keithley).pack(side="left", padx=3)
        ttk.Button(conn, text="Отключить",
                   command=app.disconnect_keithley).pack(side="left", padx=3)

        # ---------- Текущие измерения ----------
        info = ttk.LabelFrame(self, text="Измерения", padding=6)
        info.pack(fill="x", **pad)

        ttk.Label(info, text="U:").grid(row=0, column=0, sticky="w")
        ttk.Label(info, textvariable=app.keithley_v_var,
                  font=("TkDefaultFont", 11, "bold"),
                  foreground="blue").grid(row=0, column=1, sticky="w", padx=10)

        ttk.Label(info, text="I:").grid(row=1, column=0, sticky="w")
        ttk.Label(info, textvariable=app.keithley_i_var,
                  font=("TkDefaultFont", 11, "bold"),
                  foreground="blue").grid(row=1, column=1, sticky="w", padx=10)

        ttk.Label(info, text="Статус:").grid(row=2, column=0, sticky="w")
        app.keithley_status_label = ttk.Label(
            info, textvariable=app.keithley_status_var,
            font=("TkDefaultFont", 11, "bold"), foreground="green")
        app.keithley_status_label.grid(row=2, column=1, sticky="w", padx=10)

        # ---------- Источник ----------
        src = ttk.LabelFrame(self, text="Источник", padding=6)
        src.pack(fill="x", **pad)

        row = ttk.Frame(src)
        row.pack(fill="x")
        ttk.Radiobutton(row, text="Напряжение, В",
                        variable=app.keithley_source_mode,
                        value="VOLT",
                        command=app._update_compliance_label).pack(side="left")
        ttk.Radiobutton(row, text="Ток, А",
                        variable=app.keithley_source_mode,
                        value="CURR",
                        command=app._update_compliance_label).pack(
            side="left", padx=(10, 0))

        row = ttk.Frame(src)
        row.pack(fill="x", pady=(4, 0))
        ttk.Label(row, text="Уровень:").pack(side="left")
        ttk.Entry(row, textvariable=app.keithley_level,
                  width=12).pack(side="left", padx=4)

        row = ttk.Frame(src)
        row.pack(fill="x", pady=(4, 0))
        app.compliance_label = ttk.Label(row, text="Предел по току, А:")
        app.compliance_label.pack(side="left")
        ttk.Entry(row, textvariable=app.keithley_compliance,
                  width=12).pack(side="left", padx=4)

        row = ttk.Frame(src)
        row.pack(fill="x", pady=(4, 0))
        ttk.Checkbutton(row, text="Выход включён",
                        variable=app.keithley_output_on,
                        command=app._keithley_toggle_output).pack(side="left")

        app._update_compliance_label()

        # ---------- Свип ----------
        sw = ttk.LabelFrame(self, text="Свип / ВАХ", padding=6)
        sw.pack(fill="x", **pad)

        ttk.Checkbutton(sw, text="Фиксированный уровень (без свипа)",
                        variable=app.keithley_measure_fixed).pack(anchor="w")
        ttk.Checkbutton(sw, text="Свип туда-обратно",
                        variable=app.keithley_bidirectional_var).pack(
            anchor="w", pady=(2, 4))

        row = ttk.Frame(sw)
        row.pack(fill="x")
        ttk.Label(row, text="От:").pack(side="left")
        ttk.Entry(row, textvariable=app.keithley_sweep_start,
                  width=10).pack(side="left", padx=(2, 8))
        ttk.Label(row, text="До:").pack(side="left")
        ttk.Entry(row, textvariable=app.keithley_sweep_end,
                  width=10).pack(side="left", padx=(2, 0))

        row = ttk.Frame(sw)
        row.pack(fill="x", pady=(4, 0))
        ttk.Label(row, text="Шаг:").pack(side="left")
        ttk.Entry(row, textvariable=app.keithley_sweep_step,
                  width=10).pack(side="left", padx=(2, 8))
        ttk.Label(row, text="Задержка, с:").pack(side="left")
        ttk.Entry(row, textvariable=app.keithley_sweep_delay,
                  width=6).pack(side="left", padx=(2, 0))

        hint = ttk.Label(sw, foreground="gray",
                         text="ВАХ сохраняется в папку эксперимента\n"
                              "как VAH_<дата>_<время>.txt + _info.txt",
                         justify="left", font=("TkDefaultFont", 8, "italic"))
        hint.pack(anchor="w", pady=(6, 0))

        # ---------- Кнопки ----------
        btn = ttk.Frame(self)
        btn.pack(fill="x", **pad)
        ttk.Button(btn, text="Применить настройки",
                   command=app._keithley_apply_settings).pack(
            side="left", expand=True, fill="x", padx=2)
        ttk.Button(btn, text="Измерить ВАХ",
                   command=app.run_keithley_measurement).pack(
            side="left", expand=True, fill="x", padx=2)


# ============================================================
#  ГЛАВНОЕ ОКНО
# ============================================================

class Application(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Bruker IFS 125-HR + UTS-150PP (ESP300/SMC100) + Keithley 2400")
        self.geometry("1380x1050")
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.opus = None
        self.recovery = None
        self.motion = None
        self.keithley = None
        self.keithley_window = None
        self.measurement_thread = None
        self.stop_requested = False
        self.data_queue = queue.Queue()
        self.closing = False
        self.cbar_3d = None
        self.im_3d = None

        self.spectra = []
        self.last_2d_data = None

        self.mode_var = tk.IntVar(value=0)
        self.use_keithley_var = tk.BooleanVar(value=False)
        self.use_recovery = tk.BooleanVar(value=True)

        self.driver_var = tk.StringVar(value="ESP300")
        self.com_port = tk.StringVar(value="")
        self.keithley_port = tk.StringVar(value="")
        self.opus_status = tk.StringVar(value="Не подключено")
        self.motion_status = tk.StringVar(value="Не подключено")
        self.keithley_status = tk.StringVar(value="Не подключено")

        self.opus_exe_path = tk.StringVar(value="")
        self.ping_interval = tk.IntVar(value=10)
        self.fail_threshold = tk.IntVar(value=3)

        self.folder_path = tk.StringVar(value=os.path.expanduser("~"))
        self.file_basename = tk.StringVar(value="spectrum")
        self.unit_scale = tk.DoubleVar(value=1000.0)

        self.target_um = tk.DoubleVar(value=0.0)
        self.velocity_um = tk.DoubleVar(value=500.0)
        self.acceleration_um = tk.DoubleVar(value=500.0)
        self.current_pos_str = tk.StringVar(value="--")

        self.start_um = tk.DoubleVar(value=0.0)
        self.end_um = tk.DoubleVar(value=100.0)
        self.step_um = tk.DoubleVar(value=10.0)
        self.map_update_every = tk.IntVar(value=3)

        # --- Keithley (все настройки в одном месте) ---
        self.keithley_source_mode = tk.StringVar(value="VOLT")
        self.keithley_level = tk.DoubleVar(value=0.0)
        # Начальный compliance 40 µA / 40 µV
        self.keithley_compliance = tk.DoubleVar(value=DEFAULT_COMPLIANCE)
        self.keithley_measure_fixed = tk.BooleanVar(value=True)
        self.keithley_sweep_start = tk.DoubleVar(value=0.0)
        self.keithley_sweep_end = tk.DoubleVar(value=10.0)
        self.keithley_sweep_step = tk.DoubleVar(value=1.0)
        self.keithley_sweep_delay = tk.DoubleVar(value=0.2)
        self.keithley_bidirectional_var = tk.BooleanVar(value=False)
        self.keithley_output_on = tk.BooleanVar(value=False)

        # Живая индикация V/I/статуса
        self.keithley_v_var = tk.StringVar(value="-- V")
        self.keithley_i_var = tk.StringVar(value="-- A")
        self.keithley_status_var = tk.StringVar(value="--")
        self.keithley_status_label = None
        self.compliance_label = None

        # Фоновый опрос
        self._k_poll_active = False
        self._k_poll_stop = None
        self._k_poll_thread = None

        self.axis_x_min = tk.StringVar(value="")
        self.axis_x_max = tk.StringVar(value="")
        self.axis_y_min = tk.StringVar(value="")
        self.axis_y_max = tk.StringVar(value="")
        self.axis_z_min = tk.StringVar(value="")
        self.axis_z_max = tk.StringVar(value="")
        self.autoscale_3d = tk.BooleanVar(value=True)

        self.point_status = tk.StringVar(value="")
        self.progress = None

        self.create_widgets()
        self.refresh_com_ports()
        self.refresh_keithley_ports()
        self.update_mode()
        self.keithley_window = KeithleyWindow(self)
        self.after(100, self.process_queue)

    # ------------------------------------------------------------
    def create_widgets(self):
        left_outer = ttk.Frame(self, width=600)
        left_outer.pack(side="left", fill="y", padx=5, pady=5)
        left_outer.pack_propagate(False)

        self.left_canvas = tk.Canvas(left_outer, borderwidth=0,
                                     highlightthickness=0)
        self.left_scrollbar = ttk.Scrollbar(left_outer, orient="vertical",
                                            command=self.left_canvas.yview)
        self.left_canvas.configure(yscrollcommand=self.left_scrollbar.set)
        self.left_scrollbar.pack(side="right", fill="y")
        self.left_canvas.pack(side="left", fill="both", expand=True)

        left_frame = ttk.Frame(self.left_canvas)
        self._left_canvas_window = self.left_canvas.create_window(
            (0, 0), window=left_frame, anchor="nw")

        def _on_left_frame_configure(event):
            self.left_canvas.configure(
                scrollregion=self.left_canvas.bbox("all"))

        def _on_left_canvas_configure(event):
            self.left_canvas.itemconfig(self._left_canvas_window,
                                        width=event.width)

        left_frame.bind("<Configure>", _on_left_frame_configure)
        self.left_canvas.bind("<Configure>", _on_left_canvas_configure)

        def _on_mousewheel(event):
            try:
                x, y = self.winfo_pointerxy()
                w = self.winfo_containing(x, y)
                while w is not None:
                    if w == self.left_canvas or w == left_outer:
                        self.left_canvas.yview_scroll(
                            int(-1 * (event.delta / 120)), "units")
                        return
                    try:
                        w = w.master
                    except Exception:
                        break
            except Exception:
                pass

        self.bind_all("<MouseWheel>", _on_mousewheel)

        right_frame = ttk.Frame(self)
        right_frame.pack(side="left", fill="both", expand=True, padx=5, pady=5)

        # ============ ПОДКЛЮЧЕНИЯ ============
        conn_frame = ttk.LabelFrame(left_frame, text="Подключения", padding=6)
        conn_frame.pack(fill="x", pady=3)

        row_drv = ttk.Frame(conn_frame)
        row_drv.pack(fill="x", pady=1)
        ttk.Label(row_drv, text="Драйвер:").pack(side="left")
        ttk.Radiobutton(row_drv, text="ESP300", value="ESP300",
                        variable=self.driver_var,
                        command=self.update_mode).pack(side="left", padx=3)
        ttk.Radiobutton(row_drv, text="SMC100", value="SMC100",
                        variable=self.driver_var,
                        command=self.update_mode).pack(side="left", padx=3)

        row = ttk.Frame(conn_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="COM подвижки:").pack(side="left")
        self.com_combo = ttk.Combobox(row, textvariable=self.com_port, width=9)
        self.com_combo.pack(side="left", padx=3)
        ttk.Button(row, text="Обновить",
                   command=self.refresh_com_ports).pack(side="left", padx=2)
        ttk.Button(row, text="Подключить",
                   command=self.connect_motion).pack(side="left", padx=2)

        row_motion_status = ttk.Frame(conn_frame)
        row_motion_status.pack(fill="x", pady=(0, 2))
        ttk.Label(row_motion_status, text="Подвижка:").pack(side="left")
        self.motion_label = ttk.Label(row_motion_status,
                                      textvariable=self.motion_status,
                                      foreground="red",
                                      font=("TkDefaultFont", 9, "bold"))
        self.motion_label.pack(side="left", padx=5)

        self.esp_options_frame = ttk.Frame(conn_frame)
        row_us = ttk.Frame(self.esp_options_frame)
        row_us.pack(fill="x", pady=1)
        ttk.Label(row_us, text="unit_scale (µm/ед):").pack(side="left")
        ttk.Entry(row_us, textvariable=self.unit_scale,
                  width=8).pack(side="left", padx=3)
        ttk.Label(row_us, text="(1000 = мм)",
                  foreground="gray",
                  font=("TkDefaultFont", 7, "italic")).pack(side="left")

        row_k = ttk.Frame(conn_frame)
        row_k.pack(fill="x", pady=1)
        ttk.Label(row_k, text="COM Keithley:").pack(side="left")
        self.k_combo = ttk.Combobox(row_k, textvariable=self.keithley_port,
                                    width=9)
        self.k_combo.pack(side="left", padx=3)
        self.k_combo.bind("<Button-1>",
                          lambda e: self.refresh_keithley_ports())
        ttk.Button(row_k, text="Подключить",
                   command=self.connect_keithley).pack(side="left", padx=2)
        ttk.Button(row_k, text="Настройки",
                   command=self.toggle_keithley_window).pack(side="left", padx=2)

        row_k_status = ttk.Frame(conn_frame)
        row_k_status.pack(fill="x", pady=(0, 2))
        ttk.Label(row_k_status, text="Keithley:").pack(side="left")
        self.k_label = ttk.Label(row_k_status,
                                 textvariable=self.keithley_status,
                                 foreground="red",
                                 font=("TkDefaultFont", 9, "bold"))
        self.k_label.pack(side="left", padx=5)

        row2 = ttk.Frame(conn_frame)
        row2.pack(fill="x", pady=1)
        ttk.Label(row2, text="OPUS:").pack(side="left")
        self.opus_label = ttk.Label(row2, textvariable=self.opus_status,
                                    foreground="red",
                                    font=("TkDefaultFont", 9, "bold"))
        self.opus_label.pack(side="left", padx=5)
        ttk.Button(row2, text="Подключить OPUS",
                   command=self.connect_opus).pack(side="left", padx=2)

        # ============ ИНДИКАТОРЫ СТАТУСА ============
        status_frame = ttk.LabelFrame(left_frame, text="Статус устройств",
                                      padding=6)
        status_frame.pack(fill="x", pady=3)
        ind_row = ttk.Frame(status_frame)
        ind_row.pack(anchor="w")

        self.motion_ind_canvas = tk.Canvas(ind_row, width=18, height=18,
                                           highlightthickness=0)
        self.motion_ind_canvas.pack(side="left")
        self.motion_circle = self.motion_ind_canvas.create_oval(
            2, 2, 16, 16, fill="gray", outline="black")
        ttk.Label(ind_row, text="Подвижка").pack(side="left", padx=(2, 12))

        self.keithley_ind_canvas = tk.Canvas(ind_row, width=18, height=18,
                                             highlightthickness=0)
        self.keithley_ind_canvas.pack(side="left")
        self.keithley_circle = self.keithley_ind_canvas.create_oval(
            2, 2, 16, 16, fill="gray", outline="black")
        ttk.Label(ind_row, text="Keithley").pack(side="left", padx=(2, 12))

        self.opus_ind_canvas = tk.Canvas(ind_row, width=18, height=18,
                                         highlightthickness=0)
        self.opus_ind_canvas.pack(side="left")
        self.opus_circle = self.opus_ind_canvas.create_oval(
            2, 2, 16, 16, fill="gray", outline="black")
        ttk.Label(ind_row, text="OPUS").pack(side="left", padx=(2, 12))

        ttk.Button(status_frame, text="Проверить подключения",
                   command=self.check_connections).pack(anchor="w", pady=(4, 0))

        # ============ ВОССТАНОВЛЕНИЕ OPUS ============
        rec_frame = ttk.LabelFrame(left_frame,
                                   text="Авто-восстановление OPUS", padding=6)
        rec_frame.pack(fill="x", pady=3)
        ttk.Checkbutton(rec_frame,
                        text="Включить watchdog (убивать и перезапускать OPUS)",
                        variable=self.use_recovery,
                        command=self.apply_recovery_settings).pack(anchor="w")
        row = ttk.Frame(rec_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Путь к OPUS.exe:").pack(side="left")
        ttk.Entry(row, textvariable=self.opus_exe_path,
                  width=30).pack(side="left", padx=3, fill="x", expand=True)
        ttk.Button(row, text="...", width=3,
                   command=self.browse_opus_exe).pack(side="left")
        row = ttk.Frame(rec_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Интервал пинга (с):").pack(side="left")
        ttk.Entry(row, textvariable=self.ping_interval,
                  width=5).pack(side="left", padx=3)
        ttk.Label(row, text="Провалов:").pack(side="left", padx=(6, 0))
        ttk.Entry(row, textvariable=self.fail_threshold,
                  width=5).pack(side="left", padx=3)

        # ============ СОХРАНЕНИЕ ============
        save_frame = ttk.LabelFrame(left_frame,
                                    text="Сохранение .txt-копий", padding=6)
        save_frame.pack(fill="x", pady=3)
        row = ttk.Frame(save_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Папка:").pack(side="left")
        ttk.Entry(row, textvariable=self.folder_path,
                  width=28).pack(side="left", padx=3, fill="x", expand=True)
        ttk.Button(row, text="...", width=3,
                   command=self.browse_folder).pack(side="left")
        row = ttk.Frame(save_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Имя файла:").pack(side="left")
        ttk.Entry(row, textvariable=self.file_basename,
                  width=28).pack(side="left", padx=3, fill="x", expand=True)

        # ============ РЕЖИМ РАБОТЫ ============
        mode_frame = ttk.LabelFrame(left_frame, text="Режим работы", padding=6)
        mode_frame.pack(fill="x", pady=3)

        ttk.Radiobutton(mode_frame, text="0 – Спектр в точке",
                        value=0, variable=self.mode_var,
                        command=self.update_mode).pack(anchor="w")
        ttk.Radiobutton(mode_frame, text="1 – Сканирование (3D карта)",
                        value=1, variable=self.mode_var,
                        command=self.update_mode).pack(anchor="w")
        ttk.Radiobutton(mode_frame, text="2 – Задание позиции подвижки",
                        value=2, variable=self.mode_var,
                        command=self.update_mode).pack(anchor="w")
        ttk.Radiobutton(mode_frame,
                        text="3 – Спектры при разных токах/напряжениях (Keithley)",
                        value=3, variable=self.mode_var,
                        command=self.update_mode).pack(anchor="w")
        ttk.Radiobutton(mode_frame,
                        text="4 – Измерение ВАХ (Keithley, без OPUS)",
                        value=4, variable=self.mode_var,
                        command=self.update_mode).pack(anchor="w")

        # ============ ИСПОЛЬЗОВАНИЕ KEITHLEY ============
        k_frame = ttk.LabelFrame(left_frame, text="Keithley 2400", padding=6)
        k_frame.pack(fill="x", pady=3)
        ttk.Checkbutton(k_frame,
                        text="Использовать Keithley в измерениях спектров",
                        variable=self.use_keithley_var,
                        command=self.on_use_keithley_toggle).pack(anchor="w")
        ttk.Label(k_frame,
                  text="Все настройки Keithley — в отдельном окне.",
                  foreground="gray",
                  font=("TkDefaultFont", 8, "italic")).pack(anchor="w", pady=(4, 0))

        # ============ УПРАВЛЕНИЕ ПОДВИЖКОЙ ============
        motion_frame = ttk.LabelFrame(left_frame,
                                      text="Управление подвижкой (µm)",
                                      padding=6)
        motion_frame.pack(fill="x", pady=3)
        row = ttk.Frame(motion_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="V (мкм/с):").pack(side="left")
        ttk.Entry(row, textvariable=self.velocity_um, width=7).pack(side="left", padx=3)
        ttk.Label(row, text="A (мкм/с²):").pack(side="left", padx=(6, 0))
        ttk.Entry(row, textvariable=self.acceleration_um, width=7).pack(side="left", padx=3)
        ttk.Button(row, text="Применить",
                   command=self.apply_motion_params).pack(side="left", padx=3)

        row = ttk.Frame(motion_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Позиция (µm):").pack(side="left")
        ttk.Entry(row, textvariable=self.target_um, width=10).pack(side="left", padx=3)
        ttk.Button(row, text="Перейти",
                   command=self.do_move).pack(side="left", padx=2)
        ttk.Button(row, text="Home",
                   command=self.do_home).pack(side="left", padx=2)

        row = ttk.Frame(motion_frame)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text="Текущая:").pack(side="left")
        ttk.Label(row, textvariable=self.current_pos_str,
                  font=("TkDefaultFont", 10, "bold"),
                  foreground="darkgreen").pack(side="left", padx=3)
        ttk.Label(row, text="µm").pack(side="left")
        ttk.Button(row, text="Обновить",
                   command=self.refresh_position).pack(side="left", padx=3)

        # ============ ПАРАМЕТРЫ РЕЖИМА ============
        self.param_frame = ttk.LabelFrame(left_frame,
                                          text="Параметры режима", padding=6)
        self.param_frame.pack(fill="x", pady=3)

        # ============ КНОПКИ ============
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(pady=6)
        self.start_btn = ttk.Button(btn_frame, text="Старт",
                                    command=self.start_measurement)
        self.start_btn.pack(side="left", padx=5)
        self.stop_btn = ttk.Button(btn_frame, text="Стоп",
                                   command=self.stop_measurement,
                                   state="disabled")
        self.stop_btn.pack(side="left", padx=5)

        self.progress = ttk.Progressbar(left_frame, length=200,
                                         mode="determinate")
        self.progress.pack(fill="x", pady=(3, 0))

        self.point_status_label = ttk.Label(
            left_frame, textvariable=self.point_status,
            foreground="darkblue",
            font=("TkDefaultFont", 9, "bold"))
        self.point_status_label.pack(fill="x", pady=(0, 3))

        log_frame = ttk.LabelFrame(left_frame, text="Журнал", padding=5)
        log_frame.pack(fill="both", expand=True, pady=3)
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8,
                                                  state="disabled")
        self.log_text.pack(fill="both", expand=True)

        # ============ ПРАВАЯ ПАНЕЛЬ ============
        self.right_paned = ttk.PanedWindow(right_frame, orient=tk.VERTICAL)
        self.right_paned.pack(fill="both", expand=True)

        top_frame = ttk.LabelFrame(self.right_paned,
                                   text="Текущий график", padding=5)
        self.right_paned.add(top_frame, weight=1)

        self.fig_spectrum = Figure(figsize=(6, 3), dpi=100,
                                   constrained_layout=True)
        self.ax_spectrum = self.fig_spectrum.add_subplot(111)
        self.canvas_spectrum = FigureCanvasTkAgg(self.fig_spectrum,
                                                 master=top_frame)
        self.canvas_spectrum.get_tk_widget().pack(fill="both", expand=True)
        self.ax_spectrum.set_xlabel("Энергия фотонов (мэВ)")
        self.ax_spectrum.set_ylabel("Интенсивность")
        self.ax_spectrum.grid(True, alpha=0.3)

        self.bottom_frame = ttk.LabelFrame(
            self.right_paned,
            text="3D карта (X: позиция, Y: энергия фотонов в мэВ, Z: интенсивность)",
            padding=5)

        axis_ctrl = ttk.Frame(self.bottom_frame)
        axis_ctrl.pack(side="top", fill="x")
        ttk.Label(axis_ctrl, text="X мин:").grid(row=0, column=0, padx=2)
        ttk.Entry(axis_ctrl, textvariable=self.axis_x_min, width=7).grid(row=0, column=1)
        ttk.Label(axis_ctrl, text="X макс:").grid(row=0, column=2, padx=2)
        ttk.Entry(axis_ctrl, textvariable=self.axis_x_max, width=7).grid(row=0, column=3)
        ttk.Label(axis_ctrl, text="Y мин:").grid(row=0, column=4, padx=(8, 2))
        ttk.Entry(axis_ctrl, textvariable=self.axis_y_min, width=7).grid(row=0, column=5)
        ttk.Label(axis_ctrl, text="Y макс:").grid(row=0, column=6, padx=2)
        ttk.Entry(axis_ctrl, textvariable=self.axis_y_max, width=7).grid(row=0, column=7)
        ttk.Label(axis_ctrl, text="Z мин:").grid(row=0, column=8, padx=(8, 2))
        ttk.Entry(axis_ctrl, textvariable=self.axis_z_min, width=7).grid(row=0, column=9)
        ttk.Label(axis_ctrl, text="Z макс:").grid(row=0, column=10, padx=2)
        ttk.Entry(axis_ctrl, textvariable=self.axis_z_max, width=7).grid(row=0, column=11)
        ttk.Checkbutton(axis_ctrl, text="Автомасштаб",
                        variable=self.autoscale_3d).grid(row=0, column=12, padx=8)
        ttk.Button(axis_ctrl, text="Применить",
                   command=self.apply_axes_3d).grid(row=0, column=13, padx=4)

        row_thr = ttk.Frame(self.bottom_frame)
        row_thr.pack(side="top", fill="x", pady=(2, 0))
        ttk.Label(row_thr, text="Обновлять 3D каждые N точек:").pack(side="left")
        ttk.Entry(row_thr, textvariable=self.map_update_every,
                  width=5).pack(side="left", padx=3)

        self.fig_3d = Figure(figsize=(6, 4), dpi=100,
                             constrained_layout=True)
        self.ax_3d = self.fig_3d.add_subplot(111)
        self.canvas_3d = FigureCanvasTkAgg(self.fig_3d,
                                           master=self.bottom_frame)
        self.canvas_3d.get_tk_widget().pack(fill="both", expand=True)
        self.ax_3d.set_xlabel("Позиция (мкм)")
        self.ax_3d.set_ylabel("Энергия фотонов (мэВ)")
        self.ax_3d.set_title("3D карта интенсивности")

        self._bottom_pane_visible = False

    # ------------------------------------------------------------
    def set_indicator(self, dev, color):
        mapping = {
            "motion": (self.motion_ind_canvas, self.motion_circle),
            "keithley": (self.keithley_ind_canvas, self.keithley_circle),
            "opus": (self.opus_ind_canvas, self.opus_circle),
        }
        if dev in mapping:
            mapping[dev][0].itemconfig(mapping[dev][1], fill=color)

    def browse_opus_exe(self):
        path = filedialog.askopenfilename(
            title="Выберите OPUS.exe",
            filetypes=[("OPUS executable", "*.exe"), ("All files", "*.*")])
        if path:
            self.opus_exe_path.set(path)
            if self.recovery is not None:
                self.recovery.exe_path = path

    def apply_recovery_settings(self):
        if self.recovery is not None:
            self.recovery.set_enabled(self.use_recovery.get())
            try:
                self.recovery.ping_interval = max(3, int(self.ping_interval.get()))
                self.recovery.fail_threshold = max(1, int(self.fail_threshold.get()))
            except Exception:
                pass
            if self.opus_exe_path.get().strip():
                self.recovery.exe_path = self.opus_exe_path.get().strip()

    # ------------------------------------------------------------
    def clear_3d_map(self):
        if self.cbar_3d is not None:
            try:
                self.cbar_3d.remove()
            except Exception:
                pass
            self.cbar_3d = None
        self.im_3d = None
        try:
            self.ax_3d.clear()
        except Exception:
            pass
        try:
            self.ax_3d.set_xlabel("Позиция (мкм)")
            self.ax_3d.set_ylabel("Энергия фотонов (мэВ)")
            self.ax_3d.set_title("3D карта интенсивности")
        except Exception:
            pass
        self.last_2d_data = None
        try:
            self.canvas_3d.draw_idle()
        except Exception:
            pass

    def refresh_com_ports(self):
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.com_combo['values'] = ports
            if ports and not self.com_port.get():
                self.com_port.set(ports[0])
        except Exception as e:
            self.log(f"COM порты: {e}")

    def refresh_keithley_ports(self):
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.k_combo['values'] = ports
            if self.keithley_window is not None:
                try:
                    self.keithley_window.combo['values'] = ports
                except Exception:
                    pass
            if ports and not self.keithley_port.get():
                self.keithley_port.set(ports[-1] if len(ports) > 1 else ports[0])
        except Exception:
            pass

    def connect_motion(self):
        port = self.com_port.get()
        if not port:
            messagebox.showerror("Ошибка", "Выберите COM-порт")
            return
        if self.motion is not None:
            try:
                self.motion.disconnect()
            except Exception:
                pass
            self.motion = None
        driver = self.driver_var.get()
        try:
            self.log(f"Подключение {driver} на {port}...")
            if driver == "ESP300":
                scale = float(self.unit_scale.get())
                self.motion = ESP300Motion(
                    port, log_func=self.log, unit_scale=scale,
                    velocity_um=self.velocity_um.get(),
                    accel_um=self.acceleration_um.get())
            elif driver == "SMC100":
                self.motion = SMC100Motion(port, log_func=self.log)
            self.motion_status.set(f"Подключено ({driver})")
            self.motion_label.config(foreground="green")
            self.set_indicator("motion", "green")
            self.log(f"Подвижка: подключён {driver}")
            self.refresh_position()
        except Exception as e:
            self.log(f"Подвижка: ошибка — {e}")
            self.motion_status.set("Ошибка")
            self.motion_label.config(foreground="red")
            self.set_indicator("motion", "red")
            self.motion = None

    # ------------------------------------------------------------
    #  Keithley
    # ------------------------------------------------------------
    def on_use_keithley_toggle(self):
        if self.use_keithley_var.get():
            self.toggle_keithley_window()
        else:
            if self.keithley_window is not None:
                self.keithley_window.withdraw()

    def toggle_keithley_window(self):
        if self.keithley_window is None:
            self.keithley_window = KeithleyWindow(self)
        self.keithley_window.deiconify()
        self.keithley_window.lift()

    def connect_keithley(self):
        port = self.keithley_port.get()
        if not port:
            messagebox.showerror("Keithley", "Выберите COM-порт")
            return
        if self.keithley is not None:
            self.disconnect_keithley()
        try:
            self.keithley = Keithley2400(
                port, log_func=lambda m: self.data_queue.put(("log", m)))
            # Применить текущий compliance (40 µA / 40 µV)
            try:
                self.keithley.set_source_function(
                    self.keithley_source_mode.get())
                self.keithley.set_compliance(
                    float(self.keithley_compliance.get()))
            except Exception:
                pass
            self.keithley_status.set("Подключено")
            self.k_label.config(foreground="green")
            self.set_indicator("keithley", "green")
            self.log(f"Keithley 2400 подключён на {port}")
            self._start_k_poll()
        except Exception as e:
            self.keithley_status.set("Ошибка")
            self.k_label.config(foreground="red")
            self.set_indicator("keithley", "red")
            self.keithley = None
            messagebox.showerror("Keithley", f"Ошибка подключения: {e}")

    def disconnect_keithley(self):
        self._stop_k_poll()
        if self.keithley is not None:
            try:
                self.keithley.close()
            except Exception:
                pass
            self.keithley = None
        self.keithley_status.set("Не подключено")
        self.k_label.config(foreground="red")
        self.set_indicator("keithley", "gray")
        self.keithley_v_var.set("-- V")
        self.keithley_i_var.set("-- A")
        self.keithley_status_var.set("--")

    def _update_compliance_label(self):
        if self.compliance_label is None:
            return
        if self.keithley_source_mode.get() == "VOLT":
            self.compliance_label.config(text="Предел по току, А:")
        else:
            self.compliance_label.config(text="Предел по напряжению, В:")

    def _start_k_poll(self):
        if self._k_poll_active:
            return
        self._k_poll_active = True
        self._k_poll_stop = threading.Event()
        self._k_poll_thread = threading.Thread(
            target=self._poll_k_loop, daemon=True)
        self._k_poll_thread.start()

    def _stop_k_poll(self):
        self._k_poll_active = False
        if self._k_poll_stop is not None:
            self._k_poll_stop.set()
        self._k_poll_stop = None
        self._k_poll_thread = None

    def _poll_k_loop(self):
        while not self._k_poll_stop.is_set():
            if self.keithley is None:
                break
            measuring = (self.measurement_thread is not None
                         and self.measurement_thread.is_alive())
            output_on = self.keithley_output_on.get()
            if (not measuring) and output_on:
                try:
                    v, i = self.keithley.read_last()
                    if v is not None and i is not None:
                        self.data_queue.put(
                            ("k_rates", (f"{v:.4g} V", f"{i:.4g} A")))
                except Exception:
                    pass
                try:
                    st = self.keithley.get_compliance_status()
                    self.data_queue.put(("k_status", st))
                except Exception:
                    pass
            else:
                self.data_queue.put(("k_rates", ("-- V", "-- A")))
                self.data_queue.put(("k_status", "--"))
            self._k_poll_stop.wait(K_POLL_INTERVAL_S)

    def _keithley_apply_settings(self):
        if self.keithley is None:
            messagebox.showerror("Keithley", "Прибор не подключён")
            return
        try:
            mode = self.keithley_source_mode.get()
            self.keithley.set_source_function(mode)
            self.keithley.set_source_level(self.keithley_level.get())
            self.keithley.set_compliance(self.keithley_compliance.get())
            self.log(f"Настройки Keithley применены "
                     f"(compliance={self.keithley_compliance.get():.3e})")
        except Exception as e:
            messagebox.showerror("Keithley", f"Ошибка: {e}")

    def _keithley_toggle_output(self):
        if self.keithley is None:
            messagebox.showerror("Keithley", "Прибор не подключён")
            self.keithley_output_on.set(False)
            return
        try:
            if self.keithley_output_on.get():
                self.keithley.set_source_function(
                    self.keithley_source_mode.get())
                self.keithley.set_source_level(self.keithley_level.get())
                self.keithley.set_compliance(self.keithley_compliance.get())
                self.keithley.output_on()
                self.log("Keithley: выход включён")
            else:
                self.keithley.output_off()
                self.log("Keithley: выход выключен")
        except Exception as e:
            self.log(f"Keithley: ошибка управления выходом: {e}")

    def _get_keithley_values(self):
        mode = self.keithley_source_mode.get()
        if self.keithley_measure_fixed.get():
            return [(mode, self.keithley_level.get())]
        start = self.keithley_sweep_start.get()
        end = self.keithley_sweep_end.get()
        step = self.keithley_sweep_step.get()
        if step == 0:
            return [(mode, start)]
        n = int(math.ceil(abs(end - start) / abs(step))) + 1
        if end >= start:
            values = [start + i * step for i in range(n)]
        else:
            values = [start - i * step for i in range(n)]
        return [(mode, v) for v in values]

    # ------------------------------------------------------------
    #  ВАХ: измерение и сохранение
    # ------------------------------------------------------------
    def _write_vah_info(self, path, mode, compliance,
                        start, end, step, n_points, bidirectional):
        try:
            lines = [
                "VAH info",
                "=" * 40,
                f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
                f"Source mode: {mode}",
                f"Source unit: {'V' if mode == 'VOLT' else 'A'}",
                f"Compliance: {compliance} "
                f"{'A' if mode == 'VOLT' else 'V'}",
                f"Start: {start}",
                f"End: {end}",
                f"Step: {step}",
                f"Points (including return if bidirectional): {n_points}",
                f"Bidirectional: {bidirectional}",
            ]
            with open(path, "w", encoding="utf-8") as f:
                f.write("\n".join(lines) + "\n")
        except Exception as e:
            self.data_queue.put(("log", f"VAH info.txt: {e}"))

    def run_keithley_measurement(self):
        if self.keithley is None:
            self.data_queue.put(("error", "Keithley 2400 не подключён"))
            return
        threading.Thread(target=self._run_vah_thread, daemon=True).start()

    def _run_vah_thread(self):
        folder = self.folder_path.get()
        try:
            os.makedirs(folder, exist_ok=True)
        except Exception as e:
            self.data_queue.put(("error", f"Не создать папку: {e}"))
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_path = os.path.join(folder, f"VAH_{ts}.txt")
        info_path = os.path.join(folder, f"VAH_{ts}_info.txt")

        mode = self.keithley_source_mode.get()
        unit_src = 'V' if mode == 'VOLT' else 'A'
        unit_meas = 'A' if mode == 'VOLT' else 'V'
        compliance = self.keithley_compliance.get()
        bidirectional = bool(self.keithley_bidirectional_var.get())

        try:
            self.keithley.set_source_function(mode)
            self.keithley.set_compliance(compliance)
        except Exception as e:
            self.data_queue.put(("error", f"Keithley (настройка): {e}"))
            return

        # --- фиксированный уровень ---
        if self.keithley_measure_fixed.get():
            level = self.keithley_level.get()
            try:
                self.keithley.set_source_level(level)
                self.keithley.output_on()
                time.sleep(0.5)
                v, i = self.keithley.read_v_and_i()
                if v is None or i is None:
                    v, i = 0.0, 0.0
                self.data_queue.put(("log",
                    f"Keithley: {level:.6g} {unit_src} -> "
                    f"V={v:.6g}, I={i:.6g}"))
                self.data_queue.put(("plot_keithley",
                                     ([level],
                                      [v if mode == 'VOLT' else i],
                                      f"{level:.4g} {unit_src} (фиксир.)")))
                self.keithley.output_off()
                with open(data_path, "w", encoding="utf-8") as f:
                    f.write("# V\tI\n")
                    f.write(f"{v:.8e}\t{i:.8e}\n")
                self._write_vah_info(info_path, mode, compliance,
                                     level, level, 0.0, 1, bidirectional)
                self.data_queue.put(("log", f"ВАХ сохранена: {data_path}"))
            except Exception as e:
                self.data_queue.put(("error", f"Keithley: {e}"))
            return

        # --- свип ---
        start = self.keithley_sweep_start.get()
        end = self.keithley_sweep_end.get()
        step = self.keithley_sweep_step.get()
        delay = self.keithley_sweep_delay.get()

        if step == 0:
            self.data_queue.put(("error", "Шаг свипа = 0"))
            return

        n = int(math.ceil(abs(end - start) / abs(step))) + 1
        forward = [start + i * step if end >= start else start - i * step
                   for i in range(n)]
        if bidirectional:
            sequence = forward + list(reversed(forward[:-1]))
        else:
            sequence = forward

        total = len(sequence)
        self.data_queue.put(("log",
            f"ВАХ: свип {start:.4g} → {end:.4g} {unit_src}, "
            f"{total} точек, compliance {compliance} {unit_meas}"))
        self.data_queue.put(("progress", (0, total)))

        xs, ys = [], []
        try:
            self.keithley.set_source_level(sequence[0])
            self.keithley.output_on()
            time.sleep(0.2)
            with open(data_path, "w", encoding="utf-8") as f:
                f.write("# V\tI\n")
                for idx, x in enumerate(sequence, start=1):
                    if self.stop_requested:
                        self.data_queue.put(("log", "ВАХ остановлена"))
                        break
                    self.keithley.set_source_level(x)
                    time.sleep(delay)
                    v, i = self.keithley.read_v_and_i()
                    if v is None or i is None:
                        v, i = 0.0, 0.0
                    xs.append(x)
                    ys.append(v if mode == 'VOLT' else i)
                    f.write(f"{v:.8e}\t{i:.8e}\n")
                    f.flush()
                    self.data_queue.put(("plot_keithley",
                                         (xs.copy(), ys.copy(), "")))
                    self.data_queue.put(("progress", (idx, total)))
            self.keithley.output_off()
            self._write_vah_info(info_path, mode, compliance,
                                 start, end, step, total, bidirectional)
            self.data_queue.put(("log", f"ВАХ сохранена: {data_path}"))
        except Exception as e:
            self.data_queue.put(("error", f"Keithley (свип): {e}"))
            try:
                self.keithley.output_off()
            except Exception:
                pass

    # ------------------------------------------------------------
    def connect_opus(self):
        if self.opus is None:
            self.opus = BrukerOPUS(log_func=self.log)
        if self.opus.check_connection():
            self.opus_status.set("Подключено")
            self.opus_label.config(foreground="green")
            self.set_indicator("opus", "green")
            if self.recovery is None:
                self.recovery = OPUSRecovery(
                    self.opus,
                    exe_path=self.opus_exe_path.get().strip() or None,
                    ping_interval=int(self.ping_interval.get()),
                    fail_threshold=int(self.fail_threshold.get()),
                    log_func=self.log)
            self.recovery.set_enabled(self.use_recovery.get())
            self.recovery.start()
        else:
            self.opus_status.set("Ошибка")
            self.opus_label.config(foreground="red")
            self.set_indicator("opus", "red")

    def check_connections(self):
        self.set_indicator("motion", "gray")
        self.set_indicator("keithley", "gray")
        self.set_indicator("opus", "gray")
        threading.Thread(target=self._check_devices, daemon=True).start()

    def _check_devices(self):
        try:
            if self.keithley is not None:
                idn = self.keithley.query('*IDN?')
                if idn:
                    self.data_queue.put(("indicator", ("keithley", "green")))
                    self.data_queue.put(("log", f"Keithley OK: {idn}"))
                else:
                    self.data_queue.put(("indicator", ("keithley", "red")))
            else:
                self.data_queue.put(("indicator", ("keithley", "gray")))
        except Exception as e:
            self.data_queue.put(("indicator", ("keithley", "red")))
            self.data_queue.put(("log", f"Keithley: {e}"))

        try:
            if self.opus is None:
                self.opus = BrukerOPUS(log_func=self.log)
            if self.opus.check_connection():
                self.data_queue.put(("indicator", ("opus", "green")))
            else:
                self.data_queue.put(("indicator", ("opus", "red")))
        except Exception as e:
            self.data_queue.put(("indicator", ("opus", "red")))
            self.data_queue.put(("log", f"OPUS: {e}"))

        if self.motion is not None:
            self.data_queue.put(("indicator", ("motion", "green")))
        else:
            self.data_queue.put(("indicator", ("motion", "gray")))

    def browse_folder(self):
        path = filedialog.askdirectory(initialdir=self.folder_path.get())
        if path:
            self.folder_path.set(path)

    def apply_motion_params(self):
        if self.motion is None:
            messagebox.showerror("Ошибка", "Подвижка не подключена")
            return
        try:
            self.motion.set_velocity(self.velocity_um.get())
            self.motion.set_acceleration(self.acceleration_um.get())
            self.log(f"V={self.velocity_um.get()} µm/с, "
                     f"A={self.acceleration_um.get()} µm/с²")
        except Exception as e:
            self.log(f"Ошибка: {e}")

    def do_move(self):
        if self.motion is None:
            messagebox.showerror("Ошибка", "Подвижка не подключена")
            return
        try:
            target = float(self.target_um.get())
        except Exception:
            return

        def _worker():
            try:
                self.motion.move_to(target, wait=True)
                pos = self.motion.get_position()
                self.data_queue.put(("position", f"{pos:.3f}"))
            except Exception as e:
                self.data_queue.put(("error", f"Ошибка перемещения: {e}"))
        threading.Thread(target=_worker, daemon=True).start()

    def do_home(self):
        if self.motion is None:
            messagebox.showerror("Ошибка", "Подвижка не подключена")
            return

        def _worker():
            try:
                self.motion.home(wait=True)
                pos = self.motion.get_position()
                self.data_queue.put(("position", f"{pos:.3f}"))
            except Exception as e:
                self.data_queue.put(("error", f"Ошибка homing: {e}"))
        threading.Thread(target=_worker, daemon=True).start()

    def refresh_position(self):
        if self.motion is None:
            self.current_pos_str.set("--")
            return
        try:
            self.current_pos_str.set(f"{self.motion.get_position():.3f}")
        except Exception:
            self.current_pos_str.set("--")

    # ------------------------------------------------------------
    def log(self, msg):
        t = datetime.now().strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{t}] {msg}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def process_queue(self):
        try:
            while True:
                msg = self.data_queue.get_nowait()
                if msg[0] == "log":
                    self.log(msg[1])
                elif msg[0] == "progress":
                    cur, tot = msg[1]
                    self.progress["maximum"] = max(1, tot)
                    self.progress["value"] = cur
                elif msg[0] == "status":
                    self.point_status.set(msg[1])
                elif msg[0] == "spectrum":
                    wn, intensity, pos = msg[1]
                    self.plot_spectrum(wn, intensity, pos)
                elif msg[0] == "plot_keithley":
                    x, y, title = msg[1]
                    self._plot_keithley(x, y, title)
                elif msg[0] == "3d_update":
                    self._rebuild_and_draw_3d()
                elif msg[0] == "finish":
                    self.set_ui_state(False)
                    self.measurement_thread = None
                    self.point_status.set("")
                elif msg[0] == "position":
                    self.current_pos_str.set(msg[1])
                elif msg[0] == "indicator":
                    self.set_indicator(*msg[1])
                elif msg[0] == "k_rates":
                    self.keithley_v_var.set(msg[1][0])
                    self.keithley_i_var.set(msg[1][1])
                elif msg[0] == "k_status":
                    txt = msg[1]
                    self.keithley_status_var.set(txt)
                    try:
                        if self.keithley_status_label is not None:
                            if txt == "OK":
                                self.keithley_status_label.config(
                                    foreground="green")
                            elif txt in ("?", "--"):
                                self.keithley_status_label.config(
                                    foreground="gray")
                            else:
                                self.keithley_status_label.config(
                                    foreground="red")
                    except Exception:
                        pass
                elif msg[0] == "error":
                    messagebox.showerror("Ошибка", msg[1])
        except queue.Empty:
            pass
        if self.closing and self.measurement_thread is None:
            self.destroy()
            return
        self.after(100, self.process_queue)

    # ------------------------------------------------------------
    def update_mode(self):
        if self.driver_var.get() == "ESP300":
            self.esp_options_frame.pack(fill="x", pady=1, before=None)
        else:
            self.esp_options_frame.pack_forget()

        for w in self.param_frame.winfo_children():
            w.destroy()

        mode = self.mode_var.get()

        if mode == 0:
            ttk.Label(self.param_frame,
                      text="Один спектр. Параметры — в OPUS.",
                      foreground="gray").pack(anchor="w")
        elif mode == 1:
            row = ttk.Frame(self.param_frame)
            row.pack(fill="x", pady=1)
            ttk.Label(row, text="Старт (µm):").pack(side="left")
            ttk.Entry(row, textvariable=self.start_um, width=8).pack(side="left", padx=3)
            ttk.Label(row, text="Конец:").pack(side="left", padx=(6, 0))
            ttk.Entry(row, textvariable=self.end_um, width=8).pack(side="left", padx=3)
            ttk.Label(row, text="Шаг:").pack(side="left", padx=(6, 0))
            ttk.Entry(row, textvariable=self.step_um, width=8).pack(side="left", padx=3)
        elif mode == 2:
            ttk.Label(self.param_frame,
                      text="Ручное управление подвижкой (панель выше).",
                      foreground="gray").pack(anchor="w")
        elif mode == 3:
            ttk.Label(self.param_frame,
                      text="Свип по уровням Keithley с измерением спектров.\n"
                           "Уровни — в окне Keithley.",
                      foreground="gray", justify="left").pack(anchor="w")
        elif mode == 4:
            ttk.Label(self.param_frame,
                      text="Измерение ВАХ (Keithley, без OPUS).\n"
                           "Параметры свипа — в окне Keithley.",
                      foreground="gray", justify="left").pack(anchor="w")
            ttk.Button(self.param_frame, text="Измерить ВАХ",
                       command=self.run_keithley_measurement).pack(
                anchor="w", pady=6)

        show_3d = (mode == 1)
        if show_3d and not self._bottom_pane_visible:
            self.right_paned.add(self.bottom_frame, weight=1)
            self._bottom_pane_visible = True
        elif not show_3d and self._bottom_pane_visible:
            self.right_paned.forget(self.bottom_frame)
            self._bottom_pane_visible = False

        self.update_start_button_label()
        self.after(50, self._refresh_scrollregion)
        self.after(150, self._refresh_scrollregion)

    def _refresh_scrollregion(self):
        try:
            self.left_canvas.configure(
                scrollregion=self.left_canvas.bbox("all"))
        except Exception:
            pass

    def update_start_button_label(self):
        mode = self.mode_var.get()
        if mode == 0:
            self.start_btn.config(text="Измерить спектр")
        elif mode == 1:
            self.start_btn.config(text="Старт сканирования")
        elif mode == 2:
            self.start_btn.config(text="Перейти")
        elif mode == 3:
            self.start_btn.config(text="Старт свипа")
        elif mode == 4:
            self.start_btn.config(text="Измерить ВАХ")

    # ------------------------------------------------------------
    def _build_txt_filename(self, pos=None, k_level=None, k_measured=None):
        base = self.file_basename.get().strip() or "spectrum"
        parts = []
        if pos is not None:
            parts.append(f"pos{pos:.2f}um")
        mode = self.keithley_source_mode.get()
        if k_level is not None:
            if mode == "CURR":
                parts.append(f"I{self._fmt_num(k_level)}A")
            else:
                parts.append(f"U{self._fmt_num(k_level)}V")
        if k_measured is not None:
            if mode == "CURR":
                parts.append(f"U{self._fmt_num(k_measured)}V")
            else:
                parts.append(f"I{self._fmt_num(k_measured)}A")
        return f"{base}_{'_'.join(parts)}.txt" if parts else f"{base}.txt"

    @staticmethod
    def _fmt_num(x):
        try:
            xf = float(x)
        except Exception:
            return str(x)
        if abs(xf - round(xf)) < 1e-9:
            return f"{int(round(xf))}"
        return f"{xf:.3f}".rstrip("0").rstrip(".")

    def save_spectrum_to_file(self, wn, intensity, pos=None,
                              k_level=None, k_measured=None):
        try:
            folder = self.folder_path.get()
            os.makedirs(folder, exist_ok=True)
            filename = self._build_txt_filename(pos, k_level, k_measured)
            filepath = os.path.join(folder, filename)
            with open(filepath, "w", encoding="utf-8") as f:
                f.write("# Wavenumber (cm^-1)\tIntensity\n")
                for i in range(len(wn)):
                    f.write(f"{wn[i]:.6f}\t{intensity[i]:.6e}\n")
            self.data_queue.put(("log", f"Сохранено: {filepath}"))
            return filepath
        except Exception as e:
            self.data_queue.put(("log", f"Ошибка сохранения: {e}"))
            return None

    # ------------------------------------------------------------
    def _wait_recovery_if_needed(self, timeout=200):
        if self.recovery is None:
            return True
        if not self.recovery.recovery_event.is_set():
            self.data_queue.put(("status", "OPUS восстанавливается..."))
        return self.recovery.wait_if_recovering(timeout=timeout)

    def _measure_opus_with_recovery(self, max_attempts=3):
        for attempt in range(1, max_attempts + 1):
            if self.stop_requested:
                return None
            self._wait_recovery_if_needed(timeout=200)
            if self.recovery is not None and not self.opus.ping(timeout_ms=3000):
                self.log(f"OPUS не отвечает (попытка {attempt}/{max_attempts})")
                self.recovery.trigger_recovery()
                self._wait_recovery_if_needed(timeout=200)
                continue
            filepath = self.opus.measure_sample()
            if filepath is not None:
                return filepath
            if self.recovery is not None and self.use_recovery.get():
                self.recovery.trigger_recovery()
                self._wait_recovery_if_needed(timeout=200)
            else:
                return None
        return None

    # ------------------------------------------------------------
    def start_measurement(self):
        mode = self.mode_var.get()

        if mode == 4:
            if self.keithley is None:
                messagebox.showerror("Ошибка", "Keithley не подключён.")
                return
            self.stop_requested = False
            self.log("=" * 30 + " СТАРТ ВАХ " + "=" * 30)
            self.run_keithley_measurement()
            return

        if mode != 2 and (self.opus is None or not self.opus.connected):
            messagebox.showerror("Ошибка", "OPUS не подключён.")
            return
        if mode in (1, 2) and self.motion is None:
            messagebox.showerror("Ошибка", "Подвижка не подключена.")
            return
        if (self.use_keithley_var.get() and self.keithley is None
                and mode in (0, 1, 3)):
            messagebox.showerror("Ошибка", "Keithley не подключён.")
            return
        if mode == 3 and not self.use_keithley_var.get():
            messagebox.showerror("Ошибка",
                                 "Для режима 3 включите Keithley.")
            return
        if mode == 2:
            self.do_move()
            return
        if mode == 1:
            if self.step_um.get() <= 0:
                return
            if self.start_um.get() == self.end_um.get():
                return

        self.stop_requested = False
        self.set_ui_state(True)
        self.spectra.clear()
        self.last_2d_data = None
        self.progress["value"] = 0
        self.point_status.set("")
        self.clear_3d_map()
        try:
            self.ax_spectrum.clear()
            self.ax_spectrum.set_xlabel("Энергия фотонов (мэВ)")
            self.ax_spectrum.set_ylabel("Интенсивность")
            self.ax_spectrum.grid(True, alpha=0.3)
            self.canvas_spectrum.draw_idle()
        except Exception:
            pass

        self.log("=" * 30 + f" СТАРТ (режим {mode}) " + "=" * 30)
        target = {0: self.run_single_point, 1: self.run_scan,
                  3: self.run_keithley_sweep}.get(mode)
        if target is None:
            self.set_ui_state(False)
            return
        self.measurement_thread = threading.Thread(target=target, daemon=True)
        self.measurement_thread.start()

    def stop_measurement(self):
        self.stop_requested = True
        self.log("Остановка...")

    def set_ui_state(self, running):
        self.start_btn.config(state="disabled" if running else "normal")
        self.stop_btn.config(state="normal" if running else "disabled")

    # ------------------------------------------------------------
    def _keithley_set_level(self, level):
        if self.keithley is None:
            return None, None
        mode = self.keithley_source_mode.get()
        try:
            if mode == "CURR":
                self.keithley.set_source_function('CURR')
                self.keithley.set_source_level(float(level))
                self.keithley.set_compliance(
                    float(self.keithley_compliance.get()))
            else:
                self.keithley.set_source_function('VOLT')
                self.keithley.set_source_level(float(level))
                self.keithley.set_compliance(
                    float(self.keithley_compliance.get()))
            self.keithley.output_on()
            time.sleep(0.4)
            v, i = self.keithley.read_v_and_i()
            if v is None or i is None:
                v, i = 0.0, 0.0
            st = self.keithley.get_compliance_status()
            measured = v if mode == "CURR" else i
            return measured, st
        except Exception as e:
            self.log(f"Keithley: ошибка установки уровня — {e}")
            return None, None

    # ------------------------------------------------------------
    def run_single_point(self):
        try:
            use_k = self.use_keithley_var.get()
            k_level = None
            k_measured = None
            if use_k:
                k_level = self.keithley_level.get()
                k_measured, st = self._keithley_set_level(k_level)
                if st and st not in ("OK", "?", "--"):
                    self.log(f"Keithley: {st}")
            self.data_queue.put(("status", "Измерение..."))
            filepath = self._measure_opus_with_recovery()
            if filepath is None:
                self.log("Ошибка измерения")
                return
            for _ in range(30):
                if os.path.exists(filepath):
                    break
                time.sleep(0.2)
            wn, intensity = self.opus.read_spectrum(filepath)
            if wn is None:
                return
            self.spectra.append({
                'pos': 0.0, 'wn': wn.copy(),
                'intensity': intensity.copy(), 'label': "точка"
            })
            self.save_spectrum_to_file(wn, intensity, pos=None,
                                       k_level=k_level, k_measured=k_measured)
            self.data_queue.put(("spectrum", (wn, intensity, 0.0)))
            self.log("Спектр измерен")
        except Exception as e:
            self.log(f"ОШИБКА: {e}")
            import traceback
            self.log(traceback.format_exc())
        finally:
            if self.use_keithley_var.get() and self.keithley is not None:
                try:
                    self.keithley.output_off()
                except Exception:
                    pass
            try:
                self.opus._close_connection()
            except Exception:
                pass
            self.data_queue.put(("finish", None))

    # ------------------------------------------------------------
    def run_scan(self):
        try:
            start = self.start_um.get()
            end = self.end_um.get()
            step = self.step_um.get()
            if end >= start:
                positions = np.arange(start, end + step / 2, step)
            else:
                positions = np.arange(start, end - step / 2, -step)

            use_k = self.use_keithley_var.get()

            tasks = []
            if use_k:
                for pos in positions:
                    for m, lvl in self._get_keithley_values():
                        tasks.append((pos, lvl))
            else:
                for pos in positions:
                    tasks.append((pos, None))

            total = len(tasks)
            self.data_queue.put(("progress", (0, total)))
            try:
                every_n = max(1, int(self.map_update_every.get()))
            except Exception:
                every_n = 3

            for idx, (pos, k_level) in enumerate(tasks, start=1):
                if self.stop_requested:
                    break
                self._wait_recovery_if_needed(timeout=300)
                self.data_queue.put(("progress", (idx, total)))
                status = f"Точка {idx}/{total}: {pos:.2f} µm"
                if k_level is not None:
                    status += f", {self.keithley_source_mode.get()}={k_level}"
                self.data_queue.put(("status", status))

                try:
                    self.motion.move_to(pos, wait=True)
                except Exception as e:
                    self.log(f"Перемещение: {e}")
                time.sleep(0.2)
                try:
                    self.data_queue.put(("position",
                                         f"{self.motion.get_position():.3f}"))
                except Exception:
                    pass

                k_measured = None
                if use_k:
                    k_measured, st = self._keithley_set_level(k_level)
                    if st and st not in ("OK", "?", "--"):
                        self.log(f"Keithley: {st}")

                filepath = self._measure_opus_with_recovery()
                if filepath is None:
                    continue
                for _ in range(30):
                    if os.path.exists(filepath):
                        break
                    time.sleep(0.2)
                wn, intensity = self.opus.read_spectrum(filepath)
                if wn is None:
                    continue
                self.spectra.append({
                    'pos': pos, 'wn': wn.copy(),
                    'intensity': intensity.copy(), 'label': f"{pos:.2f} µm"
                })
                self.save_spectrum_to_file(wn, intensity, pos=pos,
                                           k_level=k_level,
                                           k_measured=k_measured)
                self.data_queue.put(("spectrum", (wn, intensity, pos)))
                if idx % every_n == 0 or idx == total:
                    self.data_queue.put(("3d_update", None))

            if not self.stop_requested:
                self.log("Возврат в ноль...")
                try:
                    self.motion.move_to(0.0, wait=True)
                except Exception:
                    pass
            self.data_queue.put(("3d_update", None))
            self.data_queue.put(("status", "Сканирование завершено"))
        except Exception as e:
            self.log(f"ОШИБКА: {e}")
        finally:
            if self.use_keithley_var.get() and self.keithley is not None:
                try:
                    self.keithley.output_off()
                except Exception:
                    pass
            try:
                self.opus._close_connection()
            except Exception:
                pass
            self.data_queue.put(("finish", None))

    # ------------------------------------------------------------
    def run_keithley_sweep(self):
        try:
            if self.keithley is None:
                self.log("Keithley не подключён")
                return
            values = self._get_keithley_values()
            total = len(values)
            self.data_queue.put(("progress", (0, total)))
            self.log(f"Свип по {self.keithley_source_mode.get()}: {total}")

            for idx, (kmode, lvl) in enumerate(values, start=1):
                if self.stop_requested:
                    break
                self._wait_recovery_if_needed(timeout=300)
                self.data_queue.put(("progress", (idx, total)))
                self.data_queue.put(("status",
                                     f"Свип {idx}/{total}: "
                                     f"{self.keithley_source_mode.get()}={lvl}"))

                k_measured, st = self._keithley_set_level(lvl)
                if k_measured is not None:
                    if self.keithley_source_mode.get() == "CURR":
                        self.log(f"Keithley: I={lvl} A, "
                                 f"U={k_measured:.4f} V")
                    else:
                        self.log(f"Keithley: U={lvl} V, "
                                 f"I={k_measured:.4f} A")
                if st and st not in ("OK", "?", "--"):
                    self.log(f"Keithley status: {st}")

                filepath = self._measure_opus_with_recovery()
                if filepath is None:
                    continue
                for _ in range(30):
                    if os.path.exists(filepath):
                        break
                    time.sleep(0.2)
                wn, intensity = self.opus.read_spectrum(filepath)
                if wn is None:
                    continue
                self.spectra.append({
                    'pos': 0.0, 'wn': wn.copy(),
                    'intensity': intensity.copy(),
                    'label': f"{self.keithley_source_mode.get()}={lvl}"
                })
                self.save_spectrum_to_file(wn, intensity, pos=None,
                                           k_level=lvl, k_measured=k_measured)
                self.data_queue.put(("spectrum", (wn, intensity, 0.0)))

            self.data_queue.put(("status", "Свип завершён"))
        except Exception as e:
            self.log(f"ОШИБКА: {e}")
        finally:
            if self.keithley is not None:
                try:
                    self.keithley.output_off()
                except Exception:
                    pass
            try:
                self.opus._close_connection()
            except Exception:
                pass
            self.data_queue.put(("finish", None))

    # ------------------------------------------------------------
    def plot_spectrum(self, wn, intensity, pos):
        self.ax_spectrum.clear()
        energy_mev = wn * WAVENUMBER_TO_MEV
        sort_idx = np.argsort(energy_mev)
        self.ax_spectrum.plot(energy_mev[sort_idx], intensity[sort_idx],
                              'b-', linewidth=1.2)
        self.ax_spectrum.set_xlabel("Энергия фотонов (мэВ)")
        self.ax_spectrum.set_ylabel("Интенсивность")
        title = "Спектр"
        if pos != 0.0:
            title += f" при позиции {pos:.2f} µm"
        self.ax_spectrum.set_title(title)
        self.ax_spectrum.grid(True, alpha=0.3)
        self.canvas_spectrum.draw_idle()

    def _plot_keithley(self, x, y, title):
        self.ax_spectrum.clear()
        self.ax_spectrum.plot(x, y, 'r.-', markersize=5)
        mode = self.keithley_source_mode.get()
        if mode == "VOLT":
            self.ax_spectrum.set_xlabel("Напряжение, В")
            self.ax_spectrum.set_ylabel("Ток, А")
        else:
            self.ax_spectrum.set_xlabel("Ток, А")
            self.ax_spectrum.set_ylabel("Напряжение, В")
        self.ax_spectrum.set_title(title or "Keithley 2400 — ВАХ")
        self.ax_spectrum.grid(True, alpha=0.3)
        self.canvas_spectrum.draw_idle()

    def _rebuild_and_draw_3d(self):
        scan_spectra = [s for s in self.spectra if s['pos'] != 0.0]
        if not scan_spectra:
            return
        try:
            positions = np.array([s['pos'] for s in scan_spectra])
            wn_ref = scan_spectra[0]['wn']
            energy_mev = wn_ref * WAVENUMBER_TO_MEV
            sort_idx = np.argsort(energy_mev)
            energy_sorted = energy_mev[sort_idx]

            matrix = np.zeros((len(positions), len(energy_sorted)))
            for i, spec in enumerate(scan_spectra):
                if len(spec['wn']) == len(wn_ref) and np.allclose(spec['wn'], wn_ref):
                    matrix[i, :] = spec['intensity'][sort_idx]
                else:
                    matrix[i, :] = np.interp(
                        energy_sorted,
                        spec['wn'] * WAVENUMBER_TO_MEV,
                        spec['intensity'])

            self.last_2d_data = (matrix, energy_sorted, positions)

            if self.cbar_3d is not None:
                try:
                    self.cbar_3d.remove()
                except Exception:
                    pass
                self.cbar_3d = None

            self.ax_3d.clear()
            extent = [positions.min(), positions.max(),
                      energy_sorted.min(), energy_sorted.max()]
            self.im_3d = self.ax_3d.imshow(
                matrix.T, aspect='auto', origin='lower',
                extent=extent, cmap='viridis')
            self.cbar_3d = self.fig_3d.colorbar(self.im_3d, ax=self.ax_3d,
                                                label='Интенсивность')
            self.ax_3d.set_xlabel("Позиция (мкм)")
            self.ax_3d.set_ylabel("Энергия фотонов (мэВ)")
            self.ax_3d.set_title("3D карта интенсивности")
            self.apply_axes_3d()
            self.canvas_3d.draw_idle()
        except Exception as e:
            self.log(f"Ошибка отрисовки 3D: {e}")

    def update_3d_map(self):
        self._rebuild_and_draw_3d()

    def apply_axes_3d(self):
        if self.last_2d_data is None:
            return
        matrix, energy, positions = self.last_2d_data
        if self.autoscale_3d.get():
            self.ax_3d.set_xlim(positions.min(), positions.max())
            self.ax_3d.set_ylim(energy.min(), energy.max())
            if self.cbar_3d is not None:
                self.cbar_3d.mappable.autoscale()
        else:
            try:
                def _g(v):
                    s = v.get()
                    return float(s) if s else None
                xmin, xmax = _g(self.axis_x_min), _g(self.axis_x_max)
                ymin, ymax = _g(self.axis_y_min), _g(self.axis_y_max)
                zmin, zmax = _g(self.axis_z_min), _g(self.axis_z_max)
                if xmin is not None: self.ax_3d.set_xlim(left=xmin)
                if xmax is not None: self.ax_3d.set_xlim(right=xmax)
                if ymin is not None: self.ax_3d.set_ylim(bottom=ymin)
                if ymax is not None: self.ax_3d.set_ylim(top=ymax)
                if self.cbar_3d is not None:
                    if zmin is not None: self.cbar_3d.mappable.set_clim(vmin=zmin)
                    if zmax is not None: self.cbar_3d.mappable.set_clim(vmax=zmax)
            except ValueError:
                pass
        self.canvas_3d.draw_idle()

    # ------------------------------------------------------------
    def on_closing(self):
        if self.measurement_thread and self.measurement_thread.is_alive():
            self.closing = True
            self.stop_requested = True
            self.log("Завершение...")
        else:
            if self.recovery is not None:
                try:
                    self.recovery.stop()
                except Exception:
                    pass
            self._stop_k_poll()
            if self.keithley is not None:
                try:
                    self.keithley.close()
                except Exception:
                    pass
                self.keithley = None
            if self.motion is not None:
                try:
                    self.motion.disconnect()
                except Exception:
                    pass
                self.motion = None
            if self.opus is not None:
                self.opus.disconnect()
            try:
                if self.keithley_window is not None:
                    self.keithley_window.destroy()
            except Exception:
                pass
            self.destroy()


# ------------------------------------------------------------
if __name__ == "__main__":
    app = Application()
    app.mainloop()
