#!/usr/bin/env python3
"""
main.py
- LCD I2C PCF8574 @ 0x27
- ADS1115 @ 0x48 (A0 -> MQ2, A1 -> Micro KI037)
- DHT11 @ GPIO4
- Read every 5s, display on 16x2 LCD
"""

import time
import math
import smbus
import RPi.GPIO as GPIO

import json
# ----------------------------
# Config addresses / pins
# ----------------------------
# --- GIT / JSON helpers for GitHub Pages (SSH auto-push) ---
import os
import subprocess
import threading

GIT_REPO_PATH = "/home/pi/iot-dashboard"   # du?ng d?n repo c?a b?n
PUSH_INTERVAL = 60  # push m?i 60 gi�y

_last_push = 0
_pending_changes = False
_git_lock = threading.Lock()

def _git_push():
    global _pending_changes, _last_push
    with _git_lock:
        try:
            subprocess.run(["git", "-C", GIT_REPO_PATH, "add", "."], check=True)
            subprocess.run([
                "git", "-C", GIT_REPO_PATH, "commit", "-m", "auto update", "--allow-empty"
            ], check=True)
            subprocess.run(["git", "-C", GIT_REPO_PATH, "push"], check=True)
            print("[GIT] push OK")
            _pending_changes = False
            _last_push = time.time()
        except Exception as e:
            print("[GIT] push FAILED:", e)

def schedule_push_if_due():
    global _pending_changes, _last_push
    if not _pending_changes:
        return
    if time.time() - _last_push < PUSH_INTERVAL:
        return
    threading.Thread(target=_git_push, daemon=True).start()

def save_point(filename, ts, value):
    global _pending_changes
    path = os.path.join(GIT_REPO_PATH, filename)
    try:
        if os.path.exists(path):
            with open(path, "r") as f:
                arr = json.load(f)
        else:
            arr = []
    except:
        arr = []

    arr.append({"ts": ts, "value": value})

    try:
        with open(path, "w") as f:
            json.dump(arr, f)
        _pending_changes = True
    except Exception as e:
        print("save_point error:", e)

I2C_BUS = 1
LCD_ADDR = 0x27
ADS_ADDR = 0x48
DHT_PIN = 17
SLEEP_INTERVAL = 5.0
# C?u h�nh ThingsBoard � thay b?ng th�ng tin thi?t b? c?a b?n
THINGSBOARD_HOST = "mqtt.thingsboard.cloud"   # ho?c d?a ch? ThingsBoard server c?a b?n
ACCESS_TOKEN = "tCRnF8Y71NydhzpiAhHX" # <- d?i th�nh device access token c?a b?n

# ----------------------------
# ADS1115 minimal driver (no libs)
# ----------------------------
class ADS1115:
    REG_CONVERSION = 0x00
    REG_CONFIG = 0x01

    # PGA gain settings (we use gain=1 -> FS �4.096V)
    GAIN_1 = 0x0200

    # Single-shot mode flag
    OS_SINGLE = 0x8000

    # Data rate: 128SPS default or set to 860 (faster)
    DR_128SPS = 0x0080  # lower power, fine for us
    DR_860SPS = 0x00E0

    # Mode single-shot
    MODE_SINGLE = 0x0100

    def __init__(self, bus=I2C_BUS, addr=ADS_ADDR):
        self.addr = addr
        self.bus = smbus.SMBus(bus)
        self.gain = self.GAIN_1
        self.data_rate = self.DR_128SPS

    def read_single(self, channel):
        # channel: 0..3
        mux = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}[channel]
        config = (
            self.OS_SINGLE |  # start single conversion
            mux |
            self.gain |
            self.MODE_SINGLE |
            self.data_rate |
            0x0003  # comparator off
        )
        # write config (big-endian)
        cfg_hi = (config >> 8) & 0xFF
        cfg_lo = config & 0xFF
        try:
            self.bus.write_i2c_block_data(self.addr, self.REG_CONFIG, [cfg_hi, cfg_lo])
        except Exception as e:
            raise IOError("I2C write to ADS1115 failed: " + str(e))

        # wait conversion (depends on data_rate); 0.01s is safe for 128SPS
        time.sleep(0.01)

        # read conversion register (2 bytes)
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.REG_CONVERSION, 2)
        except Exception as e:
            raise IOError("I2C read from ADS1115 failed: " + str(e))

        raw = (data[0] << 8) | data[1]
        # signed conversion
        if raw & 0x8000:
            raw -= 1 << 16
        return raw

    def raw_to_voltage(self, raw):
        # For gain=1 FS = 4.096V, raw is -32768..32767
        fs = 4.096
        return float(raw) * fs / 32767.0

# ----------------------------
# LCD driver for PCF8574 @ 0x27
# stable implementation with fallback
# ----------------------------
class LCD1602_RP:
    LCD_CMD = 0
    LCD_CHR = 1
    LINE_1 = 0x80
    LINE_2 = 0xC0
    BACKLIGHT = 0x08
    ENABLE = 0x04
    def __init__(self, addr=LCD_ADDR, bus=I2C_BUS):
        self.addr = addr
        self.bus = smbus.SMBus(bus)
        self.enabled = True
        # try probe
        try:
            # simple write to probe device; if fails, disable LCD fallback
            self.bus.write_byte(self.addr, 0x00)
        except Exception as e:
            print("Warning: Cannot reach LCD at 0x{:02x}, fallback to console. ({})".format(self.addr, e))
            self.enabled = False
            return
        time.sleep(0.05)
        try:
            self._init()
        except Exception as e:
            print("LCD init failed, disabling LCD. ({})".format(e))
            self.enabled = False

    def _write_byte(self, val):
        # wrapper with small retry
        tries = 3
        for _ in range(tries):
            try:
                self.bus.write_byte(self.addr, val)
                return
            except Exception:
                time.sleep(0.01)
        raise IOError("LCD I2C write failed after retries")

    def _toggle(self, bits):
        time.sleep(0.0005)
        self._write_byte(bits | self.ENABLE)
        time.sleep(0.0005)
        self._write_byte(bits & ~self.ENABLE)
        time.sleep(0.0005)

    def _send(self, data, mode):
        high = mode | (data & 0xF0) | self.BACKLIGHT
        low  = mode | ((data << 4) & 0xF0) | self.BACKLIGHT
        self._write_byte(high); self._toggle(high)
        self._write_byte(low);  self._toggle(low)

    def command(self, cmd):
        if not self.enabled: return
        self._send(cmd, self.LCD_CMD)

    def write_char(self, ch):
        if not self.enabled: return
        self._send(ord(ch), self.LCD_CHR)

    def _init(self):
        # initialization sequence
        self.command(0x33)
        self.command(0x32)
        self.command(0x06)
        self.command(0x0C)
        self.command(0x28)
        self.command(0x01)
        time.sleep(0.005)

    def clear(self):
        if not self.enabled: return
        self.command(0x01)
        time.sleep(0.002)

    def write_line(self, text, line=1):
        if not self.enabled:
            # fallback to console printing
            if line == 1:
                print("LCD L1:", text)
            else:
                print("LCD L2:", text)
            return
        if line == 1:
            self.command(self.LINE_1)
        else:
            self.command(self.LINE_2)
        for ch in text.ljust(16)[:16]:
            self.write_char(ch)

# ----------------------------
# DHT11 read (simple, may fail sometimes)
# ----------------------------
GPIO.setmode(GPIO.BCM)

def read_dht11_once(pin=DHT_PIN):
    """
    Simplified non-library DHT11 read.
    Returns (temp_c, hum) or (None, None) on fail.
    Implementation: send start, capture pulses, decode bits.
    """
    data = []
    # send start
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
    time.sleep(0.02)  # >=18ms
    GPIO.output(pin, GPIO.HIGH)
    time.sleep(0.00002)
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
      # collect pulses
    count = 0
    last = -1
    timings = []
    # wait for response up to ~0.1s
    timeout = time.time() + 0.2
    while time.time() < timeout:
        current = GPIO.input(pin)
        if current != last:
            timings.append((current, time.time()))
            last = current
        if len(timings) > 1000:
            break

    # decode by measuring high pulse durations for data bits (skip initial response pulses)
    try:
        # extract durations between transitions
        pulses = []
        for i in range(1, len(timings)):
            dt = timings[i][1] - timings[i-1][1]
            pulses.append((timings[i-1][0], dt))
        # high pulses after initial handshake correspond to bits
        high_pulses = [dt for level, dt in pulses if level == 1]
        # We expect at least 40 high pulses for 40 bits
        if len(high_pulses) < 40:
            return None, None
        # take last 40 high pulses
        bits = []
        # pick threshold between short and long pulses
        sample = high_pulses[:40]
        thr = (min(sample) + max(sample)) / 2.0
        for h in sample[:40]:
            bits.append(1 if h > thr else 0)
        # convert bits to bytes
        def bits_to_byte(b):
            return int("".join(str(x) for x in b), 2)
        b = [bits_to_byte(bits[i*8:(i+1)*8]) for i in range(5)]
        checksum = (b[0] + b[1] + b[2] + b[3]) & 0xFF
        if checksum != b[4]:
            return None, None
        hum = b[0]
        temp = b[2]
        return float(temp), float(hum)
    except Exception:
        return None, None
def read_dht11_stable(pin, tries=6, delay=0.3):
    for i in range(tries):
        t, h = read_dht11_once(pin)   # g?i l?i h�m cu
        if t is not None and h is not None:
            return t, h
        time.sleep(delay)
    return None, None
# ----------------------------
# Conversion helpers
# ----------------------------
def mq2_voltage_to_percent(v, v_min=0.2, v_max=3.0):
    if v <= v_min: return 0.0
    if v >= v_max: return 100.0
    return (v - v_min) / (v_max - v_min) * 100.0

def mic_voltage_to_db(v, offset=55.0):
    if v <= 0: return 0.0
    vrms = v / math.sqrt(2.0)
    db = 20.0 * math.log10(vrms) + offset
    return round(db, 1)

# ----------------------------
# Main loop
# ----------------------------
def main():
    print("Starting main. LCD@0x{:02x}, ADS1115@0x{:02x}".format(LCD_ADDR, ADS_ADDR))
    lcd = None
    ads = None
    try:
        lcd = LCD1602_RP(addr=LCD_ADDR, bus=I2C_BUS)
    except Exception as e:
        print("LCD init error:", e)
    try:
        ads = ADS1115(bus=I2C_BUS, addr=ADS_ADDR)
    except Exception as e:
        print("ADS1115 init error:", e)
        ads = None
        


    last_temp, last_hum = 0.0, 0.0

    try:
        while True:
            # 1) �?c ADS1115 tru?c
            gas_percent = 0.0
            mic_db = 0.0

            if ads:
                try:
                    raw_mq2 = ads.read_single(0)
                    raw_mic = ads.read_single(1)
                    v_mq2 = ads.raw_to_voltage(raw_mq2)
                    v_mic = ads.raw_to_voltage(raw_mic)

                    gas_percent = mq2_voltage_to_percent(v_mq2)
                    mic_db = mic_voltage_to_db(v_mic, offset=55.0)

                except Exception as e:
                    print("ADS read error:", e)
                    gas_percent = 0.0
                    mic_db = 0.0

            # --- NGH? 80ms tru?c khi d?c DHT11 ---
            time.sleep(0.08)

            # 2) �?c DHT11 sau d? tr�nh xung d?t
            try:
                temp, hum = read_dht11_stable(DHT_PIN)
            except:
                temp = None
                hum = None

            if temp is None or hum is None:
                temp = last_temp
                hum = last_hum
            else:
                last_temp = temp
                last_hum = hum

            # 3) Ghi LCD
            line1 = "T:{:4.1f}C H:{:3.0f}%".format(temp, hum)
            line2 = "N:{:4.1f}dB G:{:3.0f}%".format(mic_db, gas_percent)

            if lcd and getattr(lcd, "enabled", True):
                try:
                    lcd.clear()
                    lcd.write_line(line1, 1)
                    lcd.write_line(line2, 2)
                except:
                    print("LCD error � fallback")
                    print(line1)
                    print(line2)
            else:
                print(line1)
                print(line2)

            print(time.strftime("%Y-%m-%d %H:%M:%S"), "|", line1, "|", line2)
            
            
            ts = int(time.time() * 1000)

            save_point("data_temp.json", ts, float(temp))
            save_point("data_hum.json", ts, float(hum))
            save_point("data_noise.json", ts, float(mic_db))
            save_point("data_gas.json", ts, float(gas_percent))

            schedule_push_if_due()

            


            # 4) Ch? d?n chu k? d?c ti?p theo
            time.sleep(SLEEP_INTERVAL)

                    

    except KeyboardInterrupt:
        print("Stopping (KeyboardInterrupt)...")
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass



if __name__ == "__main__":
        main()
