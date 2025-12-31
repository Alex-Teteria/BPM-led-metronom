# Author: Oleksandr Teteria
# v0.2
# 31.12.2025
# Implemented and tested on Pi Pico with RP2040
# Released under the MIT license

import _thread
import utime
from machine import Pin, PWM
import adc_dma  # RP2040-ADC-DMA-Extension
import math


# ======================================
# Конфігурація
# ======================================
SAMPLE_FREQ = 6800        # Hz
FRAME_SIZE = 148          # samples per frame
ADC0 = 0  # (GPIO26)

# LED with PWM (решта конфігурації - в core1)
LED_PIN = 15
LED_DUTY = 40_000    # задаємо максимальну яскравість LED (0–65535)

# Beat detector, Adaptive K 
K_BASE = 1.3          # мінімальний поріг (чутливість)
K_ALPHA = 0.05        # швидкість адаптації (0.02–0.1)
K_lp = 1.5            # стартове значення
MIN_BEAT_INTERVAL_MS = 250
EPS = 1e-6    # захист від ділення на нуль в нормалізації енергії

# LP for Energy
ENERGY_LP_ALPHA = 0.15        # 0.1–0.2 оптимально

# BPM (динамічний профіль)
ALPHA_SLOW = 0.10
ALPHA_FAST = 0.30

DT_MIN_SLOW = 500
DT_MAX_SLOW = 2000

DT_MIN_FAST = 200
DT_MAX_FAST = 1200


# ======================================
# Буфери та синхронізація
# ======================================
write_buf = [0] * FRAME_SIZE
read_buf = [0] * FRAME_SIZE

buffer_ready = False
lock = _thread.allocate_lock()

# ======================================
# CORE 0 — семплування
# ======================================
def core0_sampling():
    global write_buf, read_buf, buffer_ready
    
    while True:
        adc_dma.start(ADC0, SAMPLE_FREQ, FRAME_SIZE)
            
        while adc_dma.busy():
            utime.sleep_ms(1)
            
        # Коли готово — отримуємо буфер
        buf = adc_dma.buffer()
            
        for i, raw in enumerate(buf):
            write_buf[i] = (raw - 2048) << 3
            
        adc_dma.close()
            
        with lock:
            write_buf, read_buf = read_buf, write_buf
            buffer_ready = True
                

# -------------------------------
# Beat detector
# -------------------------------
last_beat_time = 0
E_prev = 0
E_prev2 = 0

def beat_detect(E, now_ms, K_eff):
    global last_beat_time, E_prev, E_prev2

    beat = 0

    if E > K_eff:
        # другий диференціал:
        # E_prev2 < E_prev < E  → підйом з прискоренням
        if E_prev2 < E_prev < E:
            if utime.ticks_diff(now_ms, last_beat_time) > MIN_BEAT_INTERVAL_MS:
                beat = 1
                last_beat_time = now_ms

    E_prev2 = E_prev
    E_prev = E

    return beat


def update_bpm_profile(bpm_lp):
    # fallback на старті або при втраті BPM
    if bpm_lp <= 0:
        return ALPHA_SLOW, DT_MIN_SLOW, DT_MAX_SLOW

    # нормалізація bpm (60–180)
    x = (bpm_lp - 60) / 120
    x = max(0.0, min(1.0, x))

    BPM_ALPHA = ALPHA_SLOW + x * (ALPHA_FAST - ALPHA_SLOW)
    DT_MIN    = DT_MIN_SLOW + x * (DT_MIN_FAST - DT_MIN_SLOW)
    DT_MAX    = DT_MAX_SLOW + x * (DT_MAX_FAST - DT_MAX_SLOW)

    return BPM_ALPHA, DT_MIN, DT_MAX


# -------------------------------
# Стан фільтрa
# -------------------------------
energy_lp = 0
# -------------------------------
# for BPM calculator
# -------------------------------
last_beat_time_bpm = 0
bpm_lp = 0.0
last_printed_bpm = 0
# -------------------------------
# core1 main loop
# -------------------------------
def core1_processing():
    global buffer_ready, energy_lp, last_beat_time_bpm, bpm_lp, K_lp
    global last_printed_bpm
    
    # LED init (core1)
    led_pwm = PWM(Pin(LED_PIN))
    led_pwm.freq(1000)

    led_phase_start = utime.ticks_ms()
    
    print("core1 started")
    
    while True:
        # =======================
        # 1. Чекаємо буфер і забираємо буфер
        # =======================
        with lock:
            if not buffer_ready:
                buf = None
            else:
                buf = read_buf
                buffer_ready = False
        
        if buf is None:
            utime.sleep_us(20)
            continue
        
        # ===============================================================
        # 3. DSP пайплайн
        #   (HP → Energy → LP(Energy) → АРУ → Adaptive K → Beat detector)
        # ===============================================================
        E = 0
        for v in buf:
            E += v * v
            #E += abs(v)

        energy_lp += ENERGY_LP_ALPHA * (E - energy_lp)
        
        # АРУ (нормалізація)
        E_norm = min(5.0, E / (energy_lp + EPS))

        # Adaptive K (по E_norm)
        K_lp += K_ALPHA * (E_norm - K_lp)
        K_eff = max(K_BASE, K_lp)
        
        # Beat detector
        now_ms = utime.ticks_ms()
        beat = beat_detect(E_norm, now_ms, K_eff)
        
        # ==================================
        # 4. Temporal logic:
        #    (Beat → BPM) beat використовується тільки для BPM
        # ==================================
        # динамічний профіль змінює параметри 
        BPM_ALPHA, DT_MIN, DT_MAX = update_bpm_profile(bpm_lp)
                        
        if beat:
            if last_beat_time_bpm != 0:
                dt = utime.ticks_diff(now_ms, last_beat_time_bpm)
                if DT_MIN <= dt <= DT_MAX:
                    bpm_raw = 60000 / dt
                    bpm_lp += BPM_ALPHA * (bpm_raw - bpm_lp) if bpm_lp else bpm_raw
            last_beat_time_bpm = now_ms
        
        
        # =======================
        # LED BPM МЕТРОНОМ
        # =======================
        if bpm_lp > 0:
            
            # для друку BPM, коли воно змінюється 
            bpm_int = int(bpm_lp + 0.5)
            if bpm_int != last_printed_bpm:
                print("LED BPM:", bpm_int)
                last_printed_bpm = bpm_int
            
            
            T = 60000 / bpm_lp
            t = utime.ticks_diff(utime.ticks_ms(), led_phase_start)

            # автономна фаза
            if t >= T:
                led_phase_start = utime.ticks_add(led_phase_start, int(T))
                t = utime.ticks_diff(utime.ticks_ms(), led_phase_start)

            phase = 2 * math.pi * t / T
            brightness = 0.5 * (1 - math.cos(phase))
            duty = int(brightness * LED_DUTY)
        else:
            duty = 0
        
        led_pwm.duty_u16(duty)
        
        utime.sleep_us(100)

# ======================================
# Старт
# ======================================
_thread.start_new_thread(core1_processing, ())
core0_sampling()
