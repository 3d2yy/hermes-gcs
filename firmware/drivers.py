"""
H.E.R.M.E.S. Robot - Hardware Drivers
Versión 2.1 - Corregida y Optimizada
Compatible con config.py unificado
"""

import time
import math
import struct
from machine import Pin, PWM, I2C
from config import (
    MOTOR_PINS, PWM_FREQUENCY, PCF8574_ADDRESSES, PCF_CONTROL_BITS,
    ADS1115_ADDR, MQ2_RL, MQ2_RO_CLEAN_AIR, MQ2_CHANNEL,
    ADS1115_REG_CONFIG, ADS1115_REG_CONVERSION, ADS1115_MUX_CONFIG, MQ2_GAIN,
    ULTRASONIC_ADDR, TRIG_BIT, ECHO_BIT,
    SCD30_ADDRESS, SAFETY_CONFIG
)


# ============================================================================
# MOTOR DRIVER - Control de 4 motores DC con PCF8574
# ============================================================================
class MotorDriver:
    """
    Controlador de motores usando PWM + PCF8574 para dirección.
    Soporta control diferencial para corrección PID.
    """
    
    def __init__(self, i2c):
        self.i2c = i2c
        self.motors = {}
        self.current_speeds = {1: 0, 2: 0, 3: 0, 4: 0}
        
        # Inicializar PWM en cada pin de motor
        for motor_id, pin_num in MOTOR_PINS.items():
            try:
                self.motors[motor_id] = PWM(Pin(pin_num), freq=PWM_FREQUENCY)
                self.motors[motor_id].duty(0)
            except Exception as e:
                print(f"[MotorDriver] Error init motor {motor_id}: {e}")
                
        print(f"[MotorDriver] Initialized {len(self.motors)} motors")
        
    def _write_pcf(self, addr, data):
        """Escribe un byte al PCF8574."""
        try:
            self.i2c.writeto(addr, bytes([data]))
            return True
        except Exception as e:
            print(f"[MotorDriver] PCF write error {hex(addr)}: {e}")
            return False

    def _get_direction_mask(self, motor_id, direction):
        """Obtiene la máscara de bits para dirección de un motor."""
        key = f"m{motor_id}_{direction}"
        return PCF_CONTROL_BITS.get(key, 0xFF)

    def stop(self):
        """Detiene todos los motores inmediatamente."""
        for motor in self.motors.values():
            motor.duty(0)
        
        # Estado seguro para PCFs (todos los bits altos = motores detenidos)
        self._write_pcf(PCF8574_ADDRESSES[1], 0xFF)
        self._write_pcf(PCF8574_ADDRESSES[2], 0xFF)
        self.current_speeds = {1: 0, 2: 0, 3: 0, 4: 0}

    def move(self, direction_name):
        """
        Ejecuta un movimiento predefinido.
        direction_name: "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"
        """
        MAX_PWM = 1023
        
        movements = {
            "FORWARD": {
                1: (MAX_PWM, "horario"), 2: (MAX_PWM, "horario"),
                3: (MAX_PWM, "horario"), 4: (MAX_PWM, "horario")
            },
            "BACKWARD": {
                1: (MAX_PWM, "antihorario"), 2: (MAX_PWM, "antihorario"),
                3: (MAX_PWM, "antihorario"), 4: (MAX_PWM, "antihorario")
            },
            "LEFT": {
                1: (MAX_PWM, "antihorario"), 2: (MAX_PWM, "antihorario"),
                3: (MAX_PWM, "horario"), 4: (MAX_PWM, "horario")
            },
            "RIGHT": {
                1: (MAX_PWM, "horario"), 2: (MAX_PWM, "horario"),
                3: (MAX_PWM, "antihorario"), 4: (MAX_PWM, "antihorario")
            },
            "STOP": {
                1: (0, "horario"), 2: (0, "horario"),
                3: (0, "horario"), 4: (0, "horario")
            }
        }
        
        cmd = movements.get(direction_name.upper(), movements["STOP"])
        
        # Calcular estados combinados para cada PCF
        # PCF1 controla motores 1 y 2, PCF2 controla motores 3 y 4
        pcf1_state = 0xFF
        pcf2_state = 0xFF
        
        for motor_id, (pwm_val, direction) in cmd.items():
            # Aplicar PWM
            self.motors[motor_id].duty(pwm_val)
            self.current_speeds[motor_id] = pwm_val
            
            # Calcular máscara de dirección
            mask = self._get_direction_mask(motor_id, direction)
            
            if motor_id <= 2:
                pcf1_state &= mask
            else:
                pcf2_state &= mask
        
        # Escribir estados a PCFs
        self._write_pcf(PCF8574_ADDRESSES[1], pcf1_state)
        self._write_pcf(PCF8574_ADDRESSES[2], pcf2_state)

    def set_differential(self, left_speed, right_speed, direction="horario"):
        """
        Control diferencial para corrección PID.
        left_speed, right_speed: 0-1023
        """
        left_speed = max(0, min(1023, int(left_speed)))
        right_speed = max(0, min(1023, int(right_speed)))
        
        # Motores 1,2 = izquierda, Motores 3,4 = derecha
        self.motors[1].duty(left_speed)
        self.motors[2].duty(left_speed)
        self.motors[3].duty(right_speed)
        self.motors[4].duty(right_speed)
        
        # Actualizar direcciones
        pcf1_state = self._get_direction_mask(1, direction) & self._get_direction_mask(2, direction)
        pcf2_state = self._get_direction_mask(3, direction) & self._get_direction_mask(4, direction)
        
        self._write_pcf(PCF8574_ADDRESSES[1], pcf1_state)
        self._write_pcf(PCF8574_ADDRESSES[2], pcf2_state)


# ============================================================================
# MQ-2 SENSOR DRIVER (via ADS1115 ADC)
# ============================================================================
class MQ2Driver:
    """
    Driver para sensor MQ-2 de gases usando ADS1115 como ADC.
    Detecta: LPG, Propano, Metano, Alcohol, Hidrógeno, Humo
    """
    
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = ADS1115_ADDR
        self.ro = MQ2_RO_CLEAN_AIR  # Valor por defecto, se calibra
        self.calibrated = False
        
    def _read_adc(self, channel=0):
        """Lee un canal del ADS1115."""
        if channel > 3:
            return 0
            
        try:
            # Configuración: Single-shot, canal específico, ganancia, 128SPS
            config = (0x8000 |  # OS: Start single conversion
                     ADS1115_MUX_CONFIG[channel] |  # MUX: Canal
                     MQ2_GAIN |  # PGA: Ganancia
                     0x0100 |  # MODE: Single-shot
                     0x0080 |  # DR: 128 SPS
                     0x0003)   # COMP_QUE: Disable comparator
            
            # Escribir configuración
            self.i2c.writeto(self.addr, bytes([ADS1115_REG_CONFIG, 
                                               (config >> 8) & 0xFF, 
                                               config & 0xFF]))
            time.sleep_ms(10)  # Esperar conversión
            
            # Leer resultado
            self.i2c.writeto(self.addr, bytes([ADS1115_REG_CONVERSION]))
            data = self.i2c.readfrom(self.addr, 2)
            
            raw = (data[0] << 8) | data[1]
            if raw > 32767:
                raw -= 65536
                
            return raw
        except Exception as e:
            print(f"[MQ2] ADC read error: {e}")
            return 0

    def read_voltage(self):
        """Convierte lectura ADC a voltaje."""
        raw = self._read_adc(MQ2_CHANNEL)
        # Con ganancia 1 (±4.096V), resolución = 4.096/32768 = 0.000125V
        voltage = raw * 0.000125
        return max(0, voltage)

    def calibrate(self, samples=20):
        """
        Calibra el sensor en aire limpio.
        Debe ejecutarse al inicio con el sensor en ambiente sin gases.
        """
        print("[MQ2] Calibrating in clean air...")
        readings = []
        
        for i in range(samples):
            v = self.read_voltage()
            if v > 0.1:  # Lectura válida
                readings.append(v)
            time.sleep_ms(100)
        
        if not readings:
            print("[MQ2] Calibration failed - no valid readings")
            return False
            
        avg_voltage = sum(readings) / len(readings)
        
        if avg_voltage < 0.1:
            print("[MQ2] Calibration failed - voltage too low")
            return False
        
        # RS = (Vc * RL) / Vout - RL
        # En realidad: RS = RL * (Vc - Vout) / Vout
        rs_air = MQ2_RL * (5.0 - avg_voltage) / avg_voltage
        self.ro = rs_air / MQ2_RO_CLEAN_AIR
        self.calibrated = True
        
        print(f"[MQ2] Calibrated. RO = {self.ro:.2f} kOhm")
        return True
        
    def read_ppm(self):
        """
        Lee concentración de gas en PPM.
        Retorna: (ppm, voltage, rs_ratio)
        """
        v = self.read_voltage()
        
        if v < 0.1:
            return 0.0, v, 0.0
        
        # Calcular RS
        rs = MQ2_RL * (5.0 - v) / v
        ratio = rs / self.ro
        
        if ratio <= 0:
            return 0.0, v, 0.0
        
        # Curva característica para humo/gas
        # log(PPM) = (log(RS/RO) - b) / m
        # Valores típicos para humo: m = -0.485, b = 1.51
        m = -0.485
        b = 1.51
        
        try:
            log_ppm = (math.log10(ratio) - b) / m
            ppm = 10 ** log_ppm
            return max(0, ppm), v, ratio
        except:
            return 0.0, v, ratio


# ============================================================================
# ULTRASONIC DRIVER (via PCF8574)
# ============================================================================
class UltrasonicDriver:
    """
    Driver para sensor ultrasónico HC-SR04 conectado via PCF8574.
    Permite medir distancia sin usar pines GPIO directos.
    """
    
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = ULTRASONIC_ADDR
        self.pcf_state = 0xFF
        
    def _write(self, val):
        """Escribe al PCF8574."""
        try:
            self.i2c.writeto(self.addr, bytes([val]))
            self.pcf_state = val
        except:
            pass

    def _read(self):
        """Lee del PCF8574."""
        try:
            return self.i2c.readfrom(self.addr, 1)[0]
        except:
            return 0xFF
        
    def get_distance_cm(self, timeout_us=30000):
        """
        Mide distancia en centímetros.
        Retorna -1 en caso de error o timeout.
        """
        try:
            # Asegurar trigger bajo
            self.pcf_state &= ~(1 << TRIG_BIT)
            self._write(self.pcf_state)
            time.sleep_us(5)
            
            # Pulso trigger (10us HIGH)
            self.pcf_state |= (1 << TRIG_BIT)
            self._write(self.pcf_state)
            time.sleep_us(10)
            
            self.pcf_state &= ~(1 << TRIG_BIT)
            self._write(self.pcf_state)
            
            # Esperar echo HIGH (inicio)
            t0 = time.ticks_us()
            while not (self._read() & (1 << ECHO_BIT)):
                if time.ticks_diff(time.ticks_us(), t0) > timeout_us:
                    return -1
                
            # Medir duración echo HIGH
            t1 = time.ticks_us()
            while (self._read() & (1 << ECHO_BIT)):
                if time.ticks_diff(time.ticks_us(), t1) > timeout_us:
                    return -1
                
            t2 = time.ticks_us()
            duration = time.ticks_diff(t2, t1)
            
            # Distancia = (tiempo * velocidad_sonido) / 2
            # Velocidad del sonido ≈ 343 m/s = 0.0343 cm/μs
            distance = (duration * 0.0343) / 2
            
            return distance if distance < 400 else -1  # Max ~4m
            
        except Exception as e:
            print(f"[Ultrasonic] Error: {e}")
            return -1


# ============================================================================
# SCD30 DRIVER - CO2, Temperatura, Humedad
# ============================================================================
class SCD30Driver:
    """
    Driver para sensor Sensirion SCD30.
    Mide: CO2 (400-10000 ppm), Temperatura (-40 a 70°C), Humedad (0-100%)
    """
    
    # Comandos SCD30
    CMD_START_MEASUREMENT = b'\x00\x10'
    CMD_STOP_MEASUREMENT = b'\x01\x04'
    CMD_DATA_READY = b'\x02\x02'
    CMD_READ_MEASUREMENT = b'\x03\x00'
    CMD_SET_INTERVAL = b'\x46\x00'
    
    def __init__(self, i2c, address=None):
        self.i2c = i2c
        self.addr = address or SCD30_ADDRESS
        self.co2 = 0.0
        self.temperature = 0.0
        self.humidity = 0.0
        
    def _crc8(self, data):
        """Calcula CRC-8 para verificación."""
        crc = 0xFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = crc << 1
                crc &= 0xFF
        return crc
        
    def begin(self, interval=2):
        """Inicia medición continua."""
        try:
            # Iniciar medición con presión ambiente (0 = deshabilitado)
            cmd = b'\x00\x10\x00\x00\x81'  # Presión 0, CRC
            self.i2c.writeto(self.addr, cmd)
            time.sleep_ms(50)
            return True
        except Exception as e:
            print(f"[SCD30] Init error: {e}")
            return False
    
    def data_ready(self):
        """Verifica si hay datos disponibles."""
        try:
            self.i2c.writeto(self.addr, self.CMD_DATA_READY)
            time.sleep_ms(3)
            data = self.i2c.readfrom(self.addr, 3)
            return (data[0] << 8 | data[1]) == 1
        except:
            return False
            
    def read(self):
        """
        Lee medición completa.
        Retorna True si exitoso, actualiza self.co2, self.temperature, self.humidity
        """
        try:
            self.i2c.writeto(self.addr, self.CMD_READ_MEASUREMENT)
            time.sleep_ms(5)
            data = self.i2c.readfrom(self.addr, 18)
            
            # Parsear datos (cada valor es 4 bytes + 2 CRC)
            # CO2: bytes 0-1, CRC 2, bytes 3-4, CRC 5
            co2_bytes = bytes([data[0], data[1], data[3], data[4]])
            self.co2 = struct.unpack('>f', co2_bytes)[0]
            
            # Temperatura: bytes 6-7, CRC 8, bytes 9-10, CRC 11
            temp_bytes = bytes([data[6], data[7], data[9], data[10]])
            self.temperature = struct.unpack('>f', temp_bytes)[0]
            
            # Humedad: bytes 12-13, CRC 14, bytes 15-16, CRC 17
            hum_bytes = bytes([data[12], data[13], data[15], data[16]])
            self.humidity = struct.unpack('>f', hum_bytes)[0]
            
            return True
        except Exception as e:
            print(f"[SCD30] Read error: {e}")
            return False
    
    def get_values(self):
        """Retorna última lectura como diccionario."""
        return {
            "co2": self.co2,
            "temperature": self.temperature,
            "humidity": self.humidity
        }
