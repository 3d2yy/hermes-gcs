"""
H.E.R.M.E.S. Robot Controller - ESP32 MicroPython
Versión 2.1 - Optimizada y Corregida

Sistema unificado: Robot + PID + MPU6050 + Ultrasónico + SCD30 + MQ-2
Arquitectura asíncrona para máximo rendimiento
"""

import network
import time
import json
import gc
import math
import uasyncio as asyncio
import ubinascii
from machine import I2C, Pin, PWM, reset, unique_id
from umqtt.simple import MQTTClient

# ========================
# IMPORTAR CONFIGURACIÓN
# ========================
try:
    from config import *
    print(f"[BOOT] Config loaded OK (v{PROTOCOL_VERSION})")
except ImportError as e:
    print(f"[ERROR] Cannot import config.py: {e}")
    SAFE_START = True

# ========================
# IMPORTAR DRIVERS
# ========================
try:
    from drivers import MotorDriver, MQ2Driver, UltrasonicDriver, SCD30Driver
    DRIVERS_AVAILABLE = True
    print("[BOOT] Drivers loaded OK")
except ImportError as e:
    print(f"[WARN] drivers.py not found, using inline classes: {e}")
    DRIVERS_AVAILABLE = False

# ========================
# IMPORTAR MPU6050
# ========================
try:
    from MPU6050 import MPU6050
    MPU6050_AVAILABLE = True
    print("[BOOT] MPU6050 available")
except ImportError:
    MPU6050_AVAILABLE = False
    print("[WARN] MPU6050 not available")

# ========================
# IMPORTAR PID
# ========================
try:
    from pid import PID
    PID_AVAILABLE = True
    print("[BOOT] PID available")
except ImportError:
    PID_AVAILABLE = False
    print("[WARN] PID not available, using dummy")
    
    class PID:
        """Dummy PID controller cuando no está disponible la librería."""
        def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0):
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.setpoint = setpoint
            
        def compute(self, input_val):
            return 0
            
        def reset(self):
            pass


# ========================
# LOGGER ESTRUCTURADO
# ========================
class Logger:
    """Sistema de logging ligero para ESP32."""
    LEVELS = {"INFO": "🔵", "WARN": "🟠", "ERROR": "🔴", "SUCCESS": "🟢", "DEBUG": "⚪"}

    @staticmethod
    def log(message, level="INFO"):
        if not DEBUG_ENABLED and level == "DEBUG":
            return
        icon = Logger.LEVELS.get(level, "⚪")
        uptime = time.ticks_ms() / 1000.0
        print(f"[{uptime:8.2f}] {icon} {level:7} | {message}")

log = Logger.log


# ========================
# SISTEMA PRINCIPAL
# ========================
class HermesRobot:
    """
    Sistema principal del robot H.E.R.M.E.S.
    Maneja todos los subsistemas de forma asíncrona.
    """
    
    def __init__(self):
        print("=" * 50)
        print(" H.E.R.M.E.S. Robot Controller v2.1")
        print(" Hostile Environment Reconnaissance")
        print("=" * 50)
        
        # ===== INICIALIZAR I2C =====
        self.i2c = I2C(0, 
                       scl=Pin(I2C_SCL_PIN), 
                       sda=Pin(I2C_SDA_PIN), 
                       freq=I2C_FREQUENCY)
        
        devices = self.i2c.scan()
        print(f"[I2C] Found devices: {[hex(d) for d in devices]}")
        
        # ===== INICIALIZAR DRIVERS =====
        self.motors = None
        self.mq2 = None
        self.scd30 = None
        self.ultrasonic = None
        self.mpu = None
        self.pid = None
        
        # Motor Driver
        try:
            if DRIVERS_AVAILABLE:
                self.motors = MotorDriver(self.i2c)
            else:
                self.motors = self._create_inline_motor_driver()
            print("[HW] Motors OK")
        except Exception as e:
            print(f"[ERROR] Motors init failed: {e}")
            
        # MQ-2 Gas Sensor
        try:
            if ADS1115_ADDR in devices:
                if DRIVERS_AVAILABLE:
                    self.mq2 = MQ2Driver(self.i2c)
                    self.mq2.calibrate(10)
                print("[HW] MQ-2 OK")
            else:
                print("[WARN] ADS1115/MQ-2 not found")
        except Exception as e:
            print(f"[ERROR] MQ-2 init failed: {e}")
            
        # SCD30 Environment Sensor
        try:
            if SCD30_ADDRESS in devices:
                if DRIVERS_AVAILABLE:
                    self.scd30 = SCD30Driver(self.i2c)
                    self.scd30.begin()
                print("[HW] SCD30 OK")
            else:
                print("[WARN] SCD30 not found")
        except Exception as e:
            print(f"[ERROR] SCD30 init failed: {e}")
            
        # Ultrasonic Sensor
        try:
            if ULTRASONIC_ADDR in devices:
                if DRIVERS_AVAILABLE:
                    self.ultrasonic = UltrasonicDriver(self.i2c)
                print("[HW] Ultrasonic OK")
            else:
                print("[WARN] Ultrasonic PCF not found")
        except Exception as e:
            print(f"[ERROR] Ultrasonic init failed: {e}")
            
        # MPU6050 IMU
        try:
            if MPU6050_AVAILABLE and MPU6050_ADDR in devices:
                self.mpu = MPU6050(bus=self.i2c, addr=MPU6050_ADDR)
                print("[HW] MPU6050 OK")
            else:
                print("[WARN] MPU6050 not found")
        except Exception as e:
            print(f"[ERROR] MPU6050 init failed: {e}")
            
        # PID Controller
        if PID_AVAILABLE and self.mpu:
            self.pid = PID(kp=2.0, ki=0.5, kd=0.1, setpoint=0)
            print("[HW] PID Controller OK")
            
        # ===== ESTADO DEL SISTEMA =====
        self.wifi = None
        self.mqtt = None
        self.connected = False
        
        self.active_command = "stop"
        self.last_cmd_time = time.time()
        self.emergency_stop = False
        
        # IMU state
        self.gyro_bias = 0.0
        self.yaw = 0.0
        self.target_yaw = 0.0
        
        # Sensor readings
        self.last_distance = -1
        self.last_ppm = 0
        self.last_co2 = 400
        self.last_temp = 25
        self.last_hum = 50
        
        log(f"Initialization complete (Protocol v{PROTOCOL_VERSION})", "SUCCESS")
        print("=" * 50)

    def _create_inline_motor_driver(self):
        """Crea motor driver inline si drivers.py no está disponible."""
        class InlineMotorDriver:
            def __init__(self, i2c):
                self.i2c = i2c
                self.motors = {}
                for motor_id, pin_num in MOTOR_PINS.items():
                    self.motors[motor_id] = PWM(Pin(pin_num), freq=PWM_FREQUENCY)
                    self.motors[motor_id].duty(0)
                    
            def stop(self):
                for m in self.motors.values():
                    m.duty(0)
                try:
                    self.i2c.writeto(PCF8574_ADDRESSES[1], bytes([0xFF]))
                    self.i2c.writeto(PCF8574_ADDRESSES[2], bytes([0xFF]))
                except:
                    pass
                    
            def move(self, direction):
                # Implementación simplificada
                pass
                
            def set_differential(self, left, right, direction="horario"):
                left = max(0, min(1023, int(left)))
                right = max(0, min(1023, int(right)))
                self.motors[1].duty(left)
                self.motors[2].duty(left)
                self.motors[3].duty(right)
                self.motors[4].duty(right)
                
        return InlineMotorDriver(self.i2c)

    # ========================
    # TAREAS ASÍNCRONAS
    # ========================
    
    async def task_wifi_mqtt(self):
        """Mantiene conexión WiFi y MQTT."""
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        try:
            wlan.config(dhcp_hostname="hermes-robot")
        except:
            pass # Algunos firmwares no soportan esto
        
        client_id = ubinascii.hexlify(unique_id()).decode()
        
        while True:
            try:
                # ===== CONEXIÓN WIFI =====
                if not wlan.isconnected():
                    log("Searching for known WiFi networks...", "INFO")
                    networks = wlan.scan()
                    found_ssids = [n[0].decode() for n in networks]
                    
                    for net in WIFI_NETWORKS:
                        ssid = net["ssid"]
                        if ssid in found_ssids:
                            log(f"Connecting to {ssid}...", "INFO")
                            wlan.connect(ssid, net["pass"])
                            
                            timeout = WIFI_TIMEOUT
                            while not wlan.isconnected() and timeout > 0:
                                await asyncio.sleep(1)
                                timeout -= 1
                                
                            if wlan.isconnected():
                                log(f"Connected to {ssid}! IP: {wlan.ifconfig()[0]}", "SUCCESS")
                                break
                    
                    if not wlan.isconnected():
                        log("No known networks found, retrying in 5s...", "WARN")
                        await asyncio.sleep(5)
                        continue
                
                # ===== CONEXIÓN MQTT =====
                if not self.connected:
                    try:
                        print(f"[MQTT] Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
                        
                        self.mqtt = MQTTClient(
                            client_id, 
                            MQTT_BROKER, 
                            port=MQTT_PORT,
                            user=MQTT_USER,
                            password=MQTT_PASSWORD,
                            keepalive=MQTT_KEEPALIVE
                        )
                        
                        self.mqtt.set_callback(self._mqtt_callback)
                        self.mqtt.connect()
                        self.mqtt.subscribe(TOPIC_COMMAND)
                        
                        self.connected = True
                        print("[MQTT] Connected!")
                        
                        # Publicar estado online
                        self._publish(TOPIC_STATUS, {
                            "status": "online",
                            "device": DEVICE_NAME,
                            "location": LOCATION
                        })
                        
                    except Exception as e:
                        print(f"[MQTT] Connection failed: {e}")
                        self.connected = False
                        await asyncio.sleep(RECONNECT_DELAY)
                        continue
                
                # ===== CHECK MENSAJES =====
                try:
                    self.mqtt.check_msg()
                except Exception as e:
                    print(f"[MQTT] Check error: {e}")
                    self.connected = False
                    
            except Exception as e:
                print(f"[NET] Error: {e}")
                self.connected = False
                
            await asyncio.sleep_ms(10) # 10ms = 100Hz polling rate

    def _mqtt_callback(self, topic, msg):
        """Callback para mensajes MQTT recibidos."""
        try:
            topic_str = topic.decode() if isinstance(topic, bytes) else topic
            payload = json.loads(msg.decode())
            
            cmd = payload.get("command", "").upper()
            
            # Mapear comandos alternativos
            cmd_map = {
                "PARAR": "STOP",
                "DETENER": "STOP",
                "FORWARD": "ADELANTE",
                "BACKWARD": "ATRAS",
                "BACK": "ATRAS",
                "LEFT": "IZQUIERDA",
                "RIGHT": "DERECHA",
                "ADELANTE": "FORWARD",
                "ATRAS": "BACKWARD",
                "IZQUIERDA": "LEFT",
                "DERECHA": "RIGHT"
            }
            cmd = cmd_map.get(cmd, cmd)
            
            # Actualizar timestamp
            self.last_cmd_time = time.time()
            
            # Si cambiamos a FORWARD, guardar heading actual para PID
            if cmd == "FORWARD" and self.active_command != "FORWARD":
                self.target_yaw = self.yaw
                if self.pid:
                    self.pid.reset()
                    
            # Manejar LED
            if cmd == "LED":
                val = payload.get("val", payload.get("value", 128))
                # Aquí iría el control de LED si hubiera hardware
                log(f"LED intensity set to {val}", "INFO")
                return
                
            # Manejar Calibración Remota
            if cmd == "CALIBRATE":
                log("Remote calibration requested...", "WARN")
                self.calibrate_imu()
                log("Remote calibration complete!", "SUCCESS")
                return
                
            self.active_command = cmd
            print(f"[CMD] Received: {cmd}")
            
        except Exception as e:
            print(f"[MQTT] Callback error: {e}")

    def _publish(self, topic, data):
        """Helper para publicar mensajes MQTT con versionado."""
        if self.mqtt and self.connected:
            try:
                # Inyectar versión si es un diccionario
                if isinstance(data, dict):
                    data["v"] = PROTOCOL_VERSION
                
                self.mqtt.publish(topic, json.dumps(data))
                return True
            except Exception as e:
                log(f"Publish error: {e}", "ERROR")
                return False
        return False

    async def task_navigation(self):
        """Loop de control de navegación con PID (50Hz)."""
        last_ticks = time.ticks_ms()
        
        while True:
            try:
                now = time.ticks_ms()
                dt = time.ticks_diff(now, last_ticks) / 1000.0
                last_ticks = now
                
                # ===== INTEGRAR GIROSCOPIO =====
                if self.mpu and dt > 0 and dt < 1.0:
                    gyro = self.mpu.read_gyro_data()
                    gz = gyro['z'] - self.gyro_bias
                    
                    # Deadband para ruido
                    if abs(gz) > 0.5:
                        self.yaw += gz * dt
                
                # ===== LÓGICA DE CONTROL =====
                if self.emergency_stop:
                    if self.motors:
                        self.motors.stop()
                        
                elif self.active_command == "FORWARD" and self.mpu and self.pid:
                    # Control PID para mantener línea recta
                    self.pid.setpoint = self.target_yaw
                    correction = self.pid.compute(self.yaw)
                    
                    # Aplicar corrección diferencial
                    base_speed = 1000  # PWM aumentado para mayor velocidad (max 1023)
                    max_correction = 200
                    correction = max(-max_correction, min(max_correction, correction))
                    
                    left_speed = base_speed + correction
                    right_speed = base_speed - correction
                    
                    if self.motors:
                        self.motors.set_differential(left_speed, right_speed, "horario")
                        
                elif self.active_command in ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]:
                    # Control open-loop
                    if self.motors:
                        self.motors.move(self.active_command)
                        
                elif self.active_command == "STOP":
                    if self.motors:
                        self.motors.stop()
                else:
                    if self.motors:
                        self.motors.stop()
                        
            except Exception as e:
                print(f"[NAV] Error: {e}")
                if self.motors:
                    self.motors.stop()
                    
            await asyncio.sleep_ms(20)  # 50Hz

    async def task_sensors_fast(self):
        """Sensores rápidos: MQ-2, IMU (10Hz)."""
        while True:
            try:
                if self.connected:
                    # ===== MQ-2 GAS SENSOR =====
                    if self.mq2:
                        ppm, voltage, ratio = self.mq2.read_ppm()
                        self.last_ppm = ppm
                        
                        # Determinar nivel de alerta
                        if ppm >= SMOKE_THRESHOLDS["critico"]:
                            alert = "critico"
                            if SAFETY_CONFIG.get("gas_emergency_stop", True):
                                self.emergency_stop = True
                                self.active_command = "stop"
                        elif ppm >= SMOKE_THRESHOLDS["peligro"]:
                            alert = "peligro"
                        elif ppm >= SMOKE_THRESHOLDS["advertencia"]:
                            alert = "advertencia"
                        else:
                            alert = "normal"
                            if self.emergency_stop:
                                # Resetear emergency stop si niveles son seguros
                                self.emergency_stop = False
                        
                        self._publish(TOPIC_MQ2_DATA, {
                            "sensor_data": {
                                "ppm": round(ppm, 1),
                                "voltage": round(voltage, 3),
                                "rs_ro_ratio": round(ratio, 3),
                                "alert_status": alert
                            }
                        })
                        
                        # Publicar alerta si es necesario
                        if alert in ["peligro", "critico"]:
                            self._publish(TOPIC_MQ2_ALERT, {
                                "message": f"GAS ALERT: {alert.upper()} - {ppm:.0f} PPM"
                            })
                    
                    # ===== IMU DATA =====
                    if self.mpu:
                        accel = self.mpu.read_accel_data()
                        gyro = self.mpu.read_gyro_data()
                        angle = self.mpu.read_angle()
                        
                        self._publish(TOPIC_MPU_DATA, {
                            "accelerometer": {
                                "x": round(accel['x'], 3),
                                "y": round(accel['y'], 3),
                                "z": round(accel['z'], 3)
                            },
                            "gyroscope": {
                                "x": round(gyro['x'], 3),
                                "y": round(gyro['y'], 3),
                                "z": round(gyro['z'], 3)
                            },
                            "orientation": {
                                "roll": round(math.degrees(angle['x']), 2),
                                "pitch": round(math.degrees(angle['y']), 2),
                                "yaw": round(self.yaw, 2)
                            }
                        })
                        
            except Exception as e:
                print(f"[SENSOR_FAST] Error: {e}")
                
            await asyncio.sleep_ms(100)  # 10Hz

    async def task_sensors_slow(self):
        """Sensores lentos: SCD30, Ultrasónico (2Hz)."""
        while True:
            try:
                if self.connected:
                    # ===== SCD30 ENVIRONMENT =====
                    if self.scd30:
                        if self.scd30.data_ready():
                            if self.scd30.read():
                                self.last_co2 = self.scd30.co2
                                self.last_temp = self.scd30.temperature
                                self.last_hum = self.scd30.humidity
                                
                                # Aplicar calibración si está configurada
                                temp_offset = SENSOR_CALIBRATION.get("temperature_offset", 0)
                                hum_slope = SENSOR_CALIBRATION.get("humidity_slope", 1.0)
                                hum_intercept = SENSOR_CALIBRATION.get("humidity_intercept", 0)
                                
                                calibrated_temp = self.last_temp + temp_offset
                                calibrated_hum = self.last_hum * hum_slope + hum_intercept
                                calibrated_hum = max(0, min(100, calibrated_hum))
                                
                                self._publish(TOPIC_SCD30_DATA, {
                                    "co2": round(self.last_co2, 1),
                                    "temperature": round(calibrated_temp, 2),
                                    "humidity": round(calibrated_hum, 2)
                                })
                    
                    # ===== ULTRASONIC =====
                    if self.ultrasonic:
                        # Usar método síncrono por simplicidad
                        dist = self.ultrasonic.get_distance_cm()
                        
                        if dist > 0:
                            self.last_distance = dist
                            
                            # Detección de obstáculos
                            obstacle_dist = SAFETY_CONFIG.get("obstacle_distance", 10)
                            if dist < obstacle_dist and self.active_command == "FORWARD":
                                print(f"[OBSTACLE] Stopping - {dist:.1f}cm")
                                self.active_command = "STOP"
                                
                            self._publish(TOPIC_ULTRASONIC, {
                                "distance_cm": round(dist, 2)
                            })
                            
            except Exception as e:
                print(f"[SENSOR_SLOW] Error: {e}")
                
            await asyncio.sleep_ms(500)  # 2Hz

    async def task_heartbeat(self):
        """Envía heartbeat periódico y verifica watchdog."""
        while True:
            try:
                if self.connected:
                    self._publish(TOPIC_HEARTBEAT, {
                        "timestamp": time.time(),
                        "uptime": time.ticks_ms() // 1000,
                        "free_mem": gc.mem_free(),
                        "active_cmd": self.active_command
                    })
                    
                # ===== WATCHDOG =====
                # Si no hay comando en X segundos, detener
                if self.active_command != "STOP":
                    timeout = SAFETY_CONFIG.get("auto_stop_timeout", 30)
                    if time.time() - self.last_cmd_time > timeout:
                        print("[WATCHDOG] Safety stop - command timeout")
                        self.active_command = "STOP"
                        if self.motors:
                            self.motors.stop()
                            
                # Garbage collection
                gc.collect()
                
            except Exception as e:
                print(f"[HEARTBEAT] Error: {e}")
                
            await asyncio.sleep(HEARTBEAT_INTERVAL)

    def calibrate_imu(self):
        """Calibra el bias del giroscopio."""
        if not self.mpu:
            print("[CALIB] No MPU6050 available")
            return
            
        print("[CALIB] Calibrating gyroscope (keep robot still)...")
        
        bias_sum = 0
        samples = 50
        
        for i in range(samples):
            gyro = self.mpu.read_gyro_data()
            bias_sum += gyro['z']
            time.sleep_ms(20)
            
        self.gyro_bias = bias_sum / samples
        print(f"[CALIB] Done. Z-axis bias: {self.gyro_bias:.4f} deg/s")

    async def start(self):
        """Inicia todas las tareas del sistema."""
        # Calibrar IMU primero
        self.calibrate_imu()
        
        print("[BOOT] Starting async tasks...")
        
        # Crear y ejecutar tareas
        asyncio.create_task(self.task_wifi_mqtt())
        asyncio.create_task(self.task_navigation())
        asyncio.create_task(self.task_sensors_fast())
        asyncio.create_task(self.task_sensors_slow())
        asyncio.create_task(self.task_heartbeat())
        
        print("[BOOT] System running!")
        print("=" * 50)
        
        # Mantener el loop principal
        while True:
            await asyncio.sleep(1)


# ========================
# PUNTO DE ENTRADA
# ========================
if __name__ == "__main__":
    # Verificar modo seguro
    if SAFE_START:
        print("[SAFE MODE] System halted. Fix config.py and restart.")
    else:
        robot = HermesRobot()
        try:
            asyncio.run(robot.start())
        except KeyboardInterrupt:
            print("\n[SHUTDOWN] Interrupted by user")
            if robot.motors:
                robot.motors.stop()
        except Exception as e:
            print(f"[FATAL] {e}")
            reset()
