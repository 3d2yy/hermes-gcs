"""
H.E.R.M.E.S. GCS - MQTT Service
Versión 2.1 - Corregida
Maneja toda la comunicación MQTT con el robot
"""

import json
import threading
import numpy as np
import paho.mqtt.client as mqtt

from src.config import MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASS, PROTOCOL_VERSION
from src.state import state
from src.constants import MQTTTopics  # FIX: Import faltante
from src.services.simulation import start_simulation

# Cliente MQTT global
mqtt_client = None
_reconnect_timer = None


def publish_command(topic, payload):
    """
    Publica un comando al robot via MQTT.
    
    Args:
        topic: Tópico MQTT destino
        payload: Dict o string a enviar
        
    Returns:
        bool: True si se envió exitosamente
    """
    global mqtt_client
    
    if mqtt_client is None:
        state.log("MQTT client not initialized", "ERROR")
        return False
        
    if state.status["connection"] != "ONLINE":
        state.log("Cannot publish - not connected", "WARN")
        return False
        
    try:
        if isinstance(payload, dict):
            payload["v"] = PROTOCOL_VERSION # Inyectar versión
            payload = json.dumps(payload)
        
        result = mqtt_client.publish(topic, payload, qos=0)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            return True
        else:
            state.log(f"MQTT publish failed: {result.rc}", "ERROR")
            return False
            
    except Exception as e:
        state.log(f"Error publishing MQTT: {e}", "ERROR")
        return False


def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    """Callback cuando se establece conexión MQTT."""
    if rc == 0:
        state.log("Conexión MQTT establecida", "SUCCESS")
        state.status["connection"] = "ONLINE"
        state.status["mode"] = "MQTT"
        
        # Suscribirse a todos los tópicos relevantes
        topics = [
            (MQTTTopics.MQ2_DATA, 0),
            (MQTTTopics.MQ2_ALERT, 0),
            (MQTTTopics.ENVIRONMENT, 0),
            (MQTTTopics.IMU, 0),
            (MQTTTopics.ULTRASONIC, 0),
            (MQTTTopics.AUDIO, 0),
            (MQTTTopics.POSITION, 0),
            (MQTTTopics.RADAR, 0),
            (MQTTTopics.STATUS, 0),
            (MQTTTopics.POWER, 0),
            (MQTTTopics.HEARTBEAT, 0),
        ]
        
        for topic, qos in topics:
            client.subscribe(topic, qos)
            
        state.log(f"Subscribed to {len(topics)} topics", "INFO")
        
    else:
        error_msgs = {
            1: "Protocol version incorrect",
            2: "Client identifier rejected",
            3: "Server unavailable",
            4: "Bad username/password",
            5: "Not authorized"
        }
        error = error_msgs.get(rc, f"Unknown error {rc}")
        state.log(f"MQTT connection failed: {error}", "ERROR")
        state.status["connection"] = "DISCONNECTED"


def on_mqtt_disconnect(client, userdata, rc, properties=None):
    """Callback cuando se pierde conexión MQTT."""
    global _reconnect_timer
    
    state.status["connection"] = "DISCONNECTED"
    state.status["mode"] = "ESPERANDO"
    
    if rc == 0:
        state.log("MQTT disconnected gracefully", "INFO")
    else:
        state.log(f"MQTT disconnected unexpectedly (rc={rc}). Reconnecting...", "WARN")
        
        # Programar reconexión
        def reconnect():
            global mqtt_client
            if mqtt_client and state.status["connection"] != "ONLINE":
                try:
                    mqtt_client.reconnect()
                except:
                    pass
                    
        _reconnect_timer = threading.Timer(5.0, reconnect)
        _reconnect_timer.start()


def on_mqtt_message(client, userdata, msg):
    """Callback para mensajes MQTT recibidos."""
    try:
        payload = json.loads(msg.payload.decode('utf-8'))
        topic = msg.topic
        
        # Verificar versión del protocolo (opcional, solo loguear si hay mismatch)
        msg_version = payload.get("v")
        if msg_version and msg_version != PROTOCOL_VERSION:
            state.log(f"Protocol mismatch: msg v{msg_version} != local v{PROTOCOL_VERSION}", "DEBUG")
        
        # ===== SENSOR MQ-2 (Gas/Humo) =====
        if topic == MQTTTopics.MQ2_DATA or "mq2/data" in topic:
            sensor_data = payload.get("sensor_data", payload)
            ppm = sensor_data.get("ppm", 0)
            voltage = sensor_data.get("voltage", 0)
            
            state.update_sensor_data(ppm=ppm, volt=voltage)
            state.add_gas_reading(
                state.robot_position["x"], 
                state.robot_position["y"], 
                ppm
            )
            
            # Actualizar nivel de alerta
            alert_status = sensor_data.get("alert_status", "normal").lower()
            if alert_status == "critico":
                state.status["alert_level"] = "CRITICAL"
            elif alert_status in ["peligro", "advertencia"]:
                state.status["alert_level"] = "WARNING"
            else:
                state.status["alert_level"] = "NORMAL"

        # ===== ALERTA MQ-2 =====
        elif topic == MQTTTopics.MQ2_ALERT or "mq2/alert" in topic:
            message = payload.get("message", payload.get("msg", "Unknown alert"))
            state.log(f"⚠️ ALERTA GAS: {message}", "WARN")
            state.status["alert_level"] = "CRITICAL"
                
        # ===== AMBIENTE (SCD30) =====
        elif topic == MQTTTopics.ENVIRONMENT or "sensor/data" in topic:
            state.update_sensor_data(
                co2=payload.get("co2"), 
                temp=payload.get("temperature"), 
                hum=payload.get("humidity")
            )
            
        # ===== POTENCIA =====
        elif topic == MQTTTopics.POWER or "sensors/power" in topic:
            state.update_sensor_data(
                volt=payload.get("voltage"), 
                curr=payload.get("current")
            )
            state.current_values["rssi"] = payload.get("rssi", -50)
            state.current_values["battery_percent"] = payload.get("battery", 
                state.current_values["battery_percent"])
            
        # ===== IMU (MPU6050) =====
        elif topic == MQTTTopics.IMU or "mpu6050" in topic:
            # Actualizar datos del IMU
            if "accelerometer" in payload:
                accel = payload["accelerometer"]
                state.imu["accel_x"] = accel.get("x", 0)
                state.imu["accel_y"] = accel.get("y", 0)
                state.imu["accel_z"] = accel.get("z", 0)
                
            if "gyroscope" in payload:
                gyro = payload["gyroscope"]
                state.imu["gyro_x"] = gyro.get("x", 0)
                state.imu["gyro_y"] = gyro.get("y", 0)
                state.imu["gyro_z"] = gyro.get("z", 0)
                
            if "orientation" in payload:
                orient = payload["orientation"]
                state.imu["roll"] = orient.get("roll", 0)
                state.imu["pitch"] = orient.get("pitch", 0)
                state.imu["yaw"] = orient.get("yaw", 0)
                
        # ===== ULTRASÓNICO =====
        elif topic == MQTTTopics.ULTRASONIC or "ultrasonic" in topic:
            dist = payload.get("distance_cm", payload.get("distance"))
            if dist is not None:
                state.current_values["ultrasonic"] = dist
                
                # Log si hay obstáculo cercano
                if dist < 15:
                    state.log(f"⚠️ Obstáculo cercano: {dist:.1f} cm", "WARN")
                
        # ===== POSICIÓN =====
        elif topic == MQTTTopics.POSITION or "position" in topic:
            x = payload.get("x", state.robot_position["x"])
            y = payload.get("y", state.robot_position["y"])
            theta = payload.get("theta", state.robot_position["theta"])
            state.update_robot_position(x, y, theta)
            
        # ===== RADAR/LIDAR =====
        elif topic == MQTTTopics.RADAR or "radar" in topic:
            if "distances" in payload:
                state.radar_distances = np.array(payload["distances"])
            if "angles" in payload:
                state.radar_angles = np.array(payload["angles"])
                
        # ===== AUDIO AI =====
        elif topic == MQTTTopics.AUDIO or "ai/audio" in topic:
            classification = payload.get("class", payload.get("classification", "SILENCE"))
            confidence = payload.get("confidence", 0)
            direction = payload.get("direction")
            
            state.status["audio_class"] = classification
            state.status["audio_confidence"] = confidence
            
            # Registrar detección significativa
            if confidence > 50 and classification not in ["SILENCE", "AMBIENT"]:
                state.add_acoustic_detection(classification, confidence, direction)
                
        # ===== STATUS =====
        elif topic == MQTTTopics.STATUS or "device/status" in topic:
            status = payload.get("status", "unknown")
            if status == "online":
                state.log("Robot online", "SUCCESS")
            elif status == "error":
                state.log(f"Robot error: {payload.get('msg', 'Unknown')}", "ERROR")
                
        # ===== HEARTBEAT =====
        elif topic == MQTTTopics.HEARTBEAT or "heartbeat" in topic:
            # Actualizar timestamp de último heartbeat
            state.status["last_heartbeat"] = payload.get("timestamp")
            
    except json.JSONDecodeError as e:
        state.log(f"Invalid JSON in MQTT message: {e}", "ERROR")
    except Exception as e:
        state.log(f"Error processing MQTT message: {str(e)}", "ERROR")


def start_mqtt(broker_host=None):
    """
    Inicia el cliente MQTT y conecta al broker.
    
    Args:
        broker_host: IP o hostname del broker (opcional, usa config si no se especifica)
    """
    global mqtt_client
    
    host = broker_host or MQTT_BROKER
    
    try:
        # Crear cliente con API versión 2 si está disponible
        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except (AttributeError, TypeError):
            # Fallback para versiones antiguas de paho-mqtt
            client = mqtt.Client()
        
        # Desconectar cliente anterior si existe
        if mqtt_client is not None:
            try:
                mqtt_client.loop_stop()
                mqtt_client.disconnect()
            except:
                pass
                
        mqtt_client = client
        
        # Configurar callbacks
        client.on_connect = on_mqtt_connect
        client.on_disconnect = on_mqtt_disconnect
        client.on_message = on_mqtt_message
        
        # Configurar opciones
        client.reconnect_delay_set(min_delay=1, max_delay=60)
        
        # Configurar Autenticación
        if MQTT_USER and MQTT_PASS:
            client.username_pw_set(MQTT_USER, MQTT_PASS)
            state.log(f"MQTT Auth configured: {MQTT_USER}", "INFO")
        
        state.log(f"Connecting to MQTT broker at {host}:{MQTT_PORT}...", "INFO")
        
        try:
            client.connect(host, MQTT_PORT, keepalive=60)
            client.loop_start()
        except Exception as e:
            state.log(f"MQTT connection failed: {e}", "WARN")
            state.status["connection"] = "DISCONNECTED"
            
        # Iniciar simulación en paralelo (se auto-regula según estado de conexión)
        start_simulation()
        
    except Exception as e:
        state.log(f"Fatal error in MQTT thread: {e}", "ERROR")
        state.status["connection"] = "DISCONNECTED"
        # Asegurar que la simulación funcione como fallback
        start_simulation()


def stop_mqtt():
    """Detiene el cliente MQTT limpiamente."""
    global mqtt_client, _reconnect_timer
    
    if _reconnect_timer:
        _reconnect_timer.cancel()
        _reconnect_timer = None
        
    if mqtt_client:
        try:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        except:
            pass
        mqtt_client = None
        
    state.log("MQTT client stopped", "INFO")
