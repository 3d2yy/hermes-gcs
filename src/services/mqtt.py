import json
import threading
import paho.mqtt.client as mqtt
import numpy as np
from src.config import MQTT_BROKER, MQTT_PORT
from src.state import state
from src.services.simulation import start_simulation

mqtt_client = None

def publish_command(topic, payload):
    global mqtt_client
    if mqtt_client and state.status["connection"] == "ONLINE":
        try:
            if isinstance(payload, dict):
                payload = json.dumps(payload)
            mqtt_client.publish(topic, payload)
            return True
        except Exception as e:
            state.log(f"Error publishing MQTT: {e}", "ERROR")
    return False

def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        state.log("Conexión MQTT establecida", "SUCCESS")
        state.status["connection"] = "ONLINE"
        state.status["mode"] = "MQTT"
        topics = [
            (MQTTTopics.MQ2_DATA, 0),
            (MQTTTopics.MQ2_ALERT, 0),
            (MQTTTopics.ENVIRONMENT, 0),
            (MQTTTopics.IMU, 0),
            (MQTTTopics.ULTRASONIC, 0),
            (MQTTTopics.AUDIO, 0),
            (MQTTTopics.POSITION, 0),
            (MQTTTopics.RADAR, 0),
            (MQTTTopics.STATUS, 0)
        ]
        for topic, qos in topics:
            client.subscribe(topic, qos)
    else:
        state.log(f"Error de conexión MQTT: código {rc}", "ERROR")

def on_mqtt_disconnect(client, userdata, rc):
    state.status["connection"] = "DISCONNECTED"
    state.status["mode"] = "ESPERANDO"
    state.log("MQTT Desconectado. Retomando simulación en 5s...", "WARN")

def on_mqtt_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        topic = msg.topic
        
        if MQTTTopics.MQ2_DATA in topic:
            sensor_data = payload.get("sensor_data", {})
            ppm = sensor_data.get("ppm", 0)
            volt = sensor_data.get("voltage", 0)
            
            state.update_sensor_data(ppm=ppm, volt=volt)
            state.add_gas_reading(state.robot_position["x"], state.robot_position["y"], ppm)
            
            status = sensor_data.get("alert_status", "normal").lower()
            if status == "critico":
                state.status["alert_level"] = "CRITICAL"
            elif status in ["peligro", "advertencia"]:
                state.status["alert_level"] = "WARNING"
            else:
                state.status["alert_level"] = "NORMAL"

        elif MQTTTopics.MQ2_ALERT in topic:
            state.log(f"ALERTA MQ-2: {payload.get('message')}", "WARN")
                
        elif MQTTTopics.ENVIRONMENT in topic:
            state.update_sensor_data(
                co2=payload.get("co2"), 
                temp=payload.get("temperature"), 
                hum=payload.get("humidity")
            )
            
        elif MQTTTopics.POWER in topic or "sensors/power" in topic:
            state.update_sensor_data(volt=payload.get("voltage"), curr=payload.get("current"))
            state.current_values["rssi"] = payload.get("rssi", -50)
            
        elif MQTTTopics.IMU in topic:
            # Sync with MPU6050Manager format from main (1).py
            state.imu.update(payload)
            
        elif MQTTTopics.ULTRASONIC in topic:
            dist = payload.get("distance_cm")
            if dist is not None:
                state.current_values["ultrasonic"] = dist
                
        elif MQTTTopics.POSITION in topic:
            state.update_robot_position(payload.get("x", 25), payload.get("y", 25), payload.get("theta", 0))
            
        elif MQTTTopics.RADAR in topic:
            if "distances" in payload:
                state.radar_distances = np.array(payload["distances"])
                
    except Exception as e:
        state.log(f"Error procesando MQTT: {str(e)}", "ERROR")

def start_mqtt(broker_host=None):
    host = broker_host or MQTT_BROKER
    try:
        try:
            client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except Exception:
            client = mqtt.Client()
        
        global mqtt_client
        if mqtt_client:
            try:
                mqtt_client.loop_stop()
                mqtt_client.disconnect()
            except:
                pass
        mqtt_client = client
        
        client.on_connect = on_mqtt_connect
        client.on_disconnect = on_mqtt_disconnect
        client.on_message = on_mqtt_message
        
        state.log(f"Conectando a {host}:{MQTT_PORT}...", "INFO")
        try:
            client.connect(host, MQTT_PORT, keepalive=5)
            client.loop_start()
        except:
            state.log("MQTT Inicial no disponible. Continuando...", "WARN")
            
        # Start simulation in parallel (it will self-regulate)
        start_simulation()
        
    except Exception as e:
        state.log(f"Error fatal MQTT thread: {e}", "ERROR")
        start_simulation()
