import json
import os

def load_config():
    default_config = {
        "mqtt_broker": "localhost",
        "mqtt_port": 1883,
        "mqtt_user": "YOUR_MQTT_USER",
        "mqtt_pass": "YOUR_MQTT_PASSWORD",
        "protocol_version": "2.2",
        "robot_ip": "192.168.1.XXX", # Main ESP32 (Motors/Sensors)
        "camera_ip": "192.168.1.XXX", # ESP32-CAM
        "camera_port": 81
    }
    try:
        # Look for config in the root directory (up two levels from src/config.py if run from module, 
        # or current dir if run from main)
        # We assume main.py is in the root, so config.json should be there.
        config_path = "config.json" 
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                user_config = json.load(f)
                default_config.update(user_config)
    except Exception as e:
        print(f"⚠️ Error loading config.json: {e}. Using defaults.")
    return default_config

CONFIG = load_config()

MQTT_BROKER = CONFIG["mqtt_broker"]
MQTT_PORT = CONFIG["mqtt_port"]
MQTT_USER = CONFIG.get("mqtt_user")
MQTT_PASS = CONFIG.get("mqtt_pass")
PROTOCOL_VERSION = CONFIG.get("protocol_version", "2.1")
ROBOT_IP = CONFIG["robot_ip"]
CAMERA_IP = CONFIG.get("camera_ip", ROBOT_IP) # Fallback to robot_ip if not set
CAMERA_PORT = CONFIG["camera_port"]
