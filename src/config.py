import json
import os

def load_config():
    default_config = {
        "mqtt_broker": "192.168.2.103",
        "mqtt_port": 1883,
        "robot_ip": "192.168.2.100",
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
ROBOT_IP = CONFIG["robot_ip"]
CAMERA_PORT = CONFIG["camera_port"]
