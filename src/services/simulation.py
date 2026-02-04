import time
import math
import random
import threading
import numpy as np
from src.state import state

def simulation_thread():
    """Generate realistic simulated data only if NOT connected to MQTT."""
    state.log("Hilo de simulación listo. Esperando estado de conexión...", "INFO")
    
    t = 0
    while True:
        # Stop simulation if Connected, Replay or Connecting
        if state.status["connection"] in ["ONLINE", "CONNECTING..."] or state.status["mode"] == "REPLAY":
            time.sleep(1)
            continue

        state.status["mode"] = "SIMULACIÓN"
        state.status["connection"] = "SIMULATED"
        
        time.sleep(0.5)
        t += 0.5
        
        # Gas simulation
        base_ppm = 350 + 150 * math.sin(t * 0.1) + random.gauss(0, 30)
        if random.random() < 0.02:
            base_ppm += random.uniform(500, 2000)
            state.log(f"SIM: Pico de gas ({int(base_ppm)} ppm)", "WARN")
        ppm = max(200, int(base_ppm))
        
        temp = 25 + 5 * math.sin(t * 0.05) + random.gauss(0, 0.5)
        hum = 50 + 15 * math.cos(t * 0.03) + random.gauss(0, 2)
        co2 = 400 + 100 * math.sin(t * 0.02) + random.gauss(0, 20)
        voltage = 12.6 - (t * 0.001) % 3.6 + random.gauss(0, 0.05)
        current = 0.5 + random.uniform(0, 1.5)
        
        state.update_sensor_data(ppm=ppm, co2=int(co2), temp=round(temp, 1), 
                                  hum=round(hum, 1), volt=round(voltage, 2), curr=round(current, 2))
        
        # Robot movement
        theta = state.robot_position["theta"] + random.gauss(0, 0.1)
        x = state.robot_position["x"] + 0.3 * math.cos(theta)
        y = state.robot_position["y"] + 0.3 * math.sin(theta)
        x, y = max(1, min(49, x)), max(1, min(49, y))
        if x <= 1 or x >= 49 or y <= 1 or y >= 49:
            theta += math.pi / 2
        state.update_robot_position(x, y, theta)
        state.add_gas_reading(x, y, ppm)
        
        # IMU
        state.imu["roll"] = 2 * math.sin(t * 0.5) + random.gauss(0, 0.5)
        state.imu["pitch"] = 3 * math.cos(t * 0.3) + random.gauss(0, 0.5)
        state.imu["yaw"] = (theta * 180 / math.pi) % 360
        
        # Radar
        base_dist = 3 + 1.5 * np.sin(np.linspace(0, 4*np.pi, 72))
        state.radar_distances = np.clip(base_dist + np.random.uniform(-0.3, 0.3, 72), 0.1, 5.0)
        
        # Audio simulation
        if random.random() < 0.03:
            cls = random.choices(["AMBIENT", "MACHINERY", "VOICE", "SCREAM", "BREATHING"], 
                                  [0.4, 0.3, 0.15, 0.08, 0.07])[0]
            conf = random.uniform(60, 98) if cls != "AMBIENT" else random.uniform(10, 40)
            state.add_acoustic_detection(cls, conf, random.uniform(0, 360))
        else:
            state.audio_confidence.append(random.uniform(5, 25))

def start_simulation():
    threading.Thread(target=simulation_thread, daemon=True).start()
