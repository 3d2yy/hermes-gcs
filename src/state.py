from collections import deque
import datetime
import numpy as np
from .services.database import db_manager

MAX_POINTS = 120
MAX_GAS_MAP_POINTS = 500

class SystemState:
    """Centralized state management for all sensor data and system status."""
    
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(SystemState, cls).__new__(cls)
            cls._instance.initialized = False
        return cls._instance
    
    def __init__(self):
        if self.initialized:
            return
        
        self.ppm = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
        self.co2 = deque([400]*MAX_POINTS, maxlen=MAX_POINTS)
        self.temperature = deque([25]*MAX_POINTS, maxlen=MAX_POINTS)
        self.humidity = deque([50]*MAX_POINTS, maxlen=MAX_POINTS)
        self.voltage = deque([12.6]*MAX_POINTS, maxlen=MAX_POINTS)
        self.current_draw = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
        self.audio_confidence = deque([0]*MAX_POINTS, maxlen=MAX_POINTS)
        self.timestamps = deque([datetime.datetime.now()]*MAX_POINTS, maxlen=MAX_POINTS)
        
        self.gas_map_points = []
        self.robot_position = {"x": 25.0, "y": 25.0, "theta": 0.0}
        self.robot_path = deque(maxlen=1000)
        
        self.current_values = {
            "ppm": 0, "co2": 400, "temperature": 25.0, "humidity": 50.0,
            "voltage": 12.6, "current": 0.0, "rssi": -50, "battery_percent": 100,
        }
        
        self.status = {
            "connection": "DISCONNECTED", "mode": "INIT", "alert_level": "NORMAL",
            "audio_class": "SILENCE", "audio_confidence": 0, "camera_active": False,
        }
        
        self.imu = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81}
        self.radar_angles = np.linspace(0, 360, 72)
        self.radar_distances = np.full(72, 5.0)
        self.logs = deque(maxlen=100)
        self.acoustic_detections = deque(maxlen=50)
        
        # Cache for expensive heatmap calculations
        self.cached_zi = None
        self.last_heatmap_count = -1
        
        self.initialized = True
        self.log("Sistema H.E.R.M.E.S. GCS v2.0 iniciado")
        
    def log(self, message, level="INFO"):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        icons = {"INFO": "ℹ️", "WARN": "⚠️", "ERROR": "❌", "SUCCESS": "✅", "DETECT": "🎯", "REPLAY": "⏪"}
        self.logs.appendleft(f"[{ts}] {icons.get(level, '📝')} {message}")
        
    def update_sensor_data(self, ppm=None, co2=None, temp=None, hum=None, volt=None, curr=None, timestamp=None):
        now = timestamp if timestamp else datetime.datetime.now()
        self.timestamps.append(now)
        
        if ppm is not None:
            self.current_values["ppm"] = ppm
            self.ppm.append(ppm)
        else:
            self.ppm.append(self.ppm[-1] if self.ppm else 0)
            
        if co2 is not None:
            self.current_values["co2"] = co2
            self.co2.append(co2)
        else:
            self.co2.append(self.co2[-1] if self.co2 else 400)
            
        if temp is not None:
            self.current_values["temperature"] = temp
            self.temperature.append(temp)
        else:
            self.temperature.append(self.temperature[-1] if self.temperature else 25)
            
        if hum is not None:
            self.current_values["humidity"] = hum
            self.humidity.append(hum)
        else:
            self.humidity.append(self.humidity[-1] if self.humidity else 50)
            
        if volt is not None:
            self.current_values["voltage"] = volt
            self.voltage.append(volt)
            self.current_values["battery_percent"] = max(0, min(100, int((volt - 9.0) / 3.6 * 100)))
        else:
            self.voltage.append(self.voltage[-1] if self.voltage else 12.6)
            
        if curr is not None:
            self.current_values["current"] = curr
            self.current_draw.append(curr)
        else:
            self.current_draw.append(self.current_draw[-1] if self.current_draw else 0)

        # Save to DB (Async) - ONLY if NOT in REPLAY mode
        if self.status["mode"] != "REPLAY":
            db_manager.add_sensor_data(
                self.current_values["ppm"], self.current_values["co2"],
                self.current_values["temperature"], self.current_values["humidity"],
                self.current_values["voltage"], self.current_values["current"]
            )
            
    def add_gas_reading(self, x, y, ppm):
        self.gas_map_points.append({"x": x, "y": y, "ppm": ppm, "timestamp": datetime.datetime.now()})
        if len(self.gas_map_points) > MAX_GAS_MAP_POINTS:
            self.gas_map_points = self.gas_map_points[-MAX_GAS_MAP_POINTS:]
        
        # Save to DB (Async) - ONLY if NOT in REPLAY mode
        if self.status["mode"] != "REPLAY":
            db_manager.add_gas_point(x, y, ppm)
            
    def update_robot_position(self, x, y, theta):
        self.robot_position = {"x": x, "y": y, "theta": theta}
        self.robot_path.append((x, y))
        
    def add_acoustic_detection(self, classification, confidence, direction=None):
        self.acoustic_detections.appendleft({
            "class": classification, "confidence": confidence,
            "direction": direction, "timestamp": datetime.datetime.now()
        })
        self.status["audio_class"] = classification
        self.status["audio_confidence"] = confidence

state = SystemState()
