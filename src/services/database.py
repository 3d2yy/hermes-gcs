import sqlite3
import queue
import threading
import datetime
import os

DB_FILE = "mission_data.db"

class DatabaseManager:
    """Handles database operations in a separate thread to prevent UI blocking."""
    
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(DatabaseManager, cls).__new__(cls)
            cls._instance.initialized = False
        return cls._instance

    def __init__(self, db_file=DB_FILE):
        if self.initialized:
            return
        self.db_file = db_file
        self.queue = queue.Queue()
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker, daemon=True)
        self.worker_thread.start()
        self._init_db()
        self.initialized = True

    def _init_db(self):
        try:
            conn = sqlite3.connect(self.db_file)
            c = conn.cursor()
            c.execute('''CREATE TABLE IF NOT EXISTS sensors
                         (timestamp TEXT, ppm REAL, co2 REAL, temperature REAL, 
                          humidity REAL, voltage REAL, current REAL)''')
            c.execute('''CREATE TABLE IF NOT EXISTS gas_map
                         (timestamp TEXT, x REAL, y REAL, ppm REAL)''')
            conn.commit()
            conn.close()
        except Exception as e:
            print(f"❌ DB Init Error: {e}")

    def add_sensor_data(self, ppm, co2, temp, hum, volt, curr):
        data = {
            "type": "sensor",
            "timestamp": datetime.datetime.now().isoformat(),
            "values": (ppm, co2, temp, hum, volt, curr)
        }
        self.queue.put(data)

    def add_gas_point(self, x, y, ppm):
        data = {
            "type": "gas",
            "timestamp": datetime.datetime.now().isoformat(),
            "values": (x, y, ppm)
        }
        self.queue.put(data)

    def _worker(self):
        print("💾 Database Manager started.")
        while self.running:
            try:
                # Get a batch of items
                items = []
                try:
                    # Block for first item
                    items.append(self.queue.get(timeout=1.0))
                    # Get more if available, up to 50
                    for _ in range(49):
                        try:
                            items.append(self.queue.get_nowait())
                        except queue.Empty:
                            break
                except queue.Empty:
                    continue

                if not items:
                    continue

                conn = sqlite3.connect(self.db_file)
                c = conn.cursor()
                
                sensor_batch = []
                gas_batch = []

                for item in items:
                    if item["type"] == "sensor":
                        vals = item["values"]
                        sensor_batch.append((item["timestamp"], *vals))
                    elif item["type"] == "gas":
                        vals = item["values"]
                        gas_batch.append((item["timestamp"], *vals))
                
                if sensor_batch:
                    c.executemany("INSERT INTO sensors VALUES (?, ?, ?, ?, ?, ?, ?)", sensor_batch)
                if gas_batch:
                    c.executemany("INSERT INTO gas_map VALUES (?, ?, ?, ?)", gas_batch)
                
                conn.commit()
                conn.close()

                # Mark tasks as done
                for _ in items:
                    self.queue.task_done()

            except Exception as e:
                print(f"❌ DB Worker Error: {e}")

# Initialize global DB manager
db_manager = DatabaseManager()
