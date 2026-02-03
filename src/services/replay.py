import sqlite3
import threading
import time
import datetime
from src.state import state, db_manager # Import db_manager just to access DB_FILE constant if needed
from src.services.database import DB_FILE

class ReplayService:
    def __init__(self):
        self.running = False
        self.thread = None
        self.paused = False
        self.speed = 1.0 # 1x speed
        self.current_index = 0
        self.data_buffer = []
        self.progress = 0  # 0-100 percent
        self.current_time_str = "00:00"
        self.total_time_str = "00:00"
        
    def load_mission_data(self):
        """Loads all sensor data from the DB for replay."""
        try:
            conn = sqlite3.connect(DB_FILE)
            c = conn.cursor()
            c.execute("SELECT timestamp, ppm, co2, temperature, humidity, voltage, current FROM sensors ORDER BY timestamp ASC")
            self.data_buffer = c.fetchall()
            conn.close()
            
            # Calculate total duration
            if len(self.data_buffer) >= 2:
                try:
                    start = datetime.datetime.fromisoformat(self.data_buffer[0][0])
                    end = datetime.datetime.fromisoformat(self.data_buffer[-1][0])
                    total_secs = int((end - start).total_seconds())
                    self.total_time_str = f"{total_secs // 60:02d}:{total_secs % 60:02d}"
                except:
                    self.total_time_str = "??:??"
            
            state.log(f"Misión cargada para Replay: {len(self.data_buffer)} puntos de datos", "SUCCESS")
            return True
        except Exception as e:
            state.log(f"Error cargando replay: {e}", "ERROR")
            return False

    def start_replay(self):
        if not self.data_buffer and not self.load_mission_data():
             return

        if self.running:
            return

        self.running = True
        self.paused = False
        state.status["mode"] = "REPLAY"
        state.status["connection"] = "REPLAY FILE"
        
        self.thread = threading.Thread(target=self._replay_loop, daemon=True)
        self.thread.start()
        state.log("Iniciando reproducción de misión...", "REPLAY")

    def stop_replay(self):
        self.running = False
        self.current_index = 0
        self.progress = 0
        self.current_time_str = "00:00"
        state.status["mode"] = "INIT"
        state.log("Replay detenido.", "INFO")

    def toggle_pause(self):
        self.paused = not self.paused
        state.log("Replay Pausado" if self.paused else "Replay Continuado", "INFO")

    def _replay_loop(self):
        if not self.data_buffer:
            return

        last_time = None
        
        while self.running and self.current_index < len(self.data_buffer):
            if self.paused:
                time.sleep(0.1)
                continue
                
            row = self.data_buffer[self.current_index]
            # row: (timestamp_str, ppm, co2, temp, hum, volt, curr)
            ts_str, ppm, co2, temp, hum, volt, curr = row
            
            try:
                # Calculate delay to match real-time (adjusted by speed)
                current_timestamp = datetime.datetime.fromisoformat(ts_str)
                
                if last_time is not None:
                    delta = (current_timestamp - last_time).total_seconds()
                    if delta > 0:
                        time.sleep(delta / self.speed)
                
                last_time = current_timestamp
                
                # Update State
                state.update_sensor_data(
                    ppm=ppm, co2=co2, temp=temp, hum=hum, volt=volt, curr=curr,
                    timestamp=current_timestamp
                )
                
                # Update progress
                self.progress = int((self.current_index / len(self.data_buffer)) * 100)
                
                # Calculate elapsed time
                if self.data_buffer:
                    try:
                        start = datetime.datetime.fromisoformat(self.data_buffer[0][0])
                        elapsed_secs = int((current_timestamp - start).total_seconds())
                        self.current_time_str = f"{elapsed_secs // 60:02d}:{elapsed_secs % 60:02d}"
                    except:
                        pass
                
                self.current_index += 1
                
            except Exception as e:
                print(f"Replay Parsing Error: {e}")
                
        self.running = False
        state.log("Fin de la reproducción de misión.", "SUCCESS")
        state.status["mode"] = "FINISHED"

replay_service = ReplayService()
