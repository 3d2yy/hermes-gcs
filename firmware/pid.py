import time

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self._integral = 0
        self._last_error = 0
        self._last_time = time.ticks_ms()
        
    def compute(self, input_val):
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_time) / 1000.0
        
        # Reset if dt is too large (system paused/lag spike) or zero
        if dt > 1.0 or dt <= 0: 
            dt = 0.0
            
        error = self.setpoint - input_val
        
        if dt > 0:
            self._integral += (error * dt)
            # Simple anti-windup could be added here if needed
            derivative = (error - self._last_error) / dt
        else:
            derivative = 0
            
        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * derivative)
        
        self._last_error = error
        self._last_time = now
        
        return output
        
    def reset(self):
        self._integral = 0
        self._last_error = 0
        self._last_time = time.ticks_ms()
