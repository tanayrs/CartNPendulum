import time

class TiltController:
    def __init__(self, Kp=25.0, Ki=0.0, Kd=0.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = time.time()

    def update(self, current_angle, gyro_rate, target_angle=0.0):
        now = time.time()
        dt = now - self.prev_time
        error = target_angle - current_angle
        
        # Anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, 20), -20)
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        
        output = (self.Kp * error) + (self.Ki * self.integral) - (self.Kd * gyro_rate)
        
        self.prev_error = error
        self.prev_time = now
        
        return output

