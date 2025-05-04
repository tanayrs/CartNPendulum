'''
PID Controller

Features:
- Classical PID control algorithm with tunable gains
- Anti-windup protection to prevent integral term saturation
- Direct use of gyroscope measurements for derivative action
- Safety limits to prevent aggressive control at large angles
- Smooth control output for reliable motor actuation

Usage:
    # Create controller with custom gains
    controller = TiltController(Kp=25.0, Ki=0.5, Kd=1.0)
    
    # In main control loop
    while running:
        # Get pendulum state from sensors
        current_angle = imu.get_angle()
        gyro_rate = imu.get_gyro_rate()
        
        # Calculate control output
        control_signal = controller.update(current_angle, gyro_rate)
        
        # Apply control to motor
        motor.set_speed(control_signal)
        
    # Reset controller state when needed
    controller.reset()

Parameters:
    Kp: Proportional gain (25.0 default)
    Ki: Integral gain (0.0 default)
    Kd: Derivative gain (0.0 default)
'''

# imports
import time

class TiltController:
    def __init__(self, Kp=25.0, Ki=0.0, Kd=0.0):
        # Controller gains
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.reset()

    def reset(self):
        # Reset controller state
        self.integral = 0.0  # Reset accumulated error
        self.prev_error = 0.0  # Reset previous error
        self.prev_time = time.time()  # Reset time tracker

    def update(self, current_angle, gyro_rate, target_angle=0.0):
        now = time.time()
        dt = now - self.prev_time  # Calculate time difference
        error = target_angle - current_angle  # Calculate error
        
        # Anti-windup: limit integral term to prevent excessive accumulation
        self.integral += error * dt
        self.integral = max(min(self.integral, 20), -20)  # Clamp integral between -20 and 20
        
        # Calculate PID output: P term + I term - D term (using gyro rate directly)
        output = (self.Kp * error) + (self.Ki * self.integral) - (self.Kd * gyro_rate)

        # Safety check: disable control if angle is too large
        if abs(current_angle) > 20:
            output = 0
        
        self.prev_time = now  # Update previous time
        
        return output