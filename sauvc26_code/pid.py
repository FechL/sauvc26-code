#!/usr/bin/env python3
import time

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def compute(self, measurement):
        now = time.time()
        dt = now - self.prev_time

        if dt <= 0:
            return 0.0

        error = self.setpoint - measurement

        # Proportional
        P = self.kp * error

        # Integral
        self.integral += error * dt
        I = self.ki * self.integral

        # Derivative
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative

        output = P + I + D

        self.prev_error = error
        self.prev_time = now

        return output
