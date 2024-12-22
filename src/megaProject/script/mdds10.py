#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

class mdds10:
    def __init__(self, pwm_pin1 , pwm_pin2, dir1_pin, dir2_pin):
        """
        Initialize motor driver for rover movement
        :param pwm_pin: Shared PWM pin for motors
        :param dir1_pin: Direction pin for left motor
        :param dir2_pin: Direction pin for right motor
        """
        GPIO.setmode(GPIO.BOARD)
        self.PWM_PIN1 = pwm_pin1
        self.PWM_PIN2 = pwm_pin2
        self.DIR1_PIN = dir1_pin  # Left motor direction
        self.DIR2_PIN = dir2_pin  # Right motor direction
        
        GPIO.setup(self.PWM_PIN1, GPIO.OUT)
        GPIO.setup(self.PWM_PIN2, GPIO.OUT)
        GPIO.setup(self.DIR1_PIN, GPIO.OUT)
        GPIO.setup(self.DIR2_PIN, GPIO.OUT)
        
        self.right_motor_speed = GPIO.PWM(self.PWM_PIN1, 100)  # 100 Hz frequency
        self.right_motor_speed.start(0)  # Start with 0% duty cycle
        
        self.left_motor_speed = GPIO.PWM(self.PWM_PIN2, 100)  # 100 Hz frequency
        self.left_motor_speed.start(0)  # Start with 0% duty cycle
    
    def move_forward(self, speed_left, speed_right):
        """Move rover forward"""
        speed_left = max(min(speed_left, 100), 0)
        speed_right = max(min(speed_right, 100), 0)
        GPIO.output(self.DIR1_PIN, GPIO.HIGH)  # Left motor forward
        GPIO.output(self.DIR2_PIN, GPIO.HIGH)  # Right motor forward
        self.right_motor_speed.ChangeDutyCycle(speed_right)
        self.left_motor_speed.ChangeDutyCycle(speed_left)
    
    def move_backward(self, speed_left, speed_right):
        """Move rover backward"""
        speed = max(min(speed, 100), 0)
        GPIO.output(self.DIR1_PIN, GPIO.LOW)   # Left motor backward
        GPIO.output(self.DIR2_PIN, GPIO.LOW)   # Right motor backward
        self.right_motor_speed.ChangeDutyCycle(speed_right)
        self.left_motor_speed.ChangeDutyCycle(speed_left)
    
    def turn_left(self, speed_left, speed_right):
        """Turn rover left (pivot)"""
        speed = max(min(speed, 100), 0)
        GPIO.output(self.DIR1_PIN, GPIO.LOW)   # Left motor backward
        GPIO.output(self.DIR2_PIN, GPIO.HIGH)  # Right motor forward
        self.right_motor_speed.ChangeDutyCycle(speed_right)
        self.left_motor_speed.ChangeDutyCycle(speed_left)
    
    def turn_right(self, speed_left, speed_right):
        """Turn rover right (pivot)"""
        speed = max(min(speed, 100), 0)
        GPIO.output(self.DIR1_PIN, GPIO.HIGH)  # Left motor forward
        GPIO.output(self.DIR2_PIN, GPIO.LOW)   # Right motor backward
        self.right_motor_speed.ChangeDutyCycle(speed_right)
        self.left_motor_speed.ChangeDutyCycle(speed_left)
    
    def stop(self):
        """Stop rover movement"""
        self.right_motor_speed.ChangeDutyCycle(0)
        self.left_motor_speed.ChangeDutyCycle(0)
        GPIO.output(self.DIR1_PIN, GPIO.LOW)
        GPIO.output(self.DIR2_PIN, GPIO.LOW)
    
    def cleanup(self):
        """Clean up GPIO settings"""
        self.right_motor_speed.stop()
        self.left_motor_speed.stop()
        GPIO.cleanup()
