import time
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 18.2 # kph set in waypoint loader

class Controller(object):
    def __init__(self, vehicle_mass, brake_deadband, wheel_radius, decel_limit, wheel_base, 
                 steer_ratio, max_lat_accel, max_steer_angle, Kp, Ki, Kd):

        min_speed = 1.0 * ONE_MPH
        self.throttle_pid = PID(Kp, Ki, Kd)
        self.yaw_control = YawController(wheel_base,
                                         steer_ratio,
                                         min_speed,
                                         max_lat_accel,
                                         max_steer_angle)

        self.brake_deadband = brake_deadband
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius
        self.decel_limit = decel_limit
        self.last_time = None


    def control(self, target_vel, target_omega, current_vel, dbw_enabled):
        # Calculate the desired throttle based on target_vel, target_omega and current_vel
        # target_vel and target_omega are desired linear and angular velocities

        if self.last_time is None or not dbw_enabled:
            self.last_time = time.time()
            return 0.0, 0.0, 0.0

        dt = time.time() - self.last_time

        # Assumed maximum speed is in mph

        error = min(target_vel.x, MAX_SPEED * 0.277778) - current_vel.x

        throttle = self.throttle_pid.step(error, dt)

        # Max throttle is 1.0
        throttle = max(0.0, min(1.0, throttle))

        # Brake or decelerate only if the target velocity is lower than the current velocity
        # Brake value is in N/m and is calculated using car mass, acceleration and wheel radius
        # longitudinal force = mass of car * acceleration (or deceleration)
        # Torque = longitudinal force * wheel radius, which is supplied as brake value
        # Further refinements can be done by adding the mass of fuel and the passengers to the mass
        # of the car in real world scenario
        if error < 0: # Needs to decelerate
            deceleration = abs(error) / dt
            if abs(deceleration) > abs(self.decel_limit)*500:
                deceleration = self.decel_limit*500  # Limited to decelartion limits
            longitudinal_force = self.vehicle_mass * deceleration
            brake = longitudinal_force * self.wheel_radius
            if brake < self.brake_deadband:
                brake = 0.0
            throttle = 0.0
        else:
            brake = 0.0

        # Steering control is using Yaw Control..
        steer = self.yaw_control.get_steering(target_vel.x, target_omega.z, current_vel.x)

        self.last_time = time.time()

        return throttle, brake, steer
