import numpy as np
from datetime import datetime
import time
from threading import Timer
import math
from sklearn.neighbors import NearestNeighbors
from scipy.interpolate import lagrange

from simple_pid import PID
from pyquaternion import Quaternion

from GeometryHelpers import distance, circle_intersects_with_line, CHECKPOINT_DISTANCE_THRESH

def initialise_pid(fsds_car):
    # PID Controllers for Steering and Throttle
        
    #fsds_car.steering_pid = PID(0.75, 0.1, 1, setpoint=0)
    fsds_car.steering_pid = PID(1, 0, 0.75, setpoint=0)
    fsds_car.last_pid_reset = time.time()
    #fsds_car.steering_pid.output_limits = (-1.0, 1.0)
    fsds_car.throttle_pid = PID(1, 0.1, 0.05, setpoint=4)
    fsds_car.throttle_pid.output_limits = (0.0, 1.0)    

def calculate_steering(fsds_car):
    # TODO : Compute steering better
    
    z = np.polyfit(fsds_car.local_checkpoints['x'], fsds_car.local_checkpoints['y'], 2)
    p = np.poly1d(z)
    
    #p = lagrange(fsds_car.local_checkpoints['x'], fsds_car.local_checkpoints['y'])

    dp = np.polyder(p)
    d2p = np.polyder(dp)

    # TODO : Use np array to compute curvature faster
    curvature = 0.0
    for i in range(-100, 2500):
        curvature += d2p(i) / (1 + (dp(i))**2)**1.5

    #print(curvature)
    #curvature = curvature * 0.5

    num_points = 1000
    search_radius = 3000 # 5 Meter radius
    
    fsds_car.fitted_curve['x'] = np.linspace(-search_radius, search_radius, num_points) 
    fsds_car.fitted_curve['y'] = p(fsds_car.fitted_curve['x'])

    # TODO : use np to find min_dist faster ?
    #min_dist = distance(fsds_car.car_x, fsds_car.car_y, fsds_car.fitted_curve['x'][0], fsds_car.fitted_curve['y'][0])
    min_dist = distance(0, 0, fsds_car.fitted_curve['x'][0], fsds_car.fitted_curve['y'][0])
    for i in range(num_points):
        d = distance(0, 0, fsds_car.fitted_curve['x'][i], fsds_car.fitted_curve['y'][i])
        if d < min_dist:
            min_dist = d

    dist = min_dist

    intersects, line_dist, point, side_of_line = circle_intersects_with_line(0, 0, CHECKPOINT_DISTANCE_THRESH, fsds_car.local_checkpoints['x'][0], fsds_car.local_checkpoints['y'][0], fsds_car.local_checkpoints['x'][1], fsds_car.local_checkpoints['y'][1])

    if side_of_line < 0:
        dist = -dist

    now = time.time()
    if now - fsds_car.last_pid_reset > 1:
        #print("PID Reset")
        #fsds_car.steering_pid = PID(2, 0, 0.5, setpoint=0)
        #fsds_car.last_pid_reset = now
        pass

    steering = fsds_car.steering_pid(dist)
    
    # To invert steering logic
    #if side_of_line < 0: steering = -steering

    final_steering = steering / 140
    if final_steering>1:
        final_steering = 1
    elif final_steering<-1:
        final_steering = -1

    #print("car_quaternion=", car_quaternion, ", angle_in_deg=", angle_in_deg, ", car_quaternion.axis=", car_quaternion.axis, sep='\t')
    #print("steering=", steering, ", dist=", dist, ", cp_q=", (fsds_car.local_checkpoints['x'][0], fsds_car.local_checkpoints['y'][0], fsds_car.local_checkpoints['z'][0]), "final_steering=", final_steering, sep='\t')

    return final_steering

def calculate_throttle(fsds_car, target_speed, steering_angle, max_throttle=0.35):
    speed_limit = target_speed
    gps = fsds_car.gps_data

    # TODO : Use fsds_car.throttle_pid to calculate throttle
    # TODO : Integrate brak and throttle into one function

    # Calculate the velocity in the vehicle's frame
    speed = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))

    # the lower the velocity, the more throttle, up to max_throttle
    return max_throttle * max(1.0 - steering_angle**2 - (speed/speed_limit)**2, 0.0)

def calculate_brake(fsds_car, target_speed, steering_angle, max_braking=0.1):
    speed_limit = target_speed
    gps = fsds_car.gps_data

    # Calculate the velocity in the vehicle's frame
    speed = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))

    # the lower the velocity, the more throttle, up to max_throttle
    return -max_braking * min(1.0 - (steering_angle*1.0)**2 - (speed/speed_limit)**2, 0.0)
