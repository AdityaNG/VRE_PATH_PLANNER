from glob import glob
import cv2
import numpy as np
import os
import time
import math
import pprint
import argparse
import threading
import subprocess
from datetime import datetime
import signal
import matplotlib.pyplot as plt
from multiprocessing import Process, Array, Pool, Queue
import ctypes
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

import DrivingPolicy

from SimEnv import Env
from TrackPlot import TrackPlot

parser = argparse.ArgumentParser()
parser.add_argument('-exe', '--executable', type=str, default=os.path.abspath(os.path.join(os.getenv("HOME"), "Apps/fsds-v2.0.0-linux/FSDS.sh")), help='Path to Airshim.sh')
parser.add_argument('-t', '--track', type=str, default='TrainingMap', help='Track name')
parser.add_argument('-r', '--render', type=int, default=10, help='Render every r frames')
parser.add_argument('-a', '--autonomous', action='store_true', help='autonomous driving')
parser.add_argument('-s', '--settings', type=str, default=os.path.abspath(os.path.join(os.getenv("HOME"), "VRE_PATH_PLANNER/settings.json")), help='Path to Airshim settings.json')
args = parser.parse_args()

# Logging the data to disk
current_date = str(datetime.now().strftime("%d-%m-%Y_%H%M%S"))
logfilename = 'logs/{}_LOG.txt'.format(current_date)
simfilename = 'logs/{}_SIM.txt'.format(current_date)
if not os.path.exists(os.path.dirname(logfilename)):
    try:
        os.makedirs(os.path.dirname(logfilename))
    except OSError as exc: # Guard against race condition
        if exc.errno != errno.EEXIST:
            raise

# Open sim log file
simfile = open(simfilename, 'w')
# Launch the sim process
fsds_proc = subprocess.Popen([args.executable, '/Game/'+args.track+'?listen', '-WINDOWED', '-ResX=640', '-ResY=480', '--settings', args.settings], stdout=simfile)

global client
connection_failed = True
while connection_failed:
    try:
        client = fsds.FSDSClient()
        client.confirmConnection()
        connection_failed = False
        print(". done", end="")
    except:
        connection_failed = True
        time.sleep(1)
# Wait for fsds to launch
# TODO : Replace wait with polling
#time.sleep(5)

        
def main_control_loop():
    """
        main_control_loop is launched as a subprocess
    """

    white_bg = np.zeros((144,256,3))
    pp = pprint.PrettyPrinter(indent=4)
    client = fsds.FSDSClient()
    client.confirmConnection()
    #client.reset()
    client.enableApiControl(args.autonomous)
    print("API Control enabled: %s" % client.isApiControlEnabled())

    fsds_car = Env(client, local_checkpoint_radius=8)
    fsds_car.step()

    DrivingPolicy.initialise_pid(fsds_car=fsds_car)
    t_plotter = TrackPlot(fsds_car.referee_state)

    frame_count = 0

    accel_2d_history = {'x': [], 'y': []}

    while True:

        target_speed = 4 # m/s

        steering = DrivingPolicy.calculate_steering(fsds_car=fsds_car)
        throttle = DrivingPolicy.calculate_throttle(fsds_car=fsds_car, target_speed=target_speed, steering_angle=steering)
        brake = DrivingPolicy.calculate_brake(fsds_car=fsds_car, target_speed=target_speed, steering_angle=steering)

        if not args.autonomous:
            fsds_car.step()
        else:
            fsds_car.step(steering, throttle, brake)

        fsds_car.car_x = fsds_car.vehicle_pose['position']['x_val']*100 + fsds_car.referee_state.initial_position.x 
        fsds_car.car_y = -fsds_car.vehicle_pose['position']['y_val']*100 + fsds_car.referee_state.initial_position.y
        
        accel_x = fsds_car.car_state.kinematics_estimated.linear_acceleration.x_val
        accel_y = fsds_car.car_state.kinematics_estimated.linear_acceleration.y_val
        accel_z = fsds_car.car_state.kinematics_estimated.linear_acceleration.z_val
        accel_2d = {'x':accel_y, 'y':accel_x}
        accel_2d_history['x'].append(accel_y)
        accel_2d_history['y'].append(accel_x)
        
        if len(accel_2d_history['x']) > 5:
            accel_2d_history['x'].pop(0)
            accel_2d_history['y'].pop(0)

        # TODO Make render an async call
        if frame_count % args.render==0 or not args.autonomous:
            if args.autonomous: fsds_car.simPause(True)
            t_plotter.update_car_position(fsds_car.car_x, fsds_car.car_y, fsds_car.cones)
            t_plotter.render(fsds_car.referee_state, fsds_car.local_checkpoints, fsds_car.fitted_curve, accel_2d_history)
            if args.autonomous: fsds_car.simPause(False)

        frame_count+=1



"""
# Process to call images from the sim and process them to generate point_cloud_array
main_control_loop_proc = Process(target=main_control_loop, args=())
main_control_loop_proc.start()

# Start blocking start_graph call
# plotter.start_graph(point_cloud_array)
input("enter to quit")
# """

main_control_loop()

# Once graph window is closed, kill the main_control_loop process

os.killpg(os.getpgid(fsds_proc.pid), signal.SIGTERM)  # Send the signal to all the process groups
os.killpg(os.getpgid(main_control_loop_proc.pid), signal.SIGTERM)