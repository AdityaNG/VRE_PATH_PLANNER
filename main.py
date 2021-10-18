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

from simple_pid import PID
from pyquaternion import Quaternion

sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds

from SimEnv import Env

parser = argparse.ArgumentParser()
parser.add_argument('-exe', '--executable', type=str, default=os.path.abspath(os.path.join(os.getenv("HOME"), "Apps/fsds-v2.0.0-linux/FSDS.sh")), help='Path to Airshim.sh')
parser.add_argument('-t', '--track', type=str, default='TrainingMap', help='Track name')
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
    client.enableApiControl(True)
    print("API Control enabled: %s" % client.isApiControlEnabled())

    fsds_car = Env(client)

    while True:
        car_controls = fsds.CarControls()

        target_speed = 4 # m/s
        fsds_car.step()
        """
        car_controls.steering = fsds_car.calculate_steering()
        car_controls.throttle = fsds_car.calculate_throttle(target_speed, car_controls.steering)
        car_controls.brake = fsds_car.calculate_brake(target_speed, car_controls.steering)
        #car_controls.brake = 0

        #self.log(str(car_controls.steering) + " " + str(car_controls.throttle) + " " + str(car_controls.brake))
        """
        #client.setCarControls(car_controls)
        pass

    camera_data = {}

# Process to call images from the sim and process them to generate point_cloud_array
main_control_loop_proc = Process(target=main_control_loop, args=())
main_control_loop_proc.start()

# Start blocking start_graph call
# plotter.start_graph(point_cloud_array)
input("enter to quit")

# Once graph window is closed, kill the main_control_loop process

os.killpg(os.getpgid(fsds_proc.pid), signal.SIGTERM)  # Send the signal to all the process groups
os.killpg(os.getpgid(main_control_loop_proc.pid), signal.SIGTERM)