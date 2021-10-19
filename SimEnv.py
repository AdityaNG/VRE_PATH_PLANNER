import numpy as np
import sys
import os
from datetime import datetime
import random
import time
import threading
import subprocess
import signal
import logging
from threading import Timer
import math
import itertools
from pyquaternion import Quaternion

from sklearn.neighbors import NearestNeighbors
from scipy.interpolate import lagrange

sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds
sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/AirSim/PythonClient")))
from airsim.types import Vector3r

from GeometryHelpers import distance, pointgroup_to_cone

MANUAL_DRIVING = False

global old_points
old_points = []

cones_range_cutoff = 7 #state_grid_size // 2# 10 # meters

class Space:
    def __init__(self, spaces_list):
        self.spaces_list = spaces_list
        pass

    def sample(self):
        res = []
        for r in self.spaces_list:
            #res.append(np.random.choice(r))
            res.append(random.choice(r))
        return res

class Env:
    def __init__(self, client, local_checkpoint_radius=8):
        # The rpc client connected to the simulation. Used to retrieve referee state.
        self.client = client

        self.local_checkpoint_radius = local_checkpoint_radius

        self.referee_state = None
        
        self.car_state = None

        self.kinematics = None

        self.gps_data = None

        self.lidar_data = None

        self.vehicle_pose = None

        self.collisions = None
        
        # Dictionary holding the time taken by each function
        self.timers = {}

        self.fitted_curve = {'x':[], 'y':[]}

        # Cone hit counter
        self.doo_count = 0

        # List of lap times
        self.lap_times = []

        # Dictionary of cone ground truth positions, colors
        self.cones_gt = {}

        # List of checkpoints
        # A checkpoint is a midpoint of a left-right cone pair
        # TODO : Use only x% of the checkpoints and use cones as 'obstacles'
        # At the lowest x%, see what is the average distance between each checkpoint
        self.checkpoints = []

        # Get all sim data 
        self.getAllSimData()
        
        # Compute the track boundaries using ground truth cone positions
        self.compute_track_boundaries()

        pass

    def get_local_checkpoints(self):
        self.local_checkpoints = {'x':[], 'y':[], 'z':[]}

        if self.vehicle_pose['orientation']['x_val']>0 or self.vehicle_pose['orientation']['y_val']>0 or self.vehicle_pose['orientation']['z_val']>0:
            car_quaternion = Quaternion(axis=[self.vehicle_pose['orientation']['x_val'], self.vehicle_pose['orientation']['y_val'], self.vehicle_pose['orientation']['z_val']],
                                    angle=self.vehicle_pose['orientation']['w_val'])
        else:
            car_quaternion = Quaternion(axis=[0, 0, 1], angle=self.vehicle_pose['orientation']['w_val'])
        
        angle_in_deg = max(min(car_quaternion.degrees/58.0 *180.0, 180.0), 0.0)
        if car_quaternion.axis[2]<0:
            angle_in_deg = 360.0 - angle_in_deg

        angle_in_rad = angle_in_deg * math.pi / 180.0

        # self.checkpoints will be sorted in ascending order of distance from vehicle
        for cp in self.checkpoints[:self.local_checkpoint_radius]:
            p = np.array( [cp['x'] - self.car_x, cp['y'] - self.car_y, 0] )
            # CP center

            r_p = complex(p[0], p[1]) * complex(math.cos(angle_in_rad), -math.sin(angle_in_rad))

            p[0] = r_p.real
            p[1] = r_p.imag

            self.local_checkpoints['x'].append(p[0])
            self.local_checkpoints['y'].append(p[1])
            self.local_checkpoints['z'].append(p[2])

            p = np.array( [cp['c1']['x'] - self.car_x, cp['c1']['y'] - self.car_y, 0] )
            # CP Left
            
            r_p = complex(p[0], p[1]) * complex(math.cos(angle_in_rad), -math.sin(angle_in_rad))

            p[0] = r_p.real
            p[1] = r_p.imag

            p = np.array( [cp['c2']['x'] - self.car_x, cp['c2']['y'] - self.car_y, 0] )
            
            # CP Right
            r_p = complex(p[0], p[1]) * complex(math.cos(angle_in_rad), -math.sin(angle_in_rad))

            p[0] = r_p.real
            p[1] = r_p.imag

    def referee_state_listener(self):
        cones_hit = 0
        if self.doo_count != self.referee_state.doo_counter:
            delta = self.referee_state.doo_counter - self.doo_count
            cones_hit = delta

            for d in range(self.doo_count + 1, self.doo_count + delta + 1):
                print('Cone hit. {} cone(s) DOO.'.format(d))
                    
            self.doo_count = self.referee_state.doo_counter

        if len(self.lap_times) != len(self.referee_state.laps):
            self.lap_times = self.referee_state.laps
            lap_count = len(self.lap_times)
            lap_time = self.lap_times[-1]
            print('Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')
        
        return cones_hit

    def sort_cones_gt(self): 
        self.time_start("sort_cones_gt", "start")
        self.car_x = self.vehicle_pose['position']['x_val']*100 + self.referee_state.initial_position.x # self.car_state.kinematics_estimated.position.x_val + 
        self.car_y = -self.vehicle_pose['position']['y_val']*100 + self.referee_state.initial_position.y

        for color in self.cones_gt:
            self.cones_gt[color].sort(key=lambda p: distance(p['x'], p['y'], self.car_x, self.car_y))
        
        self.checkpoints.sort(key=lambda p: distance(p['x'], p['y'], self.car_x, self.car_y))
        for i in range(len(self.checkpoints)):
            self.checkpoints[i]['index'] = i
        self.time_start("sort_cones_gt", "finish")

    def compute_track_boundaries(self):
        """
        <RefereeState> {
            'cones': [
                {
                    'color': 0,
                    'x': 5519.39501953125,
                    'y': 8775.1943359375
                }, ... 
            ],
            'doo_counter': 3,
            'initial_position': <Point2D> { 'x': 4575.15283203125, 'y': 8577.8154296875 },
            'laps': [ 4.393726348876953 ]
        }
        """
        self.cones_gt = {}
        for cone in self.referee_state.cones: # ["cones"]:
            # cone['color'].setdefault(3)
            self.cones_gt.setdefault(cone["color"], [])
            self.cones_gt[cone['color']].append({
                'x':cone["x"],
                'y':cone["y"]
            })
        
        self.checkpoints = []
        for c1 in self.cones_gt[0]:
            #print(self.cones_gt[1])
            cp, d = self.cones_gt[1][0], distance(self.cones_gt[1][0]['x'], self.cones_gt[1][0]['y'], c1['x'], c1['y'])
            for c2 in self.cones_gt[1]:
                d2 = distance(c2['x'], c2['y'], c1['x'], c1['y'])
                if d2 < d:
                    cp, d = c2, d2
            ind = len(self.checkpoints)
            self.checkpoints.append({
                'x': (cp['x'] + c1['x'])/2,
                'y': (cp['y'] + c1['y'])/2,
                'c1': c1,
                'c2': cp,
                'index': ind,
                'visited': False
            })

    def get_images(self):
        """
            480x270 : 300ms
        """

        #self.client.client.call("simPause", True)
        self.time_start("get_images", "start")
        z = np.zeros((10,10))
        
        responses = self.client.simGetImages([
            fsds.ImageRequest("cam1", fsds.ImageType.Scene, pixels_as_float=False, compress=False),
            fsds.ImageRequest("cam2", fsds.ImageType.Scene, pixels_as_float=False, compress=False)
            #fsds.ImageRequest("cam3", fsds.ImageType.DepthPlanner, pixels_as_float=True, compress=False)
        ])
        
        response = responses[0]
        # get np array
        #img1d = fsds.list_to_2d_float_array(response.image_data_float, response.width, response.height)
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
        # reshape array to 4 channel image array H X W X 4
        imgL = img1d.reshape(response.height, response.width, 3)

        response = responses[1]
        # get np array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
        # reshape array to 4 channel image array H X W X 4
        imgR = img1d.reshape(response.height, response.width, 3)
        
        imgD = z
        self.time_start("get_images", "finish")
        #self.client.client.call("simPause", False)
        return imgL, imgR, imgD

    def render_nearby_checkpoints(self):
        global old_points

        if old_points == self.nearby_checkpoints: return
        
        old_points = self.nearby_checkpoints
        #self.log("render_nearby_checkpoints" + str(self.nearby_checkpoints))
        #self.log("car_pos" + str((self.car_x, self.car_y)))

        points = [ self.nearby_checkpoints[1]['c1'], self.nearby_checkpoints[2]['c1'], self.nearby_checkpoints[2]['c2'], self.nearby_checkpoints[1]['c2'], self.nearby_checkpoints[0]['c2'], self.nearby_checkpoints[0]['c1'], self.nearby_checkpoints[1]['c1'], self.nearby_checkpoints[1]['c2']  ]
        #points = list(map(lambda p: Vector3r(p['x'], p['y'], 1), points))
        points = list(map(lambda p: Vector3r(p['x']/100, p['y']/100, 1), points))

    def simContinueForTime(self, seconds):
        """
        Continue the simulation for the specified number of seconds
        Args:
            seconds (float): Time to run the simulation for
        """
        self.client.client.call('simContinueForTime', seconds)
    
    def simPause(self, is_paused):
        """
        Pauses simulation
        Args:
            is_paused (bool): True to pause the simulation, False to release
        """
        self.client.client.call('simPause', is_paused)

    def simIsPause(self):
        """
        Returns true if the simulation is paused
        Returns:
            bool: If the simulation is paused
        """
        return self.client.client.call("simIsPaused")

    def step(self, steering=0, throttle=0, brake=0):
        self.time_start("step", "start")
        if not MANUAL_DRIVING:
            #self.simContinueForTime(0.3)
            pass

        
        self.getAllSimData()
        
        car_controls = fsds.CarControls()

        #target_speed = 10

        #car_controls.steering = self.calculate_steering()
        #car_controls.throttle = self.calculate_throttle(target_speed, car_controls.steering)
        #car_controls.brake = self.calculate_brake(target_speed, car_controls.steering)

        car_controls.steering = steering
        car_controls.throttle = throttle
        car_controls.brake = brake

        #car_controls.brake = 0

        #self.log(str(car_controls.steering) + " " + str(car_controls.throttle) + " " + str(car_controls.brake))
        
        self.client.setCarControls(car_controls)

        self.render_nearby_checkpoints()

        #self.client.client.call("simPause", True)
        self.time_start("step", "finish")
    

    def find_cones(self):
        # no points
        if len(self.lidar_data.point_cloud) < 3:
            return [], []

        # Convert the list of floats into a list of xyz coordinates
        points = np.array(self.lidar_data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0]/3), 3))

        lidar_pc = points

        # Go through all the points and find nearby groups of points that are close together as those will probably be cones.

        p_x, p_y = [], []
        for p in points:
            p_x.append(p[0])
            p_y.append(p[1])

        knn_model = NearestNeighbors(radius=50)

        #knn_model.fit(p_x, p_y)
        knn_model.fit(points)

        cones = knn_model.radius_neighbors_graph([[0, 0, 0], ])

        # TODO : KNN for cone detections
        #print(cones)

        cone_groups = []
        current_group = []
        cones = []
        for i in range(1, len(points)):

            # Get the distance from current to previous point
            distance_to_last_point = distance(points[i][0], points[i][1], points[i-1][0], points[i-1][1])

            if distance_to_last_point < 0.1:
                # Points closer together then 10 cm are part of the same group
                current_group.append({'x': points[i][0], 'y': points[i][1]})
            else:
                # points further away indiate a split between groups
                if len(current_group) > 0:
                    cone = pointgroup_to_cone(current_group)
                    # calculate distance between lidar and cone
                    #self.log("len(grp) = " + str(len(current_group)))
                    #self.log("grp = " + str(current_group))
                    if distance(0, 0, cone['x'], cone['y']) < cones_range_cutoff and len(current_group) > 3:
                        cones.append(cone)
                    cone_groups.append(current_group)
                    current_group = []
        return cones, lidar_pc

    def getAllSimData(self):
        """
            Gets all data from sim in one call
        """
        self.time_start("getAllSimData", "start")
        self.car_state = self.client.getCarState()
        self.kinematics = self.client.simGetGroundTruthKinematics('FSCar')
        self.gps_data = self.client.getGpsData(gps_name="Gps")
        self.lidar_data = self.client.getLidarData(lidar_name = 'Lidar')
        self.referee_state =  self.client.getRefereeState()
        self.vehicle_pose = self.client.client.call('simGetVehiclePose', 'FSCar')
        self.collisions = self.client.client.call("simGetCollisionInfo", 'FSCar')
        self.cones_hit = self.referee_state_listener()
        self.sort_cones_gt()
        self.get_local_checkpoints()

        # Images disabled as they take time
        if False: 
            self.images = self.get_images()
        
        self.cones, self.lidar_pc = self.find_cones()
        self.nearby_checkpoints = self.checkpoints[:3]

        self.time_start("getAllSimData", "finish")
    
    def time_start(self, fn, point):
        self.timers.setdefault(fn, {'start':0, 'finish':0})
        self.timers[fn][point] = time.time()
    
    def time_print(self):
        for fn in self.timers:
            self.log(str(fn) + " " + str(self.timers[fn]['finish'] - self.timers[fn]['start']) )
