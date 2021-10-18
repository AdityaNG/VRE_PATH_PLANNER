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

from sklearn.neighbors import NearestNeighbors
from scipy.interpolate import lagrange

sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/python")))
import fsds
sys.path.insert(0, os.path.abspath(os.path.join(os.getenv("HOME"), "Formula-Student-Driverless-Simulator/AirSim/PythonClient")))
from airsim.types import Vector3r
from simple_pid import PID
from pyquaternion import Quaternion

MANUAL_DRIVING = False
CHECKPOINT_DISTANCE_THRESH = 0.1 # cm
#sys.path.insert(0, os.path.abspath("stereo_vision/python/"))
max_throttle = 0.35
max_braking = 0.1
target_speed = 6 # m/s
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
    def __init__(self, client, use_pseudo_lidar=False):
        self.client = client
        self.use_pseudo_lidar = use_pseudo_lidar
        #self.steering_pid = PID(0.75, 0.1, 1, setpoint=0)
        self.steering_pid = PID(1, 0, 0.75, setpoint=0)
        self.last_pid_reset = time.time()
        #self.steering_pid.output_limits = (-1.0, 1.0)
        self.throttle_pid = PID(1, 0.1, 0.05, setpoint=4)
        self.throttle_pid.output_limits = (0.0, 1.0)


        # The list of log-lines shown in the operator web gui
        self.logs = []

        # The mission that the team should use
        self.mission = None

        # The unreal engine map that is loaded when starting the simulator.
        #self.track = None

        # The process descriptor of the simulator
        self.simulation_process = None

        # For every simulation launch, a new logfile is created and referee state (like lap times and DOO cones ) are stored.
        # This is the file object for the current run.
        self.log_file = None

        # Wether or not competition mode is enabled or disabled (boolean)
        #self.competition_mode = None 

        # Wether or not the finished signal is received
        self.finished_signal_received = False

        # The rpc client connected to the simulation. Used to retrieve referee state.
        #self.client = None
        
        # The timer that triggers periodic referee state updates.
        self.referee_state_timer = None

        self.reset_message = ""

        self.timers = {}

        self.fitted_curve = {'x':[], 'y':[]}

        self.getAllSimData()
        self.compute_track_boundaries()
        pass

    def finished(self):
        if self.finished_signal_received:
            return {}
        self.log("Received finished signal from autonomous system.")
        self.finished_signal_received = True
        return {}

    def referee_state_listener(self):
        cones_hit = 0
        if self.doo_count != self.referee_state.doo_counter:
            delta = self.referee_state.doo_counter - self.doo_count
            cones_hit = delta

            for d in range(self.doo_count + 1, self.doo_count + delta + 1):
                self.log('Cone hit. {} cone(s) DOO.'.format(d))
                    
            self.doo_count = self.referee_state.doo_counter

        if len(self.lap_times) != len(self.referee_state.laps):
            self.log('Lap ' + str(lap_count) + ' completed. Lap time: ' + str(round(lap_time, 3)) + ' s.')
        
        return cones_hit

    def reset(self, message):
        print("Sim Reset : ", message, " : ", self.reset_message)
        # self.client
        self.reset_message = ""
        self.exit_simulator()
        time.sleep(3)
        self.launch_simulator()
        return

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
        

    def render(self):
        x = self.vehicle_pose['position']['x_val']*100 + self.referee_state.initial_position.x # self.car_state.kinematics_estimated.position.x_val + 
        y = -self.vehicle_pose['position']['y_val']*100 + self.referee_state.initial_position.y # self.car_state.kinematics_estimated.position.y_val + 

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
        
        """
        response = responses[2]
        
        img1d = np.uint8(response.image_data_float)
        imgD = img1d.reshape(response.height, response.width, 1)
        """
        #print(imgL.shape)
        #psl = self.stereo_vision.generatePointCloud(imgL, imgR)
        #psl = self.stereo_vision.generatePointCloud(imgR, imgL)
        #print(psl)
        
        #psl = list(filter(lambda i: abs(i[0])!=math.inf and abs(i[1])!=math.inf and abs(i[2])!=math.inf, psl))
        #psl = list(filter(lambda i: not math.isnan(i[0]) and not math.isnan(i[1]) and not math.isnan(i[2]), psl))

        # (x, y, z) -> (z, -x, -y)
        #psl = list(map(lambda i: (i[2], -i[0], -i[1]), psl))
        
        #psl = list(filter(lambda i: abs(i[2])<=2, psl))
        #psl = list(filter(lambda i: abs(i[0])<=30 and abs(i[1])<=30 and 0.13<=abs(i[2])<=0.50, psl))
        
        #self.pseudo_lidar = psl
        #self.log("pseudo_lidar len = " + str(len(self.pseudo_lidar)))
        imgD = z
        self.time_start("get_images", "finish")
        #self.client.client.call("simPause", False)
        return imgL, imgR, imgD

    def render_nearby_checkpoints(self):
        global old_points

        if old_points == self.nearby_checkpoints:
            return
        
        old_points = self.nearby_checkpoints
        #self.log("render_nearby_checkpoints" + str(self.nearby_checkpoints))
        #self.log("car_pos" + str((self.car_x, self.car_y)))

        points = [ self.nearby_checkpoints[1]['c1'], self.nearby_checkpoints[2]['c1'], self.nearby_checkpoints[2]['c2'], self.nearby_checkpoints[1]['c2'], self.nearby_checkpoints[0]['c2'], self.nearby_checkpoints[0]['c1'], self.nearby_checkpoints[1]['c1'], self.nearby_checkpoints[1]['c2']  ]
        #points = list(map(lambda p: Vector3r(p['x'], p['y'], 1), points))
        points = list(map(lambda p: Vector3r(p['x']/100, p['y']/100, 1), points))
        color_rgba=[1.0, 0.0, 0.0, 1.0]
        thickness = 5.0
        duration = 1.0
        is_persistent = False
        #self.client.client.call('simPlotLineStrip', points, color_rgba, thickness, duration, is_persistent)
        #self.client.client.call('simPlotLineList', points, color_rgba, thickness, duration, is_persistent)
        #self.client.client.call('simPlotLineList', points, color_rgba, thickness, duration, is_persistent)
        
    def calculate_steering(self, max_steering=1.0):
        # TODO : COmpute steering
        self.sort_cones_gt()
        self.nearby_checkpoints = self.checkpoints[:3]

        if self.vehicle_pose['orientation']['x_val']>0 or self.vehicle_pose['orientation']['y_val']>0 or self.vehicle_pose['orientation']['z_val']>0:
            car_quaternion = Quaternion(axis=[self.vehicle_pose['orientation']['x_val'], self.vehicle_pose['orientation']['y_val'], self.vehicle_pose['orientation']['z_val']],
                                    angle=self.vehicle_pose['orientation']['w_val'])
        else:
            car_quaternion = Quaternion(axis=[0, 0, 1], angle=self.vehicle_pose['orientation']['w_val'])
        #car_quaternion = car_quaternion.normalised
        #car_quaternion = car_quaternion.inverse

        angle_in_deg = max(min(car_quaternion.degrees/58.0 *180.0, 180.0), 0.0)
        if car_quaternion.axis[2]<0:
            angle_in_deg = 360.0 - angle_in_deg

        angle_in_rad = angle_in_deg * math.pi / 180.0

        steering = 0
        cp1, cp2, cp3 = self.nearby_checkpoints
        
        self.local_checkpoints = {'x':[], 'y':[], 'z':[]}

        for cp in self.checkpoints[:6]:
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

            #self.local_checkpoints['x'].append(p[0])
            #self.local_checkpoints['y'].append(p[1])
            #self.local_checkpoints['z'].append(p[2])

            p = np.array( [cp['c2']['x'] - self.car_x, cp['c2']['y'] - self.car_y, 0] )
            # CP Right

            r_p = complex(p[0], p[1]) * complex(math.cos(angle_in_rad), -math.sin(angle_in_rad))

            p[0] = r_p.real
            p[1] = r_p.imag

            #self.local_checkpoints['x'].append(p[0])
            #self.local_checkpoints['y'].append(p[1])
            #self.local_checkpoints['z'].append(p[2])
        
        #z = np.polyfit(self.local_checkpoints['x'], self.local_checkpoints['y'], 2)
        #p = np.poly1d(z)
        p = lagrange(self.local_checkpoints['x'], self.local_checkpoints['y'])

        dp = np.polyder(p)
        d2p = np.polyder(dp)

        curvature = 0.0

        for i in range(-100, 2500):
            curvature += d2p(i) / (1 + (dp(i))**2)**1.5

        curvature = curvature * 0.5

        num_points = 1000
        search_radius = 3000 # 5 Meter radius
        
        self.fitted_curve['x'] = np.linspace(-search_radius, search_radius, num_points) 
        self.fitted_curve['y'] = p(self.fitted_curve['x'])

        #min_dist = distance(self.car_x, self.car_y, self.fitted_curve['x'][0], self.fitted_curve['y'][0])
        min_dist = distance(0, 0, self.fitted_curve['x'][0], self.fitted_curve['y'][0])
        for i in range(num_points):
            d = distance(0, 0, self.fitted_curve['x'][i], self.fitted_curve['y'][i])
            if d < min_dist:
                min_dist = d

        dist = min_dist

        intersects, line_dist, point, side_of_line = circle_intersects_with_line(0, 0, CHECKPOINT_DISTANCE_THRESH, self.local_checkpoints['x'][0], self.local_checkpoints['y'][0], self.local_checkpoints['x'][1], self.local_checkpoints['y'][1])

        if side_of_line < 0:
            dist = -dist

        now = time.time()
        if now - self.last_pid_reset > 1:
            #print("PID Reset")
            #self.steering_pid = PID(2, 0, 0.5, setpoint=0)
            #self.last_pid_reset = now
            pass

        steering = self.steering_pid(dist)
        #if side_of_line < 0:
        #    steering = -steering

        final_steering = steering / 140
        if final_steering>1:
            final_steering = 1
        elif final_steering<-1:
            final_steering = -1

        #print("car_quaternion=", car_quaternion, ", angle_in_deg=", angle_in_deg, ", car_quaternion.axis=", car_quaternion.axis, sep='\t')
        #print("steering=", steering, ", dist=", dist, ", cp_q=", (self.local_checkpoints['x'][0], self.local_checkpoints['y'][0], self.local_checkpoints['z'][0]), "final_steering=", final_steering, sep='\t')

        #self.log("steering=" + str(steering) + ", dist=" + str(dist))

        return final_steering

    def calculate_throttle(self, target_speed, steering_angle):
        speed_limit = target_speed
        gps = self.gps_data

        # Calculate the velocity in the vehicle's frame
        speed = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))

        # the lower the velocity, the more throttle, up to max_throttle
        return max_throttle * max(1.0 - steering_angle**2 - (speed/speed_limit)**2, 0.0)

    def calculate_brake(self, target_speed, steering_angle):
        speed_limit = target_speed
        gps = self.gps_data

        # Calculate the velocity in the vehicle's frame
        speed = math.sqrt(math.pow(gps.gnss.velocity.x_val, 2) + math.pow(gps.gnss.velocity.y_val, 2))

        # the lower the velocity, the more throttle, up to max_throttle
        return -max_braking * min(1.0 - (steering_angle*1.0)**2 - (speed/speed_limit)**2, 0.0)

    def compute_reward(self):
        reward = 0
        done = False

        now = time.time()

        
        top_speed = 1
        top_speed_reward = 10

        
        if self.car_state.speed == 0: 
            if now - self.last_speed_zero > 3: # Stationary for too long
                self.log("[SPEED_PENALTY] " + str(self.last_speed_zero))
                reward-=5
                self.last_speed_zero = now
            
        
        self.car_x = self.vehicle_pose['position']['x_val']*100 + self.referee_state.initial_position.x # self.car_state.kinematics_estimated.position.x_val + 
        self.car_y = -self.vehicle_pose['position']['y_val']*100 + self.referee_state.initial_position.y

        self.sort_cones_gt()    # sorts self.checkpoints and self.cones_gt by distance to self.car_x, self.car_y

        closest_cp, d, index = self.checkpoints[0], distance(self.checkpoints[0]['x'], self.checkpoints[0]['y'], self.car_x, self.car_y), 0
        for i in range(len(self.checkpoints)):
            c = self.checkpoints[i]
            d2 = distance(c['x'], c['y'], self.car_x, self.car_y)
            if d2<d and not c['visited']:
                closest_cp, d, index = c, d2, i

        #if d<CHECKPOINT_DISTANCE_THRESH and not self.checkpoints[index]['visited']:
        #    reward += 50
        #    self.checkpoints[index]['visited'] = True
        #    self.log("[CHECKPOINT] " + str(closest_cp))

        self.nearby_checkpoints = self.checkpoints[:3]

        for i in range(len(self.nearby_checkpoints)):
            #cp = self.nearby_checkpoints[i]
            cp = self.checkpoints[self.nearby_checkpoints[i]['index']]
            if circle_intersects_with_line(self.car_x, self.car_y, CHECKPOINT_DISTANCE_THRESH, cp['c1']['x'], cp['c1']['y'], cp['c2']['x'], cp['c2']['y']) and not self.checkpoints[cp['index']]['visited']:
                self.log("[CHECKPOINT] " + str(cp['visited']))
                reward+=50
                self.checkpoints[cp['index']]['visited'] = True
                self.log("[CHECKPOINT] " + str(cp))
            
            j = (i+1)%len(self.nearby_checkpoints)
            cp2 = self.nearby_checkpoints[j]
            intersects_border_1 = circle_intersects_with_line(self.car_x, self.car_y, CHECKPOINT_DISTANCE_THRESH, cp['c1']['x'], cp['c1']['y'], cp2['c1']['x'], cp2['c1']['y'])
            intersects_border_2 = circle_intersects_with_line(self.car_x, self.car_y, CHECKPOINT_DISTANCE_THRESH, cp['c2']['x'], cp['c2']['y'], cp2['c2']['x'], cp2['c2']['y'])
            if intersects_border_1 or intersects_border_2:
                #reward-=50
                #self.log("[BORDER] " + str(cp))
                pass

        inside_count = 0
        outside_count = 0
        for combo in itertools.permutations(list(range(len(self.nearby_checkpoints)))):
            outer_polygon = []
            for i in range(0, len(combo), +1):
                j = combo[i]
                outer_polygon.append(self.nearby_checkpoints[j]['c1'])
            
            for i in range(len(combo)-1, -1, -1):
                j = combo[i]
                outer_polygon.append(self.nearby_checkpoints[j]['c2'])

            #outer_polygon = [ self.nearby_checkpoints[0]['c1'], self.nearby_checkpoints[1]['c1'], self.nearby_checkpoints[2]['c1'], self.nearby_checkpoints[2]['c2'], self.nearby_checkpoints[1]['c2'], self.nearby_checkpoints[0]['c2'] ]
            # outer_polygon = [ self.nearby_checkpoints[1]['c1'], self.nearby_checkpoints[0]['c1'], self.nearby_checkpoints[2]['c1'], self.nearby_checkpoints[2]['c2'], self.nearby_checkpoints[0]['c2'], self.nearby_checkpoints[1]['c2'] ]
            result = point_is_inside_polygon(self.car_x, self.car_y, outer_polygon)
            if result != "INVALID":
                if result: # Inside 
                    inside_count += 1
                else:
                    outside_count +=1

        #reward = reward - self.previous_reward

        
        #self.log("inside_count " + str(inside_count))
        #self.log("outside_count " + str(outside_count))
        if inside_count>outside_count:
            if self.car_state.speed > 0:
                #self.log("Inside track")
                #reward+=1
                pass
        else:
            #self.log("[EXIT] OUTSIDE TRACK")
            #reward -= 5
            #reward = - 100
            #done = True
            #return reward, done
            pass

        
        cones_hit = self.referee_state_listener()
        if cones_hit>0:
            reward= -100
            done = True
            self.reset_message += " Cone hit "
            self.log("[EXIT] CONE HIT")
            return reward, done
        
        """
        num_cones = len(self.cones)
        if num_cones < 2:
            self.num_cones_warning_history += 1
            reward-= 1
            #done = True
            self.log("[WRN] <2 CONES VISIBLE")
            if self.num_cones_warning_history > 5:
                self.log("[EXIT] <2 CONES VISIBLE")
                self.reset_message += " Less than 2 cones visible "
        else:
            self.num_cones_warning_history = 0
        """

        return reward, done

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

    def step(self):
        self.time_start("step", "start")
        if not MANUAL_DRIVING:
            #self.simContinueForTime(0.3)
            pass

        self.getAllSimData()
        """
        #self.simPause(True)
        network_thread = threading.Thread(target=self.getAllSimData_async)
        network_thread.start()
        network_thread.join()
        #self.simPause(False)
        # """
        
        #reward, done = self.compute_reward()
        reward, done = 0, False
        info = ""
        
        
        
        car_controls = fsds.CarControls()

        #target_speed = 10

        car_controls.steering = self.calculate_steering()
        car_controls.throttle = self.calculate_throttle(target_speed, car_controls.steering)
        car_controls.brake = self.calculate_brake(target_speed, car_controls.steering)
        #car_controls.brake = 0

        #self.log(str(car_controls.steering) + " " + str(car_controls.throttle) + " " + str(car_controls.brake))
        
        self.client.setCarControls(car_controls)

        self.render_nearby_checkpoints()

        #network_thread.join()

        #self.car_state, self.kinematics, self.gps_data, self.lidar_data, self.referee_state, self.images, self.cones, self.lidar_pc = self.network_data

        if self.use_pseudo_lidar:
            self.lidar_pc = self.pseudo_lidar

        #self.client.client.call("simPause", True)
        self.time_start("step", "finish")
        #self.time_print()
        if reward!=0:
            self.log("reward : " + str(reward))

        self.previous_reward = reward

        #done = False
        return reward, done, info
    

    def find_cones(self):
        
        if self.use_pseudo_lidar:
            #return [], self.pseudo_lidar
            lidar_pc = self.pseudo_lidar
            num_cones = 500
            if len(self.pseudo_lidar)>num_cones:
                points = random.sample(self.pseudo_lidar, num_cones)
            else:
                points = self.pseudo_lidar

            def angle_of(x, y):
                if x == 0:
                    x = 0.00001
                theta = math.atan(abs(y)/abs(x))
                if x<0 and y>0: # Second quadrant
                    theta += math.pi/2
                elif x<0 and y<0: # Third quadrant
                    theta += math.pi
                elif x>0 and y<0: # Fourth quadrant
                    theta += math.pi + math.pi/2
                return theta

            points = sorted(points, key=lambda i: angle_of(i[0], i[1]))
        else:
            # no points
            if len(self.lidar_data.point_cloud) < 3:
                return [], []

            # Convert the list of floats into a list of xyz coordinates
            points = np.array(self.lidar_data.point_cloud, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))

            lidar_pc = points
            #return [], points

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
        #self.log("cone_groups = " + str(cone_groups))
        #self.log("len(cones) = " + str(len(cones)))
        return cones, lidar_pc

    def getAllSimData_async(self):
        """
            Gets all data from sim in one call
        """
        self.time_start("getAllSimData", "start")
        cones, lidar_pc = self.find_cones()
        state = self.compute_state(lidar_pc)
        
        self.vehicle_pose = self.client.client.call('simGetVehiclePose', 'FSCar')
        
        self.network_data = self.client.getCarState(), self.client.simGetGroundTruthKinematics(), self.client.getGpsData(gps_name="Gps"), self.client.getLidarData(lidar_name = 'Lidar'), self.client.getRefereeState(), self.get_images(), cones, lidar_pc, state

        #self.network_data.append(cones)
        #self.network_data.append(lidar_pc)
        #self.network_data.append(state)

        #self.collisions = self.client.client.call("simGetCollisionInfo", 'FSCar') # may not be useful
        self.time_start("getAllSimData", "finish")
        pass

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
        
        if False:
            self.images = self.get_images()
        
        self.cones, self.lidar_pc = self.find_cones()

        self.time_start("getAllSimData", "finish")
        #self.collisions = self.client.client.call("simGetCollisionInfo", 'FSCar') # may not be useful
        pass
    
    def time_start(self, fn, point):
        self.timers.setdefault(fn, {'start':0, 'finish':0})
        self.timers[fn][point] = time.time()
    
    def time_print(self):
        for fn in self.timers:
            self.log(str(fn) + " " + str(self.timers[fn]['finish'] - self.timers[fn]['start']) )


def pointgroup_to_cone(group): # Numpy could help speed this averaging operation
    average_x = 0
    average_y = 0
    for point in group:
        average_x += point['x']
        average_y += point['y']
    average_x = average_x / len(group)
    average_y = average_y / len(group)
    return {'x': average_x, 'y': average_y}

def distance(x1, y1, x2, y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5 

def circle_intersects_with_line(circle_x, circle_y, circle_r, p1_x, p1_y, p2_x, p2_y): 
    """
        Drop a perpendicular from C((circle_x, circle_y), circle_r) to line L((p1_x, p1_y), (p2_x, p2_y)) as (perp_x, perp_y)
        return True if
            1. If the point on the line is between (p1_x, p1_y), (p2_x, p2_y)
            2. distance( circle_x, circle_y,  perp_x, perp_y) < circle_r
        return False otherwise
    """
    try:
        m = (p2_y-p1_y)/(p2_x-p1_x)
    except ZeroDivisionError:
        m = 5729577.951308174 # tan(89.99999)

    if m == 0:
        m = 0.000000175 # cot(89.99999)

    # Perpenicular line
    A1 = 1.0/m
    B1 = 1.0
    C1 = -(circle_y + (1.0/m)*circle_x)

    # Original line
    A2 = m
    B2 = -1.0
    C2 = -m * p1_x + p1_y  

    side_of_line = A2*circle_x + B2*circle_y + C2

    tmp = A1*B2 - A2*B1 # tmp can never be zero as we have defined the two lines to be perpendicular
    
    perp_x = (B1*C2 - B2*C1) / tmp
    perp_y = (A2*C1 - A1*C2) / tmp

    d = distance(perp_x, perp_y, circle_x, circle_y)

    #if d < circle_r and (perp_y - p1_y)/(perp_x - p1_x) * (p2_y - perp_y)/(p1_x - perp_x) >= 0:
    if d < circle_r and ((perp_y - p1_y)*(p2_y - perp_y) >= 0 and (perp_x - p1_x)*(p1_x - perp_x) >=0):
        return True, d, (perp_x, perp_y), side_of_line

    return False, d, (perp_x, perp_y), side_of_line

"""
Approach 1 : 6 nearest cones : 

When considering the 3 nearby checkpoints, we get 6 cones
The 3 cones on left boundary are :
nearby_checkpoints[0]['c1'], nearby_checkpoints[1]['c1'], nearby_checkpoints[3]['c1'] # Why 0,1,3? why not 0,1,2?

The 3 cones on right boundary is :
nearby_checkpoints[0]['c2'], nearby_checkpoints[1]['c2'], nearby_checkpoints[3]['c2']
            
Where nearby_checkpoints[i] is 
{
    'x': (c2['x'] + c1['x'])/2,     # Midpoint of the 2 cones
    'y': (c2['y'] + c1['y'])/2,     # Midpoint of the 2 cones
    'c1': c1,                       # Left cone
    'c2': c2,                       # Right cone
    'visited': False                # Flag to see if cp is already visited 
}

These 6 cones form a polygon.
The car is inside the track iff:
    1. point_is_inside_polygon(car_x, car_y, [
        nearby_checkpoints[0]['c1'], nearby_checkpoints[1]['c1'], nearby_checkpoints[3]['c1'],
        nearby_checkpoints[0]['c2'], nearby_checkpoints[1]['c2'], nearby_checkpoints[3]['c2']
    ])

Approach 2 : Full Track Analysis : 

The track is two polygons :
    1. p_in  - inner set of cones
    2. p_out - outer set of cones

The car is inside the track iff :
    1. point_is_inside_polygon(car_x, car_y, p_in) == False
    2. point_is_inside_polygon(car_x, car_y, p_out) == True
"""
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
def point_is_inside_polygon(p_x, p_y, polygon_points):
    """
        How to check if a given point lies inside or outside a polygon?
            https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/#:~:text=1)%20Draw%20a%20horizontal%20line,on%20an%20edge%20of%20polygon.

        The order set of polygon_points must be considered such that :
            1. We start with a point p
            2. The next point p' is the closest point to p ; mark p as visited
            3. The third point p'' is the point closest to p', but not marked visited list 
        
        We repeat until all points are marked as visited
        The last line will be between the first point marked as visited and the last point

        polygon_points[i] = {
            'x': float,
            'y': float,
        }

        https://stackoverflow.com/questions/36399381/whats-the-fastest-way-of-checking-if-a-point-is-inside-a-polygon-in-python
    """
    polygon_points = list(map(lambda p: Point(p['x'], p['y']) , polygon_points))

    point = Point(p_x, p_y)
    polygon = Polygon(polygon_points)
    if polygon.is_valid:
        return polygon.contains(point)
    else:
        return "INVALID"


def point_is_inside_polygon_buggy(p_x, p_y, polygon_points_tot):
    """
        How to check if a given point lies inside or outside a polygon?
            https://www.geeksforgeeks.org/how-to-check-if-a-given-point-lies-inside-a-polygon/#:~:text=1)%20Draw%20a%20horizontal%20line,on%20an%20edge%20of%20polygon.

        The order set of polygon_points must be considered such that :
            1. We start with a point p
            2. The next point p' is the closest point to p ; mark p as visited
            3. The third point p'' is the point closest to p', but not marked visited list 
        
        We repeat until all points are marked as visited
        The last line will be between the first point marked as visited and the last point

        polygon_points[i] = {
            'x': float,
            'y': float,
        }

        https://stackoverflow.com/questions/36399381/whats-the-fastest-way-of-checking-if-a-point-is-inside-a-polygon-in-python
    """

    
    for polygon_points in itertools.permutations( polygon_points_tot ): # all_combintations
        #print(polygon_points)
        polygon_points = list(map(lambda p: Point(p['x'], p['y']) , polygon_points))

        """
        polygon_points_sorted = []
        while len(polygon_points)>1:
            closest_point, d, index = polygon_points[0], distance(polygon_points[0][0], polygon_points[0][1], p_x, p_y), 0
            for i in range(len(polygon)):
                pass
        """

        point = Point(p_x, p_y)
        polygon = Polygon(polygon_points)
        if polygon.is_valid:
            #print("=============polygon.is_valid=================")
            res = polygon.contains(point)
            #print(polygon)
            #print((p_x, p_y))
            if res:
                return True
    
    return False