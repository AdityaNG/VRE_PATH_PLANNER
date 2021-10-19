# Getting Started

## Installing FSDS

Clone the FSDS repo to your home :

```bash
cd ~/
git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator
```

Download the Formula Student Driverless Simulator from https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/tag/v2.0.0

Save it to `~/Apps/fsds-v2.0.0-linux/`

## Setting up Python env

Use the `./setup_env.sh` script. The source
```bash
source ~/fsds/bin/activate
```

## Running the demo

<img width=400 src=imgs/live_graph.png>

```bash
python main.py
```

# Tasks

## Prerequisites

1. Clean up the code
2. Compute the Frame Rate at which data is coming in. Constrain it to 30Hz. If incoming frame rate > 30 Hz then simply drop a few frames; If incoming frame rate <> 30 Hz then reduce the simulation speed by altering `"ClockSpeed": 1.0` in `settings.json` 
3. Live Graph out the GPS waypoints using a scatter plot 
4. Live Graph out the acceleration vector on a line plot (x_axis: accel_x, y_axis: accel_y)
5. On the above graph, also plot out the path traced by the tip of the acceleration vector over the last X seconds

## Task 1

You are given a set of GPS waypoints(lat, lon), a set of obstacles(lat, lon, radius) and current GPS position(lat, lon). Produce a spline going through the waypoints, avoiding obstacles with a buffer of x meters.

## Task 2

Approximate vehicle GPS position by integrating accelerometer data twice. This is done because GPS will operate at <10Hz and we'll need position feedback closer to >30Hz.

Note that GPS data will give position and speed at 10Hz. Accelerometer will give this at say 30Hz. You can use the GPS speed, velocity to validate the accelerometers predicted speed and velocity.


# Simulator Data

The simulator provides a lot of data. They can be classified into 2 types :
1. Ground Truth Data - this data is truth. We can't aquire this data form a real world setup. So we use this data to validate our driving model, but don't use this data to make any driving decisions
2. Vehicle Data - the data that comes from sensors on the vehicle. This is what we'll be using to make driving decisions

## Ground Truth

```python
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
fsds_car.referee_state

"""
{
    'position': {'x_val': 1.4648437172581907e-05, 'y_val': 1.9531249563442543e-05, 'z_val': 0.24647490680217743}, 
    'orientation': {'w_val': 1.0, 'x_val': 3.7359335692599416e-05, 'y_val': -2.8133388696005568e-05, 'z_val': 1.0510446957212594e-09}
}
"""
fsds_car.vehicle_pose

"""
{
    'has_collided': False, 
    'penetration_depth': 0.0, 
    'time_stamp': 0, 
    'normal': {'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}, 
    'impact_point': {'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}, 
    'position': {'x_val': 0.0, 'y_val': 0.0, 'z_val': 0.0}, 
    'object_name': '', 'object_id': -1
}
"""
fsds_car.collisions

"""
<KinematicsState> {   'angular_acceleration': <Vector3r> {   'x_val': 0.4556628465652466,
    'y_val': -1.072081208229065,
    'z_val': 9.041301727294922},
    'angular_velocity': <Vector3r> {   'x_val': -0.018733734264969826,
    'y_val': 0.015553808771073818,
    'z_val': -0.1384759396314621},
    'linear_acceleration': <Vector3r> {   'x_val': -1.6026678085327148,
    'y_val': -2.5434296131134033,
    'z_val': -0.06134575977921486},
    'linear_velocity': <Vector3r> {   'x_val': 5.451869010925293,
    'y_val': -0.05416398495435715,
    'z_val': -0.014015535824000835},
    'orientation': <Quaternionr> {   'w_val': 0.9999423623085022,
    'x_val': -0.0005744663067162037,
    'y_val': -0.0036147371865808964,
    'z_val': 0.010092003270983696},
    'position': <Vector3r> {   'x_val': 2.2934277057647705,
    'y_val': 0.133544921875,
    'z_val': 0.25240659713745117}
}
"""
fsds_car.kinematics
```

## Vehicle Data

```python
"""
<CarState> {   'gear': 1,
    'handbrake': False,
    'kinematics_estimated': <KinematicsState> {   'angular_acceleration': <Vector3r> {   'x_val': -0.15900176763534546,
    'y_val': -0.42379963397979736,
    'z_val': -1.327945590019226},
    'angular_velocity': <Vector3r> {   'x_val': -0.030406050384044647,
    'y_val': 0.022839326411485672,
    'z_val': -0.024887345731258392},
    'linear_acceleration': <Vector3r> {   'x_val': -1.9401495456695557,
    'y_val': -2.7916672229766846,
    'z_val': -0.10652903467416763},
    'linear_velocity': <Vector3r> {   'x_val': 5.673172950744629,
    'y_val': 0.2383269965648651,
    'z_val': -0.011968697421252728},
    'orientation': <Quaternionr> {   'w_val': 0.9995890855789185,
    'x_val': 0.0005920188268646598,
    'y_val': -0.0036310639698058367,
    'z_val': 0.028425142168998718},
    'position': <Vector3r> {   'x_val': 2.467294931411743,
    'y_val': 0.20791015028953552,
    'z_val': 0.2524517774581909}},
    'maxrpm': 13000.0,
    'rpm': 4234.337890625,
    'speed': 5.661040782928467,
    'timestamp': 1634618931990498000
}
"""
fsds_car.car_state

"""
<GpsData> {   'gnss': <GnssReport> {   'eph': 0.03999999910593033,
    'epv': 0.03999999910593033,
    'geo_point': <GeoPoint> {   'altitude': 122.24626159667969,
    'latitude': 47.64146799936157,
    'longitude': -122.14016500059651},
    'time_utc': 1634619351580,
    'velocity': <Vector3r> {   'x_val': 0.0,
    'y_val': -0.0,
    'z_val': 0.0}},
    'time_stamp': 1634619351580831000
}
"""
fsds_car.gps_data

"""
<LidarData> {   'point_cloud': [   2.406177520751953, -2.1483519077301025,-1.901892687783402e-08, ],
    'pose': <Pose> {   'orientation': <Quaternionr> {   'w_val': 1.0,
    'x_val': 3.7359335692599416e-05,
    'y_val': -2.8133388696005568e-05,
    'z_val': 1.0510446957212594e-09},
    'position': <Vector3r> {   'x_val': 1.3000088930130005,
    'y_val': 1.205938224302372e-05,
    'z_val': 0.34654805064201355}},
    'time_stamp': 1634619612143407000
}
"""
fsds_car.lidar_data
```
