import matplotlib.pyplot as plt 
import json

def replaceAll(s, sub, rep):
    temp = str.maketrans(sub, rep)
    return s.translate(temp)

def sanetise_ref(s):
    s = s.replace("<RefereeState>", "").replace("<Point2D>", "")
    s = replaceAll(s, "'", '"')
    return s

def getColor(c):
    if c==0:
        return 'blue'
    elif c==1:
        return 'yellow'
    else:
        return 'red'

global ax1, ax2, ax3, ax4, ax5
ax1 = plt.subplot(2,2,1)
ax2 = plt.subplot(2,2,2)
ax3 = plt.subplot(2,2,3)
ax4 = plt.subplot(2,2,4)
#ax5 = ax4.add_subplot(111, projection='3d')
#if SCATTER_PLOT: ax5 = plt.subplot(2,2,2, projection='3d')

plt.ion()
# plt.draw()
plt.show(block=False)

def distance(x1, y1, x2, y2):
    #print(x1, x2)
    return ( (x1-x2)**2 + (y1-y2)**2 )**0.5

class TrackPlot:

    def __init__(self, RefereeState):
        self.rs = RefereeState
        self.car_pos = dict()
        # print(self.rs)

    def render(self, RefereeState, local_checkpoints, fitted_curve, accel_2d):
        self.rs = RefereeState
        self.compute()
        global ax1, ax2, ax3, ax4, ax5
        #ax3.imshow(imgL)
        
        #ax4.imshow(CarState)

        ax1.set_aspect('equal', adjustable='box')
        ax1.cla()

        ax2.set_aspect('equal', adjustable='box')
        ax2.cla()

        ax3.set_aspect('equal', adjustable='box')
        ax3.cla()

        ax3.plot(0, 0, "o", color="g")
        ax3.plot(accel_2d['x'], accel_2d['y'], color="r")
        ax3.set_xlim([-10, 10])
        ax3.set_ylim([-10, 10])

        # plt.clf()
        for c in self.cones:
            ax1.plot(self.cones[c]["x"], self.cones[c]["y"], "o", color=getColor(c))

        self.car_pos.setdefault("x", False)
        self.car_pos.setdefault("y", False)

        if self.car_pos["x"] and self.car_pos["y"]:
            ax1.plot(self.car_pos["x"], self.car_pos["y"], "o", color="r")
            #ax2.plot(self.car_pos["x"], self.car_pos["y"], "o", color="red")

        #local_checkpoints = {'x':[], 'y':[]}
        #for cp in nearby_checkpoints:
        #    local_checkpoints['x'].append(cp['x'])
        #    local_checkpoints['y'].append(cp['y'])

        ax2.plot(0, 0, "o", color="red")
        ax2.set_xlim([-4000, 4000])
        ax2.set_ylim([-4000, 4000])
        
        ax2.plot(local_checkpoints["x"], local_checkpoints["y"], "o", color='blue')
        ax2.plot(fitted_curve["x"], fitted_curve["y"], "-", color='green')

        #plt.draw()
        plt.pause(0.001)
        pass
    
    def compute(self):
        self.cones = dict()
        for cone in self.rs.cones: # ["cones"]:
            # cone['color'].setdefault(3)
            self.cones.setdefault(cone["color"], {"x": [], "y": []})
            self.cones[cone['color']]["x"].append(cone["x"])
            self.cones[cone['color']]["y"].append(cone["y"])
        pass

    def update_car_position(self, x, y, car_cones):
        self.car_pos = {"x": x, "y": y}
        self.car_cones = {"x": [], "y": []}
        for cone in car_cones:
            self.car_cones["y"].append(cone["x"])
            self.car_cones["x"].append(cone["y"])
        pass