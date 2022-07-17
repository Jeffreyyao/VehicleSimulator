import math

class DeepRacerModel:
    def __init__(self):
        self.dimensions = 4 # x,y,theta,v
        self.max_u = [1,2] # max_steer, max_speed

    def dynamics(self, x, u):
        u_steer = self.map_steering(u[0])
        u_speed = self.map_speed(u[1])
        L = 0.165
        [a,b] = self.get_v_params(u_speed)
        return [x[3]*math.cos(x[2]), x[3]*math.sin(x[2]), (x[3]/L)*math.tan(u_steer), a*x[3]+b*u_speed]

    def get_v_params(self, u_speed):
        if u_speed == 0.0:
            K = 0.0; T = 0.25
        elif u_speed == 0.45:
            K = 1.9953; T = 0.9933
        elif u_speed == 0.50:
            K = 2.3567; T = 0.8943
        elif u_speed == 0.55:
            K = 3.0797; T = 0.88976
        elif u_speed == 0.60:
            K = 3.2019; T = 0.87595
        elif u_speed == 0.65:
            K = 3.3276; T = 0.89594
        elif u_speed == 0.70:
            K = 3.7645; T = 0.92501
        elif u_speed == -0.45:
            K = 1.8229; T = 1.8431
        elif u_speed == -0.50:
            K = 2.3833; T = 1.2721
        elif u_speed == -0.55:
            K = 2.512; T = 1.1403
        elif u_speed == -0.60:
            K = 3.0956; T = 1.12781
        elif u_speed == -0.65:
            K = 3.55; T = 1.1226
        elif u_speed == -0.70:
            K = 3.6423; T = 1.1539
        else:
            raise ValueError('Invalid input !')
        a = -1.0/T
        b = K/T
        return [a,b]
    
    def map_steering(self, angle_in):
        p1 = -0.1167
        p2 = 0.01949
        p3 = 0.3828
        p4 = -0.0293
        x = angle_in
        psi = p1*x**3 + p2*x**2 + p3*x + p4
        return psi
    
    def map_speed(self, speed_in):
        if speed_in == 6:
            s = 0.70
        if speed_in == 5:
            s = 0.65
        if speed_in == 4:
            s = 0.60
        if speed_in == 3:
            s = 0.55
        if speed_in == 2:
            s = 0.50
        if speed_in == 1:
            s = 0.45
        if speed_in == 0:
            s = 0.00
        if speed_in ==-1:
            s = -0.45
        if speed_in ==-2:
            s = -0.50
        if speed_in ==-3:
            s = -0.55
        if speed_in ==-4:
            s = -0.60
        if speed_in ==-5:
            s = -0.65
        if speed_in ==-6:
            s = -0.70
        return s

class BicycleModel:
    def __init__(self, vehicle_length):
        self.vehicle_length = vehicle_length
        self.dimensions = 3 # x,y,theta
        self.max_u = [math.pi/8,1] # max_steer=22.5deg, max_speed=1
    
    def dynamics(self, x, u):
        steer, velocity = u[0], u[1]
        theta = x[2]
        beta = math.atan(math.tan(steer)*0.5)
        xdot = velocity*math.cos(theta+beta)
        ydot = velocity*math.sin(theta+beta)
        thetadot = velocity*math.tan(steer)*math.cos(beta)/self.vehicle_length
        return [xdot, ydot, thetadot]