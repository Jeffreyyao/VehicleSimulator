import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import RungeKuttaSolver, Models

class Object:
    def __init__(self, position, size, object_type, shape):
        self.position = position
        self.size = size
        self.object_type = object_type
        self.shape = shape
        self.x_lower = self.position[0]-self.size[0]/2
        self.x_upper = self.position[0]+self.size[0]/2
        self.y_lower = self.position[1]-self.size[1]/2
        self.y_upper = self.position[1]+self.size[1]/2

    def __contains__(self, point):
        if point[0]>=self.x_lower and point[0]<=self.x_upper:
            if point[1]>=self.y_lower and point[1]<=self.y_upper:
                return True
        return False
    
    def to_matplotlib_patch(self):
        if self.shape=="Rectangle":
            if self.object_type=="Target":
                return Rectangle((self.x_lower,self.y_lower),self.size[0],self.size[1],facecolor="green")
            elif self.object_type=="Obstacle":
                return Rectangle((self.x_lower,self.y_lower),self.size[0],self.size[1],facecolor="red")

class Node:
    def __init__(self, state, parent=None, u_from_parent=None, t_from_parent=None, path_from_parent=None):
        self.state = state
        self.parent = parent # Parent node
        self.u_from_parent = u_from_parent
        self.t_from_parent = t_from_parent
        self.path_from_parent = path_from_parent
        self.next = []

class RRTPlanner:
    def __init__(self, size, target, obstacles, dynamics, start_state, max_steer, max_velocity, tau):
        self.size = size
        self.target = target
        self.obstacles = obstacles
        self.dynamics = dynamics
        self.max_velocity = max_velocity
        self.max_steer = max_steer
        self.tau = tau # time step between integration
        self.k = 500 # number of nodes to explore

        self.rk_solver = RungeKuttaSolver.RungeKuttaSolver(dynamics,len(start_state))
        self.time_range = [0.75,1.25]
        self.node_list = [Node(np.array(start_state))]

    def point_is_valid(self, point):
        y,x = point
        if y<0 or y>self.size[0] or x<0 or x>self.size[1]: return False
        for obs in self.obstacles:
            if point in obs: return False
        return True
    
    def get_random_valid_point(self):
        res = None
        while res is None:
            point = np.random.rand(2)*self.size
            if self.point_is_valid(point): res = point
        return res
    
    def get_nearest_node(self, point_new)->Node:
        return min(self.node_list,key=lambda node:np.linalg.norm(point_new-node.state[:2]))

    def steer(self, state, u, t):
        path = []
        state = list(state) # RK4 must take in list objects
        u = list(u)
        for _ in np.arange(0,t,self.tau):
            path.append(state.copy())
            state = self.rk_solver.RK4(state,u,self.tau)
            if not self.point_is_valid(state[:2]): return None
        return path

    def rrt(self):
        for i in range(self.k):
            point_rand = self.get_random_valid_point()
            if np.random.rand()<0.05: # steer towards target occasionally
                point_rand = np.array(self.target.position)
            node_nearest = self.get_nearest_node(point_rand)
            u_rand = [np.random.uniform(-self.max_steer, self.max_steer), np.random.choice([self.max_velocity])]
            t_rand = np.random.uniform(self.time_range[0], self.time_range[1])
            path = self.steer(node_nearest.state, u_rand, t_rand)
            if path != None:
                state_new = path[-1]
                node_new = Node(state_new.copy(), node_nearest, u_rand, t_rand, path)
                self.node_list.append(node_new)
                node_nearest.next.append(node_new)
                if state_new[:2] in self.target:
                    print("target reached")
                    return node_new
    
    def render_map(self, target_node=None):
        plt.xlim(0,self.size[0])
        plt.ylim(0,self.size[1])
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().add_patch(self.target.to_matplotlib_patch())
        for obs in self.obstacles:
            plt.gca().add_patch(obs.to_matplotlib_patch())
        for node in self.node_list:
            if node.path_from_parent is not None:
                for point in node.path_from_parent:
                    plt.plot(point[0],point[1],"o",color="#666666",markersize=1)
            plt.plot(node.state[0],node.state[1],"o",color="#0000EE",markersize=3)
        while target_node is not None: # plot path from start to target
            if target_node.path_from_parent is not None:
                for point in target_node.path_from_parent:
                    plt.plot(point[0],point[1],"o",color="#000000",markersize=2)
            target_node = target_node.parent
        plt.show()

if __name__ == "__main__":
    deepracer = Models.DeepRacerModel()
    target = Object([3.5,3.5],[0.5,0.5],"Target","Rectangle")
    obstacles = [Object([1.5,1.25],[3,0.5],"Obstacle","Rectangle"),
                Object([2.5,2.5],[3,0.5],"Obstacle","Rectangle")]
    start_state = [0.5,0.5,0,0]
    rp = RRTPlanner([4,4],target,obstacles,deepracer.dynamics,start_state,deepracer.max_u[0],deepracer.max_u[1],0.01)
    target_node = rp.rrt()
    if True: rp.render_map(target_node)