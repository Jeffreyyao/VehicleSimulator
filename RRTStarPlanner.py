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
    def __init__(self, point, parent=None, cost=0):
        self.point = point
        self.parent = parent # Parent node
        self.cost = cost

class RRTStarPlanner:
    def __init__(self, size, target, obstacles, start_point, eta, k):
        self.size = size
        self.target = target
        self.obstacles = obstacles
        self.eta = eta # max step size between nodes
        self.k = k # max number of nodes to explore
        self.node_list = [Node(np.array(start_point))]

        self.d = 2 # dimsension of space
        zeta = math.pi # area of a unit 2d circle
        mu = sum([obs.size[0]*obs.size[1] for obs in obstacles]) # area of all obstacles
        self.gamma = 2*(1+1/self.d)**(1/self.d)*(mu/zeta)**(1/self.d)

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
    
    def dist(self, point1, point2):
        return np.linalg.norm(np.array(point1)-np.array(point2))
    
    def get_nearest_node(self, point_new)->Node:
        return min(self.node_list,key=lambda node:self.dist(point_new,node.point))

    def steer(self, point_start, point_end):
        point_new = None
        cost = None
        n_start = np.array(point_start); n_end = np.array(point_end)
        d = np.linalg.norm(n_start-n_end)
        if d <= self.eta:
            point_new = point_end.tolist()
            cost = d
        else:
            point_new = (n_start+(n_start-n_end)/d*self.eta).tolist()
            cost = self.eta
        return point_new, cost
    
    def collision_free(self, point1, point2):
        for point in np.linspace(point1,point2,10):
            for obs in self.obstacles:
                if point in obs: return False
        return True
        
    def near(self, center):
        n = len(self.node_list)
        r = 1.5*min(self.eta,self.gamma*(math.log2(n)/n)**(1/self.d)) # radius to search nodes nearby
        res = []
        for node in self.node_list:
            if self.dist(node.point,center)<=r: res.append(node)
        return res

    def rrt(self): # basic rrt algorithm
        for i in range(self.k):
            point_rand = self.get_random_valid_point()
            if np.random.rand()<0.05: # steer towards target occasionally
                point_rand = np.array(self.target.position)
            node_nearest = self.get_nearest_node(point_rand)
            point_new,cost = self.steer(node_nearest.point, point_rand)
            if self.collision_free(point_new,node_nearest.point):
                node_new = Node(point_new,node_nearest,cost+node_nearest.cost)
                self.node_list.append(node_new)
                if point_new in self.target: return node_new

    # http://personal.cimat.mx:8181/~miguelvargas/Slides/Optimal%20Rapidly-exploring%20Random%20Trees.pdf
    def rrt_star(self): # optimal rrt* algorithm
        target_node = None
        for i in range(self.k):
            point_rand = self.get_random_valid_point()
            if np.random.rand()<0.05: # steer towards target occasionally
                point_rand = np.array(self.target.position)
            node_nearest = self.get_nearest_node(point_rand)
            point_new,cost = self.steer(node_nearest.point, point_rand)
            if self.collision_free(point_new,node_nearest.point):
                near_list = self.near(point_new)
                near_min = node_nearest
                cost_min = cost+node_nearest.cost
                for near in near_list:
                    if self.collision_free(point_new,near.point):
                        cost_near = self.dist(near.point,point_new)+near.cost
                        if cost_near < cost_min:
                            near_min = near
                            cost_min = cost_near
                node_new = Node(point_new,near_min,cost_min)
                self.node_list.append(node_new)
                if point_new in self.target:
                    if target_node is None: target_node=node_new
                    else: target_node=min(target_node,node_new,key=lambda x:x.cost)
                for near in near_list:
                    if self.collision_free(point_new,near.point):
                        cost_new = node_new.cost+self.dist(point_new,near.point)
                        if cost_new < near.cost:
                            near.parent = node_new
                            near.cost = cost_new
        return target_node
    
    def render_map(self, target_node=None):
        plt.xlim(0,self.size[0])
        plt.ylim(0,self.size[1])
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().add_patch(self.target.to_matplotlib_patch())
        for obs in self.obstacles:
            plt.gca().add_patch(obs.to_matplotlib_patch())
        while target_node is not None: # plot path from start to target
            plt.plot(target_node.point[0],target_node.point[1],"o",color="#0000EE",markersize=5)
            if target_node.parent is not None:
                for point in np.linspace(target_node.point,target_node.parent.point,10):
                    plt.plot(point[0],point[1],"o",color="#000000",markersize=1)
            target_node = target_node.parent
        plt.show()

if __name__ == "__main__":
    deepracer = Models.DeepRacerModel()
    target = Object([3.5,3.5],[0.5,0.5],"Target","Rectangle")
    obstacles = [Object([1.5,1.25],[3,0.5],"Obstacle","Rectangle"),
                Object([2.5,2.5],[3,0.5],"Obstacle","Rectangle")]
    start_state = [0.5,0.5]
    eta, k = 1, 500
    rp = RRTStarPlanner([4,4],target,obstacles,start_state,eta,k)
    #target_node = rp.rrt()
    target_node = rp.rrt_star()
    if target_node:
        print(f"target reached")
    else: print(f"target not reached")
    rp.render_map(target_node)