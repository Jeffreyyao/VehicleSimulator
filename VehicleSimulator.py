import math, sys
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5 import QtTest
from PIL import Image, ImageDraw
import numpy as np

import RungeKuttaSolver, DynRRTPlanner, Models

class VehicleSimulator(QWidget):
    def __init__(self):
        super().__init__()

        self.sim_size = (300,300)
        self.world_size = (4,4)
        self.vehicle_size = (0.25,0.15)
        self.sim_height = self.sim_size[0]
        self.sim_width = self.sim_size[1]
        self.world_height = self.world_size[0]
        self.world_width = self.world_size[1]
        self.vehicle_length = self.vehicle_size[0]
        self.vehicle_width = self.vehicle_size[1]
        
        self.bounds = []

        self.canvas = QLabel()
        self.canvas.mousePressEvent = self.canvas_press_event
        self.status = QLabel()
        self.box = QVBoxLayout()
        self.box.addWidget(self.canvas)
        self.box.addWidget(self.status)
        self.setLayout(self.box)

        self.model = Models.DeepRacerModel()
        #self.model = Models.DeepRacerModelAlt()
        #self.model = Models.BicycleModel(self.vehicle_length)
        self.dynamics_dimension = self.model.dimensions
        self.dynamics = self.model.dynamics
        self.x = [0.5,0.5,0.0,0.0][:self.model.dimensions]
        self.u = [0,0]
        self.tau = 0.03
        self.rk_solver = RungeKuttaSolver.RungeKuttaSolver(self.dynamics,self.dynamics_dimension)

        self.timer = QTimer()
        self.timer.timeout.connect(self.timeout)
        self.timer.start(int(self.tau*1000))

        self.max_steer = self.model.max_u[0]
        self.max_velocity = self.model.max_u[1]

        self.is_rrt_started = False
        self.target = None
        self.target_default_size = (0.5,0.5)
        self.obstacles = []
        self.obstacle_default_size = (3.0,0.5)

    def keyPressEvent(self, event:QKeyEvent):
        if event.key() == Qt.Key_Up:
            self.u[1] = self.max_velocity
        elif event.key() == Qt.Key_Right:
            self.u[0] = self.max_steer
        elif event.key() == Qt.Key_Left:
            self.u[0] = -self.max_steer
        elif event.key() == Qt.Key_Down:
            self.u[1] = -self.max_velocity
        elif event.key() == Qt.Key_Space and not self.is_rrt_started:
            self.start_dyn_rrt()
        elif event.key() == Qt.Key_Backspace:
            self.target = None
            self.obstacles = []
        event.accept()
    
    def keyReleaseEvent(self, event:QKeyEvent):
        if event.key() == Qt.Key_Up or event.key() == Qt.Key_Down:
            self.u[1] = 0.0
        elif event.key() == Qt.Key_Right or event.key() == Qt.Key_Left:
            self.u[0] = 0.0
        event.accept()

    def canvas_press_event(self, event:QMouseEvent):
        x = event.pos().x()
        y = event.pos().y()
        [x,y] = self.sim2world([x,y])
        if event.button()==Qt.LeftButton:
            self.target = DynRRTPlanner.Object([x,y],self.target_default_size,"Target","Rectangle")
        elif event.button()==Qt.RightButton:
            self.obstacles.append(DynRRTPlanner.Object([x,y],self.obstacle_default_size,"Obstacle","Rectangle"))

    def timeout(self):
        self.x = self.rk_solver.RK4(self.x,self.u,self.tau)
        self.canvas.setPixmap(self.pil2pixmap(self.draw()))
        self.update_status(self.x,self.u)

    def get_polygon(self, x, y, w, l, theta):
        c,s = math.cos(theta),math.sin(theta)
        coords = [(l/2.0, w/2.0), (l/2.0, -w/2.0), (-l/2.0, -w/2.0), (-l/2.0, w/2.0)]
        return [(c*x_val-s*y_val+x, s*x_val+c*y_val+y) for (x_val,y_val) in coords]
    
    def world2sim(self, pos):
        x = pos[0]/self.world_width*self.sim_width
        y = pos[1]/self.world_height*self.sim_height
        return [x,y]
    
    def sim2world(self, pos):
        x = pos[0]/self.sim_width*self.world_width
        y = pos[1]/self.sim_height*self.world_height
        return [x,y]

    def draw_object(self, draw, obj:DynRRTPlanner.Object):
        [x,y] = self.world2sim(obj.position)
        [w,l] = self.world2sim(obj.size)
        color = "#0000EE"
        if obj.object_type=="Target": color="#00EE00"
        elif obj.object_type=="Obstacle": color="#EE0000"
        draw.polygon(self.get_polygon(x,y,l,w,0), fill=color)

    def draw(self):
        img = Image.new(mode="RGBA", size=(self.sim_width, self.sim_height), color="#AAAAAA")
        draw = ImageDraw.Draw(img)
        for obstacle in self.obstacles:
            self.draw_object(draw,obstacle)
        if self.target is not None:
            self.draw_object(draw,self.target)
        x,y,theta = self.x[0],self.x[1],self.x[2]
        [x,y] = self.world2sim([x,y])
        [w,l] = self.world2sim([self.vehicle_width,self.vehicle_length])
        draw.polygon(self.get_polygon(x,y,w,l,theta), fill="#000000")
        return img
    
    def pil2pixmap(self, im:Image.Image)->QPixmap: # function to convert pil image to pyqt pixmap
        r, g, b, a = im.split()
        im = Image.merge("RGBA", (b, g, r, a))
        data = im.tobytes("raw", "RGBA")
        qim = QImage(data, im.size[0], im.size[1], QImage.Format_ARGB32)
        pixmap = QPixmap.fromImage(qim)
        return pixmap

    def update_status(self, x, u):
        self.status.setText(f"Steer: {str(u[0])[:5]} Velocity: {str(u[1])}\nx: {str(x[0])[:5]} y: {str(x[1])[:5]} theta: {str(x[2])[:5]}")

    def start_dyn_rrt(self):
        self.is_rrt_started = True
        rrt_planner = DynRRTPlanner.DynRRTPlanner(self.world_size,self.target,self.obstacles,self.dynamics,self.x,self.max_steer,self.max_velocity,self.tau,1000)
        target_node = rrt_planner.rrt()
        if target_node is not None:
            transitions = []
            while target_node is not None:
                transitions = [(target_node.state,target_node.t_from_parent)]+transitions
                target_node = target_node.parent
            self.timer.stop()
            for i in range(1,len(transitions)):
                state,t_from_parent = transitions[i]
                for _ in np.arange(0,t_from_parent,self.tau):
                    action = rrt_planner.get_action_greedy(self.x,state[:2])
                    self.x = self.rk_solver.RK4(self.x,action,self.tau)
                    self.canvas.setPixmap(self.pil2pixmap(self.draw()))
                    self.update_status(self.x,action)
                    QtTest.QTest.qWait(int(self.tau*1000))
            self.timer.start()
        else:
            print("path not found")
        self.is_rrt_started = False

if __name__ == "__main__":
    App = QApplication(sys.argv)
    window = VehicleSimulator()
    window.show()
    sys.exit(App.exec())