#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from tensorflow import keras
from rocon_std_msgs.msg import StringArray
from sensor_msgs.msg import PointCloud2
import PointNet

CHECKPOINTS = rospy.get_param("/checkpoints")  # Checkpoints del circuito


class TrajectoryController:
    def __init__(self):
        rospy.init_node("trajectory_controller")
        self.prev_pose = None
        self.current_pose = (0.0, 0.0, 90.0)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.chromosomes_sub = rospy.Subscriber("chromosomes", StringArray, self.chromosomes_callback)
        self.lidar_sub = rospy.Subscriber("/scan", )
        
    def odom_callback(self, msg):
        """Actualiza la pose actual del robot desde la odometría."""
        self.prev_pose = self.current_pose
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        rospy.loginfo(f"X: {x}, Y: {y}, Theta: {theta}")
        self.current_pose = (x, y, theta)
    
    def chromosomes_callback(self, msg):
        """Recibe una nueva coordenada objetivo y comienza a seguirla."""
        dim_lat, anchura_ini, dropout, grid_decod = int(msg[0]), int(msg[1]), float(msg[2]), int(msg[3])

        self.follow_target(dim_lat, anchura_ini, dropout, grid_decod)
    
    def follow_target(self, dim_lat, anchura_ini, droput, grid_decod):
        """Controlador PID para moverse hacia la coordenada objetivo."""
        pn = PointNet.create_autoencoder(dim_lat )
        

if __name__ == "__main__":
    controller = TrajectoryController()