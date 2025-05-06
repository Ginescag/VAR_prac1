#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

class TrajectoryController:
    def __init__(self):
        rospy.init_node("trajectory_controller")
        self.target = None                       # Coordenada objetivo actual
        self.current_pose = None                 # Pose actual del robot (x, y, theta)
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.trajectory_sub = rospy.Subscriber("/target_trajectory", Point, self.trajectory_callback)
        
    def odom_callback(self, msg):
        """Actualiza la pose actual del robot desde la odometría."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = (x, y, theta)
    
    def trajectory_callback(self, msg):
        """Recibe una nueva coordenada objetivo y comienza a seguirla."""
        self.target = (msg.x, msg.y)
        self.follow_target()
    
    def follow_target(self):
        """Controlador PID para moverse hacia la coordenada objetivo."""
        rate = rospy.Rate(10)  # 10 Hz
        while self.target and not rospy.is_shutdown():
            if self.current_pose is None:
                continue  # Esperar primera actualización de odometría
            
            dx = self.target[0] - self.current_pose[0]
            dy = self.target[1] - self.current_pose[1]
            distance = np.hypot(dx, dy)
            angle_to_target = np.arctan2(dy, dx)
            angle_error = angle_to_target - self.current_pose[2]
            
            # Controlador proporcional
            v = min(0.5 * distance, 0.5)  # Velocidad lineal máxima 0.5 m/s
            w = 1.5 * angle_error         # Ganancia angular
            
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.velocity_pub.publish(twist)
            
            if distance < 0.1:  # Umbral de llegada (10 cm)
                break
            rate.sleep()

if __name__ == "__main__":
    controller = TrajectoryController()
    rospy.spin()