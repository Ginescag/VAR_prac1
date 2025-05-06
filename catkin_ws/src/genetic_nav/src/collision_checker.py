#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import numpy as np

class CollisionChecker:
    def __init__(self):
        rospy.init_node("collision_checker")
        self.collision_pub = rospy.Publisher("/collision", Bool, queue_size=10)
        self.checkpoints = rospy.get_param("/checkpoints")  # Lista de checkpoints
        self.current_checkpoint = 0                         # Checkpoint actual
        self.start_time = rospy.Time.now()                 # Tiempo de inicio
        
        # Suscriptores
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
    
    def scan_callback(self, msg):
        """Detecta colisiones usando el LIDAR."""
        collision = min(msg.ranges) < 0.2  # Umbral de 20 cm
        self.collision_pub.publish(Bool(collision))
        rospy.set_param("/collision_occurred", collision)
        
        if collision:
            rospy.set_param("/simulation_done", True)  # Terminar simulación
    
    def odom_callback(self, msg):
        """Verifica checkpoints alcanzados y mide el tiempo."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Verificar checkpoints
        if self.current_checkpoint < len(self.checkpoints):
            target = self.checkpoints[self.current_checkpoint]
            dx = x - target["x"]
            dy = y - target["y"]
            if np.hypot(dx, dy) < 0.3:  # Umbral de 30 cm
                self.current_checkpoint += 1
                rospy.set_param("/checkpoints_reached", self.current_checkpoint)
        
        # Actualizar tiempo transcurrido
        elapsed = (rospy.Time.now() - self.start_time).to_sec()
        rospy.set_param("/time_elapsed", elapsed)
        if elapsed >= rospy.get_param("/genetic_algorithm/max_time"):
            rospy.set_param("/simulation_done", True)

if __name__ == "__main__":
    checker = CollisionChecker()
    rospy.spin()