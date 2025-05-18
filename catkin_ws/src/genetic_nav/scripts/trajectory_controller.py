#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import  Twist
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from turtlebot3_msgs.msg import SensorState  # Importa el mensaje correcto
from genetic_nav.msg import Weights
import NeuralNetwork 
import math
import random

CHECKPOINTS = rospy.get_param("/checkpoints")  # Checkpoints del circuito
MAX_LIDAR_DIST = 20.5
NUM_LIDAR_POINTS = 15
MAX_TIME_WITHOUT_PROGRESS = 20.0
class TrajectoryController:
    def __init__(self):
        rospy.init_node("trajectory_controller")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.check_pub = rospy.Publisher('fitness_parameters', Int32MultiArray, queue_size=10)

    def obtener_posicion(self):
        odom = rospy.wait_for_message("/odom", Odometry)
        #rospy.loginfo("Posición obtenida")
        x_odom = odom.pose.pose.position.x
        y_odom = odom.pose.pose.position.y
        return x_odom, y_odom
    
    def obtener_lidar(self):
        #Obtención del escaneo del LIDAR

        scan = rospy.wait_for_message("/scan", LaserScan)
        #rospy.loginfo("Lidar obtenido")
        lidar = np.array(scan.ranges, dtype=np.float64)
        lidar[np.isinf(lidar)] = MAX_LIDAR_DIST

        if len(lidar) >= NUM_LIDAR_POINTS:
            start = (len(lidar) - NUM_LIDAR_POINTS) // 2
            lidar_c = lidar[start:start + NUM_LIDAR_POINTS]
        else:
            lidar_c = lidar

        lidar_i = np.mean(lidar[:NUM_LIDAR_POINTS])
        lidar_d = np.mean(lidar[-NUM_LIDAR_POINTS:])
        lidar_c = np.mean(lidar_c)
        return lidar_i, lidar_c, lidar_d
    
    def define_movement(self, accion):
        
        # Crear el mensaje Twist
        twist = Twist()
        
        # Definir velocidades según la acción predicha
        if accion == 0:
            # Girar a la izquierda
            twist.linear.x = 0.1
            twist.angular.z = 0.5  # Ajusta este valor según la necesidad
            #rospy.loginfo("Izquierda")
        elif accion == 1:
            # Avanzar recto
            twist.linear.x = 0.2  # Ajusta este valor según la necesidad
            twist.angular.z = 0.0
            #rospy.loginfo("Recto")
        elif accion == 2:
            # Girar a la derecha
            twist.linear.x = 0.1
            twist.angular.z = -0.5  # Ajusta este valor según la necesidad
            #rospy.loginfo("Derecha")
        
        return twist
    
    def get_robot_heading(self):
        """Extract robot's heading (yaw) from odometry quaternion"""
        odom = rospy.wait_for_message("/odom", Odometry)
        
        # Extract quaternion
        x = odom.pose.pose.orientation.x
        y = odom.pose.pose.orientation.y
        z = odom.pose.pose.orientation.z
        w = odom.pose.pose.orientation.w
        
        # Convert to Euler angles (yaw)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw

    def get_relative_checkpoint_info(self, x, y):
        """Calculate relative distance and angle to next checkpoint"""
        # Get next checkpoint coordinates
        checkpoint = CHECKPOINTS[self.current_checkpoint]
        if isinstance(checkpoint, dict):
            cp_x = float(checkpoint["x"])
            cp_y = float(checkpoint["y"])
        else:
            cp_x = float(checkpoint[0])
            cp_y = float(checkpoint[1])
        
        # Distance to checkpoint (normalized to [0,1])
        dx = cp_x - x
        dy = cp_y - y
        distance = math.sqrt(dx**2 + dy**2)
        max_possible_distance = 30.0  # Estimate of max distance in map
        norm_distance = min(distance / max_possible_distance, 1.0)
        
        # Angle to checkpoint in robot's reference frame
        absolute_angle = math.atan2(dy, dx)  # Absolute angle to checkpoint
        robot_heading = self.get_robot_heading()  # Current robot orientation
        relative_angle = absolute_angle - robot_heading  # Angle relative to robot
        
        # Normalize angle to [-1, 1] range
        normalized_angle = relative_angle / math.pi
        
        return norm_distance, normalized_angle
    
    
    def is_near_checkpoint(self, current_pos, threshold=1.0):
        """Check if robot is within threshold distance of current checkpoint"""
        if self.current_checkpoint >= len(CHECKPOINTS):
            return False
            
        checkpoint = CHECKPOINTS[self.current_checkpoint]
        if isinstance(checkpoint, dict):
            cp_x = float(checkpoint["x"])
            cp_y = float(checkpoint["y"])
        else:
            cp_x = float(checkpoint[0])
            cp_y = float(checkpoint[1])
        
        x, y = current_pos
        distance = math.sqrt((x - cp_x)**2 + (y - cp_y)**2)
        return distance < threshold


    def move(self):
        while True:
            #Inicialización de la red neuronal
            rospy.loginfo("Esperando los cromosomas")
            msg = rospy.wait_for_message("chromosomes", Weights)
            rospy.loginfo("Han llegado los cromosomas")

            self.current_checkpoint = 0
            pesos_flat = msg.pesos
            bias_flat = msg.bias

            pesos_list, bias_list = NeuralNetwork.reconstruir_listas(pesos_flat, bias_flat)
            
            model = NeuralNetwork.build_model_estandar(pesos_list, bias_list)
            x, y = self.obtener_posicion()
            crash = 0
            r = rospy.Rate(10)


            # Add progress tracking
            last_checkpoint_time = rospy.Time.now()
            # Maximum 30 seconds without reaching a checkpoint
                 
            # Add movement tracking
            last_positions = []  # Store recent positions to detect circles/loops

            while self.current_checkpoint < len(CHECKPOINTS):
                #Repetir hasta que choque o llegue al último checkpoint

                current_time = rospy.Time.now()

                # Check for lack of checkpoint progress
                time_since_last_checkpoint = (abs(current_time - last_checkpoint_time)).to_sec()
                if time_since_last_checkpoint > MAX_TIME_WITHOUT_PROGRESS:
                    rospy.logwarn(f"No checkpoint progress for {MAX_TIME_WITHOUT_PROGRESS} seconds")
                    crash = 1  # Mark as crashed due to lack of progress
                    break

                lidar_i, lidar_c, lidar_d = self.obtener_lidar()
                norm_lidar_i = min(lidar_i / MAX_LIDAR_DIST, 1.0)
                norm_lidar_c = min(lidar_c / MAX_LIDAR_DIST, 1.0)
                norm_lidar_d = min(lidar_d / MAX_LIDAR_DIST, 1.0)
                
                # Get relative distance and angle to checkpoint
                dist_to_cp, angle_to_cp = self.get_relative_checkpoint_info(x, y)
            
                # Create network input with relative measures instead of absolute positions
                entrada = np.array([[
                    #dist_to_cp,     # Normalized distance to checkpoint [0,1]
                    #angle_to_cp,    # Normalized angle to checkpoint [-1,1]
                    x,
                    y,
                    norm_lidar_i,   # Normalized left lidar [0,1]
                    norm_lidar_c,   # Normalized center lidar [0,1]
                    norm_lidar_d    # Normalized right lidar [0,1]
                ]], dtype=np.float32)

                #Obtener la predicción
                salida = model.predict(entrada, verbose=0)
                accion = np.argmax(salida)

                twist = self.define_movement(accion)        
                self.pub.publish(twist)
                r.sleep()
                
                nx, ny = self.obtener_posicion()
                #Si no ha cambiado de posicion en el tiempo es que ha chocado contra una pared
                if abs(x - nx) < 0.0001 and abs(y - ny) < 0.0001:
                    rospy.logwarn(f"The robot has crashed: RIP!!")
                    crash = 1
                    break

                # Store position for loop detection
                #last_positions.append((nx, ny))
                #if len(last_positions) > 50:  # Keep last 50 positions (5 seconds at 10Hz)
                #    last_positions.pop(0)

                # Use the new distance-based checkpoint detection
                if self.is_near_checkpoint([nx, ny], threshold=1.0):  # Increased threshold to 1.0m for easier detection
                    self.current_checkpoint += 1
                    last_checkpoint_time = rospy.Time.now()
                    rospy.loginfo(f"Checkpoint {self.current_checkpoint-1}/{len(CHECKPOINTS)} alcanzado!")

                # Add debugging to show robot's position relative to checkpoint
                if self.current_checkpoint < len(CHECKPOINTS):
                    checkpoint = CHECKPOINTS[self.current_checkpoint]
                    if isinstance(checkpoint, dict):
                        cp_x = float(checkpoint["x"])
                        cp_y = float(checkpoint["y"])
                    else:
                        cp_x = float(checkpoint[0])
                        cp_y = float(checkpoint[1])
                    
                    dist = math.sqrt((nx - cp_x)**2 + (ny - cp_y)**2)
                    if random.random() < 0.1:  # Print 10% of the time to avoid flooding logs
                        rospy.loginfo(f"Robot en ({nx:.1f}, {ny:.1f}), checkpoint en ({cp_x:.1f}, {cp_y:.1f}), dist: {dist:.1f}m")

                x, y = nx, ny

            msg = Int32MultiArray()    
            msg.data = [self.current_checkpoint, crash]
            self.check_pub.publish(msg)

        

if __name__ == "__main__":
    controller = TrajectoryController()
    rospy.loginfo("Trajectory Controller creado")
    controller.move()