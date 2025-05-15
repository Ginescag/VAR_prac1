#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import  Twist
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from turtlebot3_msgs.msg import SensorState  # Importa el mensaje correcto
from genetic_nav.msg import Weights
import NeuralNetowrk

CHECKPOINTS = rospy.get_param("/checkpoints")  # Checkpoints del circuito
MAX_LIDAR_DIST = 13.5
NUM_LIDAR_POINTS = 10
class TrajectoryController:
    def __init__(self):
        rospy.init_node("trajectory_controller")
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.check_pub = rospy.Publisher('fitness_parameters', Int32MultiArray, queue_size=10)

    def obtener_posicion(self):
        odom = rospy.wait_for_message("/odom", Odometry)
        rospy.loginfo("Posición obtenida")
        x_odom = odom.pose.pose.position.x
        y_odom = odom.pose.pose.position.y
        return x_odom, y_odom
    
    def obtener_lidar(self):
        #Obtención del escaneo del LIDAR

        lidar = rospy.wait_for_message("/scan", LaserScan)
        rospy.loginfo("Lidar obtenido")
        lidar = np.array(lidar)
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
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Ajusta este valor según la necesidad
            rospy.loginfo("Izquierda")
        elif accion == 1:
            # Avanzar recto
            twist.linear.x = 0.2  # Ajusta este valor según la necesidad
            twist.angular.z = 0.0
            rospy.loginfo("Recto")
        elif accion == 2:
            # Girar a la derecha
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Ajusta este valor según la necesidad
            rospy.loginfo("Derecha")
        
        return twist
    
    def in_between(self, p1, p2):
        x, y = CHECKPOINTS[self.current_checkpoint]
        x1, y1 = p1
        x2, y2 = p2

        esta_en_rango_x = min(x1, x2) <= x <= max(x1, x2)
        esta_en_rango_y = min(y1, y2) <= y <= max(y1, y2)
        return esta_en_rango_x and esta_en_rango_y




    def move(self):
        while True:
            #Inicialización de la red neuronal
            rospy.loginfo("Esperando los cromosomas")
            msg = rospy.wait_for_message("chromosomes", Weights)
            rospy.loginfo("Han llegado los cromosomas")

            self.current_checkpoint = 0
            pesos = msg.pesos
            bias = msg.bias
            model = NeuralNetowrk.build_model_estandar(pesos, bias)
            x, y = self.obtener_posicion()
            crash = 0
            r = rospy.Rate(10)

            while self.current_checkpoint < len(CHECKPOINTS):
                #Repetir hasta que choque o llegue al último checkpoint
                lidar_i, lidar_c, lidar_d = self.obtener_lidar()
                entrada = [x, y, lidar_i, lidar_c, lidar_d]

                #Obtener la predicción
                salida = model.predict(entrada)
                accion = np.argmax(salida)

                twist = self.define_movement(accion)        
                self.pub.publish(twist)
                r.sleep()
                nx, ny = self.obtener_posicion()
                #Si no ha cambiado de posicion en el tiempo es que ha chocado contra una pared
                if abs(x - nx) < 0.001 and abs(y - ny) < 0.001:
                    crash = 1
                    break
                
                checkpoint_passed = self.in_between([x, y], [nx, ny])
                if checkpoint_passed:
                    self.current_checkpoint += 1
                x, y = nx, ny

            msg = Int32MultiArray()    
            msg.data = [self.current_checkpoint, 1]
            self.check_pub.publish(msg)

        

if __name__ == "__main__":
    controller = TrajectoryController()
    rospy.loginfo("Trajectory Controller creado")
    controller.move()