#!/usr/bin/env python3
import rospy
import tf
from tf.transformations import euler_from_quaternion

def checkpoint_detector():
    # Iniciar nodo ROS
    rospy.init_node("checkpoint_detector")
    
    # Configurar listener de TF (para obtener la posición del robot)
    listener = tf.TransformListener()
    rate = rospy.Rate(10)  # Frecuencia de 10Hz

    while not rospy.is_shutdown():
        try:
            # Obtener la posición y orientación actual del robot (desde el mapa al base_link)
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            x, y = trans[0], trans[1]  # Coordenadas x, y
            
            # Verificar proximidad a cada checkpoint
            for i, point in enumerate(checkpoints):
                # Si está cerca (umbral de 0.3 metros en x/y)
                if abs(x - point["x"]) < 0.3 and abs(y - point["y"]) < 0.3:
                    rospy.set_param("/reached_checkpoint", True)  # Activar flag
                    rospy.loginfo(f"Checkpoint {i} alcanzado!")
                    
        except (tf.LookupException, tf.ConnectivityException) as e:
            # Manejar errores de TF (ej: transformación no disponible)
            rospy.logwarn(f"Error en TF: {e}")
            continue
        
        rate.sleep()  # Esperar para mantener la frecuencia

if __name__ == "__main__":
    checkpoint_detector()  # Ejecutar la función principal