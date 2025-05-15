#!/usr/bin/env python3
import rospy
import random
import numpy as np
from genetic_nav.msg import Weights
from std_srvs.srv import Empty
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion

# Cargar parámetros desde ROS
params = rospy.get_param("/genetic_algorithm")
POP_SIZE        = params["pop_size"]          # tamaño de población
GENOME_LENGTH   = params["genome_length"]     # longitud del genoma
MUT_PROB        = params["mut_prob"]          # prob. de mutación
CX_PROB         = params["cx_prob"]           # prob. de cruce
NUM_GENERATIONS = params["num_gen"]            # iteraciones del GA


def evaluate(params, time_elapsed):
    w1, w2 = 0.7, 0.3
    return w1 * params[0] + w2 * time_elapsed - 20 * params[1]

def get_state(model, reference="world"):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = srv(model, reference)
        if resp.success:
            return resp.pose.position.x, resp.pose.position.y
        else:
            rospy.logwarn("Error al obtener estado: %s", resp.status_message)
            return None, None
    except rospy.ServiceException as e:
        rospy.logerr("Fallo en /gazebo/get_model_state: %s", e)
        return None, None

def seleccion_por_torneo(poblacion, fitness_fn, k=3):
    """Selecciona un individuo usando torneo de tamaño k."""
    participantes = random.sample(poblacion, k)
    mejor = max(participantes, key=fitness_fn)
    return mejor



def mutate(individuo):
    w, b = individuo
    w_mut = [wi + random.uniform(-1.0, 1.0) for wi in w]
    b_mut = [bi + random.uniform(-0.1, 0.1) for bi in b]
    return [w_mut, b_mut]

def crossover_uniform(p1, p2):
    w1, b1 = p1
    w2, b2 = p2
    child1 = [[random.choice(pair) for pair in zip(w1, w2)],
              [random.choice(pair) for pair in zip(b1, b2)]]
    child2 = [[random.choice(pair) for pair in zip(w1, w2)],
              [random.choice(pair) for pair in zip(b1, b2)]]
    return [child1, child2]

def start_again(reset_world, set_model_state, initial_state):
    # Reset Gazebo
    try:
        reset_world()
        rospy.loginfo("Gazebo reiniciado.")
    except rospy.ServiceException as e:
        rospy.logwarn("Error al resetear Gazebo: %s", e)

    rospy.sleep(0.1)

    # Resetear posición del robot
    try:
        resp = set_model_state(initial_state)
        if resp.success:
            rospy.loginfo("Modelo '%s' reiniciado a posición inicial.", initial_state.model_name)
        else:
            rospy.logwarn("Error al resetear modelo: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Fallo en /gazebo/set_model_state: %s", e)


if __name__ == '__main__':
    rospy.init_node("genetic_algorithm")
    pub = rospy.Publisher('chromosomes', Weights, queue_size=10)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    rospy.loginfo("Topico genetico creado")
    population = [[
                    [random.uniform(-0.5,0.5) for i in range(GENOME_LENGTH)], 
                    [random.uniform(0.0,0.1) for i in range(GENOME_LENGTH)]] 
                   for i in range(POP_SIZE)]
    rospy.loginfo("Población creada")
    # Proxies para estados
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Definir estado inicial del robot
    initial_state = ModelState()
    initial_state.model_name = 'turtlebot3_waffle'
    initial_state.pose = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    initial_state.twist = Twist()
    initial_state.reference_frame = 'world'

    # record = stats.compile(population)
    for gen in range(NUM_GENERATIONS):
        rospy.loginfo(f"Generación {gen}")
        fitnesses = []
        for ind in population:
            pop_pub = Weights()
            pop_pub.pesos = ind[0]
            pop_pub.bias = ind[1]

            start = rospy.Time.now()
            pub.publish(pop_pub)
            rospy.loginfo("Población publicada")
            msg = rospy.wait_for_message("fitness_parameters", Int32MultiArray)
            end = rospy.Time.now()

            tiempo = (end - start).to_sec()
            fit_params = msg.data
            fit = evaluate(fit_params, tiempo)
            fitnesses.append(fit)

            rospy.sleep(0.1)  # pequeña pausa por seguridad
            start_again(reset_world, set_model_state, initial_state)
            rospy.loginfo("Miembro acaba")

        # Selección y generación siguiente
        new_pop = []
        # Elitismo
        elite_idx = int(np.argmax(fitnesses))
        new_pop.append(population[elite_idx])

        while len(new_pop) < POP_SIZE:
            p1 = seleccion_por_torneo(population, fitnesses)
            p2 = seleccion_por_torneo(population, fitnesses)
            if random.random() < CX_PROB:
                children = crossover_uniform(p1, p2)
            else:
                children = [p1, p2]

            for child in children:
                if random.random() < MUT_PROB:
                    child = mutate(child)
                if len(new_pop) < POP_SIZE:
                    new_pop.append(child)

        population = new_pop
        rospy.loginfo("Mejor fitness gén %d: %.2f", gen+1, max(fitnesses))
