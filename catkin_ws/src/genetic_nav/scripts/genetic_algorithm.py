#!/usr/bin/env python3
import rospy
import random
import numpy as np
from genetic_nav.msg import Weights
from std_srvs.srv import Empty
from std_msgs.msg import Int32MultiArray, Float32
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion

# Cargar parámetros desde ROS
params = rospy.get_param("/genetic_algorithm")
POP_SIZE        = params["pop_size"]          # tamaño de población
GENOME_WEIGHT_LENGTH   = params["genome_weight_length"]     # longitud de LOS PESOS del genoma
GENOME_BIAS_LENGTH     = params["genome_bias_length"]       # longitud de LOS BIAS del genoma
MUT_PROB        = params["mut_prob"]          # prob. de mutación
CX_PROB         = params["cx_prob"]           # prob. de cruce
NUM_GENERATIONS = params["num_gen"]            # iteraciones del GA
CHECKPOINT_VALUE = 20.0
CRASH_PENALTY = 0.5


# def evaluate(params, time_elapsed):
#     w1, w2 = 0.7, 0.3
#     return w1 * params[0] + w2 * time_elapsed - 20 * params[1]
def evaluate(params, time_elapsed):
    checkpoints_reached = params[0]
    crashed = params[1]
    
    # DRAMATICALLY increase checkpoint reward (from 10 to 50)
    checkpoint_reward = CHECKPOINT_VALUE * checkpoints_reached
    
    crash_penalty = 1
    if crashed == 1:
        crash_penalty = CRASH_PENALTY
    
    # Add completion bonus for robots that reach many checkpoints
    completion_bonus = 0
    total_checkpoints = 32  # From checkpoints.yaml
    if checkpoints_reached > 0:
        # Give exponentially increasing bonus as robots get closer to finishing
        progress_percentage = checkpoints_reached / total_checkpoints
        completion_bonus = 100.0 * (progress_percentage ** 2)  # Squared for exponential reward
    
    # Much smaller time penalties - only significant for robots making no progress
    if checkpoints_reached == 0:
        # Still penalize robots that just spin in place
        time_factor = -min(0.2 * time_elapsed, 10.0)
    #elif checkpoints_reached < 5:
    #    # Very small penalty for robots making some progress
    #    time_factor = -min(0.05 * time_elapsed, 3.0)
    else:
        # Negligible time penalty for robots with good progress
        time_factor = 0
    
    # Print detailed breakdown for debugging
    print(f"Checkpoint reward: {checkpoint_reward:.1f}, Completion bonus: {completion_bonus:.1f}, " + 
          f"Time factor: {time_factor:.1f}, Crash penalty: {crash_penalty:.1f}")
    
    return (checkpoint_reward + completion_bonus + time_factor) * crash_penalty

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

# def seleccion_por_torneo(poblacion, fitness_fn, k=3):
#     """Selecciona un individuo usando torneo de tamaño k."""
#     participantes = random.sample(poblacion, k)
#     mejor = max(participantes, key=fitness_fn)
#     return mejor

def seleccion_por_torneo(poblacion, fitnesses, k=3):
    """Selecciona un individuo usando torneo de tamaño k."""
    # Get k random indices
    indices = random.sample(range(len(poblacion)), k)
    # Find the index with maximum fitness
    mejor_idx = max(indices, key=lambda i: fitnesses[i])
    return poblacion[mejor_idx]



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


def export_chromosome(p):
    try:
        file = open("bestChromosome", "x")
    except:
        file = open("bestChromosome", "w")
    for w in p[0]:
        file.write(f"{w} ")
    file.write('\n')
    for b in p[1]:
        file.write(f"{b} ")
    

def import_chromosome():
    try:
        file = open("bestChromosome", 'r')
        chromosome = []
        weights = []
        biases = []
        i = 0
        for line in file:
            line = line.split(' ')
            if i == 0:
                for w in line:
                    if w != '':
                        weights.append(float(w))
                i = i + 1
            else:
                for b in line:
                    if b != '':
                        biases.append(float(b))
        chromosome.append(weights)
        chromosome.append(biases)
        return chromosome
    except:
        return None

if __name__ == '__main__':
    rospy.init_node("genetic_algorithm")
    pub = rospy.Publisher('chromosomes', Weights, queue_size=10, latch=True)
    plot_pub = rospy.Publisher('fitness', Float32, queue_size=10, latch=True)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    rospy.loginfo("Topico genetico creado")

    lastBest = import_chromosome()
    if lastBest == None:
        population = [[
                        [random.uniform(-0.5,0.5) for i in range(GENOME_WEIGHT_LENGTH)], 
                        [random.uniform(0.0,0.1) for i in range(GENOME_BIAS_LENGTH)]] 
                       for i in range(POP_SIZE)]
    else:
        population = [[
                        [random.uniform(-0.5,0.5) for i in range(GENOME_WEIGHT_LENGTH)], 
                        [random.uniform(0.0,0.1) for i in range(GENOME_BIAS_LENGTH)]] 
                       for i in range(POP_SIZE - 1)]
        population.append(lastBest)

    rospy.loginfo("Población creada")
    # Proxies para estados
    rospy.wait_for_service('/gazebo/get_model_state')
    rospy.wait_for_service('/gazebo/set_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Definir estado inicial del robot
    initial_state = ModelState()
    initial_state.model_name = 'turtlebot3'
    initial_state.pose = Pose(
        position=Point(x=-6.5, y=8.5, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    initial_state.twist = Twist()
    initial_state.reference_frame = 'world'

    # record = stats.compile(population)
    for gen in range(NUM_GENERATIONS):
        rospy.loginfo(f"Generación {gen}")
        fitnesses = []
        i = 0
        for ind in population:
            i += 1
            print(f"individuo n {i} de la generación {gen}")
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
            print(f"Robot #{len(fitnesses)} fitness score: {fit:.4f}")

            rospy.sleep(0.1)  # pequeña pausa por seguridad
            start_again(reset_world, set_model_state, initial_state)
            rospy.loginfo("Miembro acaba")
            msg = Float32()
            msg.data = fit
            plot_pub.publish(msg)
        if gen == NUM_GENERATIONS/2:
            CX_PROB = CX_PROB + 0.4
        # Add after calculating fitnesses for the generation
        rospy.loginfo('Mejor fitness gen %d: %.2f', gen, max(fitnesses))
      
        

        # Selección y generación siguiente
        new_pop = []
        # Elitismo
        elite_idx = int(np.argmax(fitnesses))
        new_pop.append(population[elite_idx])
        export_chromosome(population[elite_idx])

        while len(new_pop) < POP_SIZE:
            p1 = seleccion_por_torneo(population, fitnesses, 5)
            p2 = seleccion_por_torneo(population, fitnesses, 5)
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
        rospy.loginfo("Mejores fitness gén %d: %.2f", gen+1, max(fitnesses))
