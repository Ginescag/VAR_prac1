import rospy
import random
import numpy as np
import yaml
from rocon_std_msgs.msg import StringArray

# Cargar parámetros desde ROS
params = rospy.get_param("/genetic_algorithm")
POP_SIZE = params["pop_size"]          # Tamaño de población (50)
GENOME_LENGTH = params["genome_length"]# Número de pesos en el genoma (10)
MUT_PROB = params["mut_prob"]          # Probabilidad de mutación (0.2)
CX_PROB = params["cx_prob"]            # Probabilidad de cruce (0.7)
NUM_GENERATIONS = params["num_gen"]

# # Configurar DEAP para maximizar checkpoints y minimizar tiempo/colisiones
# creator.create("FitnessMax", base.Fitness, weights=(1.0,))
# creator.create("Individual", list, fitness=creator.FitnessMax)

# toolbox = base.Toolbox()
# toolbox.register("attr_params", random.uniform, 0.0, 100.0)  # Genera tuplas (x,y)
# toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_params, GENOME_LENGTH)  # n = número de coordenadas
# toolbox.register("population", tools.initRepeat, list, toolbox.individual)

def calculate_fitness():   
    return 0.0

def evaluate(individual):
    # rospy.loginfo(CHECKPOINTS)
    # Esperar resultados de la simulación
    start_time = rospy.Time.now()
    rospy.wait_for_message("/simulation_done", bool)
    end_time = rospy.Time.now()
    checkpoints = rospy.get_param("/checkpoints_reached", 0)
    rospy.loginfo(checkpoints)
    collision = rospy.get_param("/collision_occurred", False)
    fitness = calculate_fitness()
    return (fitness,)

def custom_mutate(p1, p2):
    return p1

# toolbox.register("mate", tools.cxTwoPoint) # Crossover
# toolbox.register("mutate", tools.mutFlipBit, indpb=0.1) # Mutacion
# toolbox.register("select", tools.selTournament, tournsize=3) # Selecion
# toolbox.register("evaluate", evaluate) # Evaluacion

if __name__ == '__main__':
    rospy.init_node("genetic_algorithm")
    rospy.loginfo("Starting the Genetic ALgorithm")
    pub = rospy.Publisher('chromosomes', StringArray)

    population = [[str(random.randint(64, 256)), #Dimension código latente
                   str(random.randint(32, 128)), #Anchura de las capas
                   str(random.uniform(0,0.5)),   #Dropout
                   str(random.randint(16, 32))]  #Tamaño grid decoder
                   for i in range(POP_SIZE)]

    # record = stats.compile(population)
    for gen in range(NUM_GENERATIONS):
        for pop in population:
            pub.publish(pop)
            
        # offspring = toolbox.select(populsation, len(population))
        
        # Clonamos a los invidiuos seleccionados
        offspring = list(map(toolbox.clone, offspring))

        # Aplicamos crossover y mutacion a los inviduos seleccionados
        for child1, child2 in zip(offspring[::2], offspring[1::2]):
            if random.random() < CX_PROB:
                toolbox.mate(child1, child2)
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:
            if random.random() < MUT_PROB:
                toolbox.mutate(mutant)
                del mutant.fitness.values
        
        # Evaluamos a los individuos con una fitness invalida
        weak_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = list(map(toolbox.evaluate, weak_ind))
        for ind, fit in zip(weak_ind, fitnesses):
            ind.fitness.values = fit
        print("Individuos evaluados: {}".format(len(weak_ind)))

        # Reemplazamos a la poblacion completamente por los nuevos descendientes
        population[:] = offspring

        # Mostramos las salidas de la estadisticas de la generacion actual
        fits = [ind.fitness.values[0] for ind in population]
