#!/usr/bin/env python3
import rospy
import random
import yaml
import numpy as np
from deap import base, creator, tools, algorithms
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

def random_coord():
    return (random.uniform(-5.0, 5.0), random.uniform(-5.0, 5.0))

# Cargar parámetros desde ROS
params = rospy.get_param("/genetic_algorithm")
MAX_TIME = params["max_time"]          # Tiempo máximo por simulación (60s)
POP_SIZE = params["pop_size"]          # Tamaño de población (50)
GENOME_LENGTH = params["genome_length"]# Número de coordenadas en el genoma (10)
MUT_PROB = params["mut_prob"]          # Probabilidad de mutación (0.2)
CX_PROB = params["cx_prob"]            # Probabilidad de cruce (0.7)
CHECKPOINTS = rospy.get_param("/checkpoints")  # Checkpoints del circuito

# Configurar DEAP para maximizar checkpoints y minimizar tiempo/colisiones
creator.create("FitnessMax", base.Fitness, weights=(1.0, -0.1, -0.5))
creator.create("Individual", list, fitness=creator.FitnessMax)

toolbox = base.Toolbox()
toolbox.register("attr_coord", random_coord)  # Genera tuplas (x,y)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_coord, n=GENOME_LENGTH//2)  # n = número de coordenadas
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# 4. Operadores genéticos (ejemplo)
toolbox.register("mate", tools.cxTwoPoint)
toolbox.register("select", tools.selTournament, tournsize=3)

# Publicador para enviar trayectorias al controlador
traj_pub = rospy.Publisher("/target_trajectory", Point, queue_size=10)

def evaluate(individual):
    """Evalúa un genoma publicando sus coordenadas y midiendo el desempeño."""
    rospy.set_param("/simulation_done", False)
    # Publicar cada coordenada del genoma
    for x, y in individual:
        traj_pub.publish(Point(x, y, 0))
        rospy.sleep(0.1)  # Esperar a que el controlador procese
    
    # Esperar resultados de la simulación
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < MAX_TIME:
        if rospy.get_param("/simulation_done"):
            break
    
    # Obtener métricas
    checkpoints = rospy.get_param("/checkpoints_reached", 0)
    time_elapsed = rospy.get_param("/time_elapsed", MAX_TIME)
    collision = rospy.get_param("/collision_occurred", False)
    fitness = (checkpoints * 100) - (time_elapsed * 1) - (collision * 50)
    # Nueva línea de impresión (añade esto al final, antes del return)
    
    rospy.loginfo(f"\n► Genoma evaluado: {individual}\n\
                ► Fitness: {fitness:.2f}\n\
                ► Checkpoints: {checkpoints}/{len(CHECKPOINTS)}\n\
                ► Tiempo: {time_elapsed:.2f}s\n\
                ► Colisión: {'Sí' if collision else 'No'}")
    
    return (checkpoints, time_elapsed, int(collision))

def mutCustomGaussian(individual, mu=0, sigma=0.5, indpb=0.1):
    for i in range(len(individual)):
        x, y = individual[i]  # Desempaquetamos la tupla
        
        # Mutamos x e y por separado (con probabilidad indpb)
        if random.random() < indpb:
            x += random.gauss(mu, sigma)
        if random.random() < indpb:
            y += random.gauss(mu, sigma)
            
        individual[i] = (x, y)  # Re-empaquetamos
    return individual,

# Configurar operadores genéticos
toolbox.register("mutate", mutCustomGaussian, mu=0, sigma=0.5, indpb=0.1)
toolbox.register("evaluate", evaluate)

def main():
    rospy.init_node("genetic_algorithm")
    population = toolbox.population(n=POP_SIZE)
    
    # Configurar estadísticas y registro
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("std", np.std)
    stats.register("min", np.min)
    stats.register("max", np.max)
    
    # Logbook para guardar el histórico
    logbook = tools.Logbook()
    logbook.header = ["gen", "nevals", "avg", "std", "min", "max"]
    
    # Ejecutar el algoritmo genético
    for gen in range(100):
        offspring = algorithms.varAnd(population, toolbox, cxpb=CX_PROB, mutpb=MUT_PROB)
        fits = toolbox.map(toolbox.evaluate, offspring)
        
        # Asignar fitness a la nueva población
        for ind, fit in zip(offspring, fits):
            ind.fitness.values = fit
        
        # Seleccionar la siguiente generación
        population = toolbox.select(offspring + population, k=POP_SIZE)
        
        # Registrar estadísticas
        record = stats.compile(population)
        logbook.record(gen=gen, nevals=len(offspring), **record)
        
        # Imprimir el mejor individuo de la generación
        best_ind = tools.selBest(population, k=1)[0]
        rospy.loginfo(f"\n*** Generación {gen} ***")
        rospy.loginfo(f"Mejor fitness: {best_ind.fitness.values[0]:.2f}")
        rospy.loginfo(f"Genoma: {best_ind}")
    
    # Guardar el mejor individuo
    with open("best_genome.yaml", 'w') as f:
        yaml.dump({"genome": best_ind}, f)

if __name__ == "__main__":
    main()