#!/usr/bin/env python3
import rospy
from deap import base, creator, tools, algorithms
from geometry_msgs.msg import Twist
import random
import yaml

# --- Configuración Inicial ---
# Cargar las coordenadas de los checkpoints desde un archivo YAML
with open("~/tu_workspace/src/genetic_nav/config/checkpoints.yaml", 'r') as f:
    checkpoints = yaml.safe_load(f)["checkpoints"]  # Lista de diccionarios {x, y}

# --- Configuración del Algoritmo Genético con DEAP ---
# 1. Definir la estructura del fitness (objetivo: maximizar checkpoints, minimizar tiempo)
creator.create("FitnessMax", base.Fitness, weights=(1.0, -0.1))  # Pesos positivos/negativos para cada objetivo
creator.create("Individual", list, fitness=creator.FitnessMax)    # Un individuo es una lista de genes (acciones)

# 2. Inicializar herramientas de DEAP
toolbox = base.Toolbox()
# Registro de funciones:
# - attr_float: Genera números aleatorios entre -0.5 y 0.5 (velocidades lineales/angulares)
toolbox.register("attr_float", random.uniform, -0.5, 0.5)
# - individual: Crea un individuo con 20 genes (10 acciones: [linear1, angular1, linear2, angular2, ...])
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=20)
# - population: Crea una población de 50 individuos
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# --- Función de Evaluación (Fitness) ---
def evaluate(individual):
    # Configurar el publicador de comandos de velocidad
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    twist = Twist()
    checkpoint_count = 0
    total_time = 0

    # Ejecutar cada acción del genoma (2 genes por acción: linear + angular)
    for i in range(0, len(individual), 2):
        # Asignar velocidades al robot
        twist.linear.x = individual[i]    # Velocidad lineal (m/s)
        twist.angular.z = individual[i+1] # Velocidad angular (rad/s)
        pub.publish(twist)                # Publicar comando
        
        # Esperar 1 segundo (duración fija por simplicidad)
        rospy.sleep(1.0)
        total_time += 1.0

        # Verificar si se alcanzó un checkpoint (parámetro setado por otro nodo)
        if rospy.get_param("/reached_checkpoint", False):
            checkpoint_count += 1
            rospy.set_param("/reached_checkpoint", False)  # Resetear el flag

    # Devolver tupla de fitness (checkpoints, tiempo)
    return (checkpoint_count, total_time)

# Registrar operadores genéticos
toolbox.register("evaluate", evaluate)                  # Función de evaluación
toolbox.register("mate", tools.cxBlend, alpha=0.5)     # Cruce aritmético (mezcla lineal)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.1)  # Mutación gaussiana
toolbox.register("select", tools.selTournament, tournsize=3)  # Selección por torneo

# --- Punto de Entrada ---
if __name__ == "__main__":
    rospy.init_node("genetic_navigator")  # Iniciar nodo ROS
    
    # Crear población inicial y ejecutar el algoritmo genético
    population = toolbox.population(n=50)
    # Parámetros: cruzamiento (70%), mutación (20%), 100 generaciones
    algorithms.eaSimple(population, toolbox, cxpb=0.7, mutpb=0.2, ngen=100, verbose=True)