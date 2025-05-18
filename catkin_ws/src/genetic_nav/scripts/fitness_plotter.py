#!/usr/bin/env python3
# Simple fitness plotter

import matplotlib.pyplot as plt
import glob
import os
import re
import time

def find_latest_log():
    """Find the most recent genetic_algorithm log file"""
    log_path = os.path.expanduser('~/.ros/log/')
    logs = []
    
    # Look in all subdirectories
    for root, dirs, files in os.walk(log_path):
        for file in files:
            if file.startswith('genetic_algorithm') and file.endswith('.log'):
                logs.append(os.path.join(root, file))
    
    if not logs:
        return None
        
    # Return the most recently modified file
    return max(logs, key=os.path.getmtime)

def main():
    plt.figure(figsize=(10, 6))
    generations = []
    fitness_values = []
    
    print("Fitness Evolution Plotter")
    print("Looking for genetic_algorithm log files...")
    
    last_gen = -1
    
    while True:
        log_file = find_latest_log()
        if not log_file:
            print("No log files found yet. Waiting...")
            time.sleep(5)
            continue
            
        print(f"Reading from: {log_file}")
        
        try:
            with open(log_file, 'r') as f:
                content = f.read()
                
            # Look for generation info
            gen_logs = re.findall(r'Generación (\d+)', content)
            
            # Look for fitness info (simple version that finds all fitness mentions)
            fit_logs = re.findall(r'Mejor fitness gen (\d+): ([-\d.]+)', content)
            
            if fit_logs:
                # Reset data
                generations = []
                fitness_values = []
                
                # Extract values
                for gen, fitness in fit_logs:
                    gen_num = int(gen)
                    fit_val = float(fitness)
                    
                    if gen_num > last_gen:
                        generations.append(gen_num)
                        fitness_values.append(fit_val)
                        last_gen = gen_num
                        print(f"Generation {gen_num}: Best fitness = {fit_val}")
                
                # Update the plot if we have data
                if generations:
                    plt.clf()
                    plt.plot(generations, fitness_values, 'b-o', linewidth=2)
                    plt.title('Genetic Algorithm: Best Fitness Evolution')
                    plt.xlabel('Generation')
                    plt.ylabel('Best Fitness Score')
                    plt.grid(True)
                    
                    # Dynamic y-axis scaling
                    if min(fitness_values) < 0:
                        y_min = min(fitness_values) * 1.1
                    else:
                        y_min = 0
                        
                    y_max = max(1, max(fitness_values) * 1.1)
                    plt.ylim(y_min, y_max)
                    
                    plt.draw()
                    plt.pause(0.01)
            
            time.sleep(5)  # Check every 5 seconds
            
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(5)

if __name__ == "__main__":
    print("Make sure your genetic_algorithm.py includes this line:")
    print("rospy.loginfo(\"Mejor fitness gen %d: %.2f\", gen, max(fitnesses))")
    print("Starting plotter...")
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting plotter.")