#!/usr/bin/env python3
# fitness_plotter.py - Real-time plotting of genetic algorithm fitness

import rospy
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import re
import subprocess
from std_msgs.msg import String
import time

# Global variables to store fitness data
generations = []
best_fitness = []
current_generation = 0
fig, ax = plt.subplots(figsize=(10, 6))
line, = ax.plot([], [], 'b-o', lw=2)

# Configure plot appearance
def setup_plot():
    ax.set_title('Genetic Algorithm Fitness Evolution', fontsize=15)
    ax.set_xlabel('Generation', fontsize=12)
    ax.set_ylabel('Best Fitness Score', fontsize=12)
    ax.grid(True)
    fig.tight_layout()
    return line,

# Update function for the animation
def update(frame):
    global current_generation  # Move this to the top of the function
    
    # Read the latest fitness data from ROS logs
    try:
        latest_data = subprocess.check_output(
            "rosrun rosbag play --topics /rosout | grep -i 'fitness\\|generation' | tail -n 20",
            shell=True, stderr=subprocess.PIPE, text=True
        )
        
        # Process the log data to extract generation and fitness
        for line in latest_data.split('\n'):
            # Look for generation info
            gen_match = re.search(r'Generación (\d+)', line)
            if gen_match:
                gen_num = int(gen_match.group(1))
                if gen_num > current_generation:
                    current_generation = gen_num
                    generations.append(gen_num)
            
            # Look for best fitness info
            fit_match = re.search(r'Mejores fitness gén \d+: ([-\d.]+)', line)
            if fit_match:
                fit_val = float(fit_match.group(1))
                if len(best_fitness) < len(generations):
                    best_fitness.append(fit_val)
            
        # Update the plot
        if len(generations) > 0 and len(best_fitness) > 0:
            line.set_data(generations, best_fitness)
            ax.set_xlim(0, max(10, max(generations) + 1))
            ax.set_ylim(min(min(best_fitness) * 1.1, 0), max(max(best_fitness) * 1.1, 1))
    
    except Exception as e:
        rospy.logwarn(f"Error updating plot: {e}")
    
    return line,