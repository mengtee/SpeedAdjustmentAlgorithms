import random
import numpy as np
from deap import base, creator, tools, algorithms
import subprocess
import time
import optparse
import os
import sys
import math
import numpy as np

os.environ['SUMO_HOME'] = '/Users/TEEMENGKIAT/sumo'

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options


def evaluate(individual):
    horizon, dt, max_speed, min_speed, safety_margin = individual

    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")
    traci.start([sumoBinary, '-c', 'multiple_vehicles_tjunction.sumocfg', "--tripinfo-output", "tripinfo.xml"])
    total_travel_time, collisions = run_simulation(horizon, dt, max_speed, min_speed, safety_margin)
    traci.close()
    return total_travel_time, collisions

def run_simulation(horizon, dt, max_speed, min_speed, safety_margin):
    step = 0
    total_travel_time = 0
    collisions = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()
        
        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
        
        if len(vehicles) > 0:
            calculate_speed_adjustments(vehicles, horizon, dt, max_speed, min_speed, safety_margin)
        
        for vehicle_id in vehicles:
            speed = traci.vehicle.getSpeed(vehicle_id)
            total_travel_time += 1 / speed if speed > 0 else 0
            if detect_collision(vehicle_id):
                collisions += 1
        
        step += 1
    
    return total_travel_time, collisions

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_speed_adjustments(vehicles, horizon, dt, max_speed, min_speed, safety_margin):
    vehicle_props = {}
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])
        time_to_junction = distance_to_junction / speed if speed > 0 else float('inf')
        vehicle_props[veh] = {"pos": pos, "speed": speed, "distance_to_junction": distance_to_junction, "time_to_junction": time_to_junction}

    priorities = {}
    for veh in vehicle_props:
        priority = -vehicle_props[veh]["distance_to_junction"]
        priorities[veh] = priority

    sorted_vehicles = sorted(priorities, key=priorities.get, reverse=True)
    
    predicted_states = np.zeros((horizon, len(sorted_vehicles), 2))
    for i, veh in enumerate(sorted_vehicles):
        predicted_states[0, i, 0] = vehicle_props[veh]["distance_to_junction"]
        predicted_states[0, i, 1] = vehicle_props[veh]["speed"]

    for t in range(1, horizon):
        for i in range(len(sorted_vehicles)):
            predicted_states[t, i, 0] = predicted_states[t-1, i, 0] + dt * predicted_states[t-1, i, 1]
            predicted_states[t, i, 1] = predicted_states[t-1, i, 1]

    for i, veh in enumerate(sorted_vehicles):
        for t in range(horizon):
            if t > 0:
                for j in range(i):
                    collision_time = predict_collision_time(predicted_states[t-1, i, 0], predicted_states[t-1, i, 1], predicted_states[t-1, j, 0], predicted_states[t-1, j, 1])
                    if abs(collision_time) < safety_margin:
                        new_speed = max(min_speed, predicted_states[t-1, i, 1] - 0.5 * (predicted_states[t-1, i, 1] - predicted_states[t-1, j, 1]))
                        traci.vehicle.slowDown(veh, new_speed, 5)
                        break
                else:
                    traci.vehicle.slowDown(veh, max_speed, 5)

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def predict_collision_time(distance1, speed1, distance2, speed2):
    if speed1 == 0 or speed2 == 0:
        return float('inf')
    return (distance1 / speed1) - (distance2 / speed2)

def detect_collision(vehicle_id):
    # Implement your collision detection logic here
    return False

# Ensure you don't redefine 'tools' anywhere in your code before this point
creator.create("FitnessMin", base.Fitness, weights=(-1.0, -1.0))
creator.create("Individual", list, fitness=creator.FitnessMin)

toolbox = base.Toolbox()
toolbox.register("attr_float", random.uniform, 0.01, 50.0)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=5)
toolbox.register("population", tools.initPopulation, list, toolbox.individual)

toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)
toolbox.register("evaluate", evaluate)

def main():
    population = toolbox.population(n=50)
    algorithms.eaSimple(population, toolbox, cxpb=0.5, mutpb=0.2, ngen=40, stats=None, halloffame=None, verbose=True)

if __name__ == "__main__":
    main()