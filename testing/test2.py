"""
Advanced Speed Adjustment Algorithm for Autonomous Vehicles at T-Junctions
Applying GA to the overall of the traffic, sounds like a centralise system
Report test case 2:
2. Centralised algorithms for T junction with genetic algorithms
"""

import optparse
import os
import sys
import math
import numpy as np
from deap import base, creator, tools, algorithms

# Ensure SUMO environment is set
os.environ['SUMO_HOME'] = '/Users/TEEMENGKIAT/sumo'

if 'SUMO_HOME' in os.environ:
    tools_path = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools_path)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

# Constants
MAX_SPEED = 15.0  # constant max speed
MIN_SPEED = 2.0   # constant min speed

def get_options():
    opt_parser = optparse.OptionParser()
    options, args = opt_parser.parse_args()
    return options

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def predict_collision_time(distance1, speed1, distance2, speed2):
    if speed1 == 0 or speed2 == 0:
        return float('inf')
    return (distance1 / speed1) - (distance2 / speed2)

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_speed_adjustments(vehicles, horizon=20, dt=0.01, safety_margin=3):
    horizon = max(int(round(horizon)), 1)  # Ensure horizon is at least 1 and round to the nearest integer
    dt = max(float(dt), 0.01)  # Ensure dt is positive
    
    print(f"Parameters: horizon={horizon}, dt={dt}, max_speed={MAX_SPEED}, min_speed={MIN_SPEED}, safety_margin={safety_margin}")
    
    vehicle_props = {}
    junction_pos = traci.junction.getPosition("J2")
    
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], junction_pos[0], junction_pos[1])
        time_to_junction = distance_to_junction / speed if speed > 0 else float('inf')
        vehicle_props[veh] = {"pos": pos, "speed": speed, "distance_to_junction": distance_to_junction, "time_to_junction": time_to_junction}

    sorted_vehicles = sorted(vehicle_props, key=lambda v: vehicle_props[v]["time_to_junction"])

    for veh in sorted_vehicles:
        predicted_states = np.zeros((horizon, 2))
        predicted_states[0] = [vehicle_props[veh]["distance_to_junction"], vehicle_props[veh]["speed"]]

        for t in range(1, horizon):
            predicted_states[t, 0] = predicted_states[t-1, 0] - dt * predicted_states[t-1, 1]
            predicted_states[t, 1] = predicted_states[t-1, 1]
        
        for other_veh in sorted_vehicles:
            if veh == other_veh:
                continue
            for t in range(horizon):
                collision_time = predict_collision_time(predicted_states[t, 0], predicted_states[t, 1], vehicle_props[other_veh]["distance_to_junction"], vehicle_props[other_veh]["speed"])
                if abs(collision_time) < safety_margin:
                    new_speed = max(MIN_SPEED, predicted_states[t, 1] - 0.5 * (predicted_states[t, 1] - vehicle_props[other_veh]["speed"]))
                    traci.vehicle.slowDown(veh, new_speed, int(dt * 1000))
                    break
            else:
                traci.vehicle.slowDown(veh, MAX_SPEED, int(dt * 1000))

def run_simulation(params):
    horizon, dt, safety_margin = params
    horizon = max(int(round(horizon)), 1)  # Ensure horizon is at least 1 and round to the nearest integer
    dt = max(float(dt), 0.01)  # Ensure dt is positive
    safety_margin = max(float(safety_margin), 0.1)  # Ensure safety margin is positive
    
    options = get_options()
    sumoBinary = checkBinary("sumo")
    traci.start([sumoBinary, '-c', 'tjunction_02.sumocfg', "--tripinfo-output", "tripinfo.xml"])
    
    step = 0
    total_collisions = 0
    total_travel_time = 0
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()
        
        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
        
        if vehicles:
            calculate_speed_adjustments(vehicles, horizon=horizon, dt=dt, safety_margin=safety_margin)

        for vehicle_id in vehicles:
            speed = traci.vehicle.getSpeed(vehicle_id)
            if speed == 0:
                total_collisions += 1

        step += 1

    traci.close()
    
    total_travel_time = step  # Assuming each step is a fixed time interval

    return total_travel_time + total_collisions * 100  # Combining both metrics

def evaluate(individual):
    return (run_simulation(individual),)

def main(): 
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    toolbox.register("attr_int", np.random.randint, 1, 100)  # for horizon
    toolbox.register("attr_float", np.random.uniform, 0.01, 1.0)  # for dt
    toolbox.register("attr_margin", np.random.uniform, 1, 10)  # for safety_margin
    
    toolbox.register("individual", tools.initCycle, creator.Individual,
                     (toolbox.attr_int, toolbox.attr_float, toolbox.attr_margin), n=1)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)

    toolbox.register("mate", tools.cxBlend, alpha=0.5)
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.2)
    toolbox.register("select", tools.selTournament, tournsize=3)
    toolbox.register("evaluate", evaluate)

    population = toolbox.population(n=50)
    ngen = 40
    cxpb = 0.5
    mutpb = 0.2

    algorithms.eaSimple(population, toolbox, cxpb, mutpb, ngen, stats=None, halloffame=None, verbose=True)

    best_ind = tools.selBest(population, 1)[0]
    print('Best individual:', best_ind)
    print('Fitness:', best_ind.fitness.values)

if __name__ == "__main__":
    main()
