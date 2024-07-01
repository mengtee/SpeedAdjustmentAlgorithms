'''Genetic algorithms testing: GA implementing on each of the vehicles 
Currently able to implement, but this one only implemeting the GA for each of the vehicle
while running into some problem in the MPC model, since only 1 vehicle is taking into consideration'''
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

from deap import base, creator, tools, algorithms

# Define the fitness function and individual
creator.create("FitnessMax", base.Fitness, weights=(1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)

# Define the genetic algorithm operations
toolbox = base.Toolbox()
toolbox.register("attr_float", np.random.uniform, -1, 1)
toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=3)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Define evaluation function
def evaluate(individual):
    horizon, dt, safety_margin = individual
    fitness = 0.0
    # Run SUMO simulation here with the parameters and calculate fitness
    # For simplicity, assume a dummy fitness calculation
    fitness = 1.0 / (1.0 + abs(horizon - 30) + abs(dt - 1.366) + abs(safety_margin - 0.915))
    return fitness,

toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)
toolbox.register("evaluate", evaluate)

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

def calculate_distance(x1, y1, x2, y2):
    # Return all the positive values
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def predict_collision_time(distance1, speed1, distance2, speed2):
    if speed1 == 0 or speed2 == 0:
        return float('inf')
    return (distance1 / speed1) - (distance2 / speed2)

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_speed_adjustments(vehicles, horizon, dt, max_speed, min_speed, safety_margin):
    """Calculate necessary speed adjustments to avoid collisions at the junction."""
    vehicle_props = {}
    for veh in vehicles: # Gather vehicles properties and store in vehicle_props
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])
        time_to_junction = distance_to_junction / speed if speed > 0 else float('inf')
        vehicle_props[veh] = {"pos": pos, "speed": speed, "distance_to_junction": distance_to_junction, "time_to_junction": time_to_junction}

    print(f"Vehicles props {vehicle_props}")

    priorities = {}
    for veh in vehicle_props: # Prioritise vehicles that reaching the junction first
        priority = -vehicle_props[veh]["distance_to_junction"]
        priorities[veh] = priority

    sorted_vehicles = sorted(priorities, key=priorities.get, reverse=True)

    print(f'This is the sorted vehicles list {sorted_vehicles}')

    predicted_states = np.zeros((horizon, len(sorted_vehicles), 2)) #
    
    # Storing the position and the speed of the vehicle in a 3D array with the horizon
    for i, veh in enumerate(sorted_vehicles):
        print(f"Sorted vehicles in loop {sorted_vehicles}")
        for t in range(horizon):
            if t == 0: # Initialize the current position and speed
                predicted_states[t, i, 0] = vehicle_props[veh]["pos"][0]
                predicted_states[t, i, 1] = vehicle_props[veh]["speed"]
            else: # Predict future position using and stored in the predicted states for each horizon
                predicted_states[t, i, 0] = predicted_states[t-1, i, 0] + dt * predicted_states[t-1, i, 1]
                predicted_states[t, i, 1] = predicted_states[t-1, i, 1]

    # Adjusting the speed to avoid collision
    for i, veh in enumerate(sorted_vehicles):
        for t in range(horizon):
            if t > 0:
                for j in range(i): # For each vehicles i, compare with the states of higher priority vehicles j
                    # predict collision time = (distance1, speed1, distance2, speed2)
                    collision_time = predict_collision_time(predicted_states[t-1, i, 0], predicted_states[t-1, i, 1], predicted_states[t-1, j, 0], predicted_states[t-1, j, 1])
                    if abs(collision_time) < safety_margin:
                        new_speed = max(min_speed, predicted_states[t-1, i, 1] - 0.5 * (predicted_states[t-1, i, 1] - predicted_states[t-1, j, 1]))
                        traci.vehicle.slowDown(veh, new_speed, 1)
                        # print(f"Vehicle id: {veh}, new speed: {new_speed}")
                        break
                else:
                    traci.vehicle.slowDown(veh, max_speed, 1)
                    # print(f"Vehicle id: {veh}, max speed: {max_speed}")

    # np.set_printoptions(threshold=np.inf)  
    # print(f"Predicted states: {predicted_states}")

    return

def initialize_ga_for_vehicles(vehicles): # Initialized the GA population for each of the vehicles
    # Dictionary is matching (vehicle id -> population)
    ga_population = {}
    for veh in vehicles:
        pop = toolbox.population(n=50)
        ga_population[veh] = pop
    return ga_population

def update_ga_population(ga_population, vehicles): # update vehicles
    for veh in vehicles:
        if veh not in ga_population:
            pop = toolbox.population(n=50)
            ga_population[veh] = pop
    return ga_population

def run():
    """Main simulation loop."""
    step = 0
    ga_population = {}
    ga_generations = 100
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList() # Get the vehicles ID list in the simulation
        
        if step == 0: # Initialise the dictionary vehicles id -> populations
            ga_population = initialize_ga_for_vehicles(vehicles)
        else:
            ga_population = update_ga_population(ga_population, vehicles)

        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
        
        if len(vehicles) > 0:
            for vehicle_id in vehicles:
                pop = ga_population[vehicle_id] # Retrieve the GA population for the vehicles
                for gen in range(ga_generations): #Iterate over the generation
                    offspring = toolbox.select(pop, len(pop)) # Select offspring from current population
                    offspring = list(map(toolbox.clone, offspring)) #Clone offspring
                    
                    # Iterate over offspring for crossover
                    for child1, child2 in zip(offspring[::2], offspring[1::2]):
                        if np.random.rand() < 0.5:
                            toolbox.mate(child1, child2)
                            del child1.fitness.values
                            del child2.fitness.values
                    
                    # Iterate over offspring for mutation
                    for mutant in offspring:
                        if np.random.rand() < 0.2:
                            toolbox.mutate(mutant)
                            del mutant.fitness.values
                    
                    invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
                    fitnesses = map(toolbox.evaluate, invalid_ind)
                    for ind, fit in zip(invalid_ind, fitnesses):
                        ind.fitness.values = fit
                    
                    # Select the best individual to form new population
                    pop[:] = tools.selBest(offspring + pop, len(pop))
                
                best_ind = tools.selBest(pop, 1)[0]
                horizon, dt, safety_margin = best_ind # Extract the best parameters
                calculate_speed_adjustments([vehicle_id], int(horizon), float(dt), 15, 2, float(safety_margin))

        # Print out the speed of each vehicle
        print(f"Step {step}:")
        for vehicle_id in vehicles:
            speed = traci.vehicle.getSpeed(vehicle_id)
            # print(f"The Vehicle {vehicle_id}: Speed {speed:.2f} m/s")
        
        step += 1

    traci.close()
    sys.stdout.flush()

if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    traci.start([sumoBinary, '-c', 'tjunction_02.sumocfg', "--tripinfo-output", "tripinfo.xml"])
    run()
