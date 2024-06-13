import optparse
import os
import sys
import math
import numpy as np
from scipy.optimize import minimize

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

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def predict_collision_time(vehicle1_pos, vehicle1_speed, vehicle2_pos, vehicle2_speed):

    # Will not collision if two of the vehicle moving in same speed without moving on the collision course 
    if vehicle1_speed == vehicle2_speed:
        return float('inf')  # or some other suitable value
    ttc = (vehicle1_pos - vehicle2_pos) / (vehicle1_speed - vehicle2_speed)
    return ttc

# Set the vehicle speed mode aside from the default speed adjustment algorithms
def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_speed_adjustments(vehicles):
    # Create a dictionary to store vehicle properties (position, speed, distance and time to junction)
    vehicle_props = {}
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])
        time_to_junction = distance_to_junction / speed if speed > 0 else float('inf')
        vehicle_props[veh] = {"pos": pos, "speed": speed, "distance_to_junction": distance_to_junction, "time_to_junction": time_to_junction}

    # Assign priorities to vehicles based on proximity to junction and speed (Prioritise vehicle that closer to junction)
    priorities = {}
    for veh in vehicle_props:
        # 
        priority = 1 / (vehicle_props[veh]["distance_to_junction"] + 0.1 * vehicle_props[veh]["speed"])
        priorities[veh] = priority

    # Sort vehicles by priority
    sorted_vehicles = sorted(priorities, key=priorities.get, reverse=True)

    # Initialize MPC parameters
    horizon = 10  # prediction horizon (steps)
    dt = 1  # time step (seconds)
    max_speed = 15  # maximum speed (m/s)
    min_speed = 5  # minimum speed (m/s)
    safety_margin = 2  # safety margin (seconds)

    # Create a matrix to store the predicted states
    predicted_states = np.zeros((horizon, len(sorted_vehicles), 2))  # [time, vehicle, state (pos, speed)]

    # Iterate over the sorted vehicles and predict their future states
    for i, veh in enumerate(sorted_vehicles):
        for t in range(horizon):

            # Predicting the state (next pos= current pos + (time step * speed))
            if t == 0:
                predicted_states[t, i, 0] = vehicle_props[veh]["pos"][0]
                predicted_states[t, i, 1] = vehicle_props[veh]["speed"]
            else:
                predicted_states[t, i, 0] = predicted_states[t-1, i, 0] + dt * predicted_states[t-1, i, 1]
                predicted_states[t, i, 1] = predicted_states[t-1, i, 1]

    # Optimize the speed adjustments using MPC
    for i, veh in enumerate(sorted_vehicles):
        for t in range(horizon):
            if t > 0:
                # Calculate the predicted collision time with the previous vehicle
                collision_time = predict_collision_time(predicted_states[t-1, i, 0], predicted_states[t-1, i, 1], predicted_states[t-1, i-1, 0], predicted_states[t-1, i-1, 1])
                if abs(collision_time) < safety_margin:
                    # Adjust the speed of the current vehicle to avoid collision
                    new_speed = max(min_speed, predicted_states[t-1, i, 1] - 0.5 * (predicted_states[t-1, i, 1] - predicted_states[t-1, i-1, 1]))
                    print('Slowing down')
                    traci.vehicle.slowDown(veh, new_speed, 5)

    return

def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print(f"Simulation step: {step}")

        vehicles = traci.vehicle.getIDList()
        print(f"Vehicles in simulation: {vehicles}")
        
        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
        
        if len(vehicles) > 0:
            print("Successfully run")
            calculate_speed_adjustments(vehicles)
        else:
            print("No vehicles in simulation.")
        
        step += 1

    traci.close()
    sys.stdout.flush()

if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    traci.start([sumoBinary, '-c','multiple_vehicles_tjunction.sumocfg', "--tripinfo-output", "tripinfor.xml"])
    run()