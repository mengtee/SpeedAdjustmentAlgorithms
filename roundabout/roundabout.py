'''Used a priority system, Used the predicted future states of the vehicles
to determine if the speed need to be adjusted'''

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

def calculate_distance(x1, y1, x2, y2):
    # Return all the positive values
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def predict_collision_time(distance1, speed1, distance2, speed2):
    if speed1 == 0 or speed2 == 0:
        return float('inf')
    return (distance1 / speed1) - (distance2 / speed2)

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_speed_adjustments(vehicles):
    vehicle_props = {}
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])
        time_to_junction = distance_to_junction / speed if speed > 0 else float('inf') 
        vehicle_props[veh] = {"pos": pos, "speed": speed, "distance_to_junction": distance_to_junction, "time_to_junction": time_to_junction}

    priorities = {}
    for veh in vehicle_props: # Prioritise vehicles that reaching the junction first
        # priority = 1 / (vehicle_props[veh]["distance_to_junction"] + 0.1 * vehicle_props[veh]["speed"])
        priority = -vehicle_props[veh]["distance_to_junction"]
        priorities[veh] = priority

    sorted_vehicles = sorted(priorities, key=priorities.get, reverse=True)

    print(f'This is the sorted vehicles list {sorted_vehicles}')

    horizon = 20  # increased prediction horizon
    dt = 0.01
    max_speed = 15
    min_speed = 2  # reduced minimum speed
    safety_margin = 3  # increased safety margin

    predicted_states = np.zeros((horizon, len(sorted_vehicles), 2))

    for i, veh in enumerate(sorted_vehicles):
        for t in range(horizon):
            if t == 0:
                predicted_states[t, i, 0] = vehicle_props[veh]["pos"][0]
                predicted_states[t, i, 1] = vehicle_props[veh]["speed"]
            else:
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
                        print(f"Vehicle id: {veh}, new speed: {new_speed}")
                        break
                else:
                    traci.vehicle.slowDown(veh, max_speed, 5)
                    print(f"Vehicle id: {veh}, max speed: {max_speed}")
    return

def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        # vehicles = traci.vehicle.getIDList()
        
        # for vehicle_id in vehicles:
        #     set_vehicle_speed_mode(vehicle_id)
        
        # if len(vehicles) > 0:
        #     calculate_speed_adjustments(vehicles)

        # # Print out the speed of each vehicle
        # print(f"Step {step}:")
        # for vehicle_id in vehicles:
        #     speed = traci.vehicle.getSpeed(vehicle_id)
        #     print(f"The Vehicle {vehicle_id}: Speed {speed:.2f} m/s")
        
        
        step += 1

    traci.close()
    sys.stdout.flush()


if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    traci.start([sumoBinary, '-c', 'roundabout.sumocfg', "--tripinfo-output", "tripinfor.xml"])
    run()
