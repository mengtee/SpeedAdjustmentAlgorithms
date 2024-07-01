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

def predict_collision_time_2d(pos1, speed1, pos2, speed2):
    """
    Predict time to collision in 2D space based on relative positions and speeds.
    
    Parameters:
    pos1, pos2: Tuples representing (x, y) positions of the two vehicles.
    speed1, speed2: Tuples representing (vx, vy) velocities of the two vehicles.
    
    Returns:
    Time to collision if the vehicles are on a collision course, otherwise float('inf').
    """
    rel_pos_x = abs(pos2[0]) - abs(pos1[0])
    rel_pos_y = abs(pos2[1]) - abs(pos1[1])
    rel_speed_x = abs(speed2[0]) - abs(speed1[0])
    rel_speed_y = abs(speed2[1]) - abs(speed1[1])

    print(f"Relative Position X: {rel_pos_x}, Relative Position Y: {rel_pos_y}")
    print(f"Relative Speed X: {rel_speed_x}, Relative Speed Y: {rel_speed_y}")
    
    if rel_speed_x == 0 and rel_speed_y == 0:
        print("No relative motion")
        return float('inf')
    
    # Calculate time to collision for x and y coordinates
    t_x = float('inf') if rel_speed_x == 0 else rel_pos_x / rel_speed_x
    t_y = float('inf') if rel_speed_y == 0 else rel_pos_y / rel_speed_y

    print(f"Initial t_x: {t_x}, Initial t_y: {t_y}")

    # Check if both times are positive and nearly equal
    if t_x >= 0 and t_y >= 0:
        print("Collision detected")
        return min(t_x, t_y)

    print("No collision detected")
    return float('inf')

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)


def calculate_speed_adjustments(vehicles, horizon, dt, max_speed, min_speed, safety_margin):
    """Calculate necessary speed adjustments to avoid collisions at the junction."""
    vehicle_props = {}
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        angle = traci.vehicle.getAngle(veh) * math.pi / 180.0  # Convert to radians
        speed_x = speed * math.cos(angle)
        speed_y = speed * math.sin(angle)
        print(f'Vehicles {veh}, speedx {speed_x}, speedy {speed_y}, pos {pos}')
        distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])
        vehicle_props[veh] = {"pos": pos, "speed": (speed_x, speed_y), "distance_to_junction": distance_to_junction}

    # Sort vehicles based on their distance to the junction
    priorities = {veh: -props["distance_to_junction"] for veh, props in vehicle_props.items()}
    sorted_vehicles = sorted(priorities, key=priorities.get, reverse=True)
    # print(f"This is the sorted vehicles {sorted_vehicles}")

    predicted_states = np.zeros((horizon, len(sorted_vehicles), 4))  # x, y, vx, vy

    for i, veh in enumerate(sorted_vehicles):
        for t in range(horizon):
            if t == 0:
                predicted_states[t, i, 0] = vehicle_props[veh]["pos"][0]
                predicted_states[t, i, 1] = vehicle_props[veh]["pos"][1]
                predicted_states[t, i, 2] = vehicle_props[veh]["speed"][0]
                predicted_states[t, i, 3] = vehicle_props[veh]["speed"][1]
            else:
                predicted_states[t, i, 0] = predicted_states[t-1, i, 0] + dt * predicted_states[t-1, i, 2]
                predicted_states[t, i, 1] = predicted_states[t-1, i, 1] + dt * predicted_states[t-1, i, 3]
                predicted_states[t, i, 2] = predicted_states[t-1, i, 2]
                predicted_states[t, i, 3] = predicted_states[t-1, i, 3]

    for i, veh_i in enumerate(sorted_vehicles):
        for t in range(1, horizon):
            for j, veh_j in enumerate(sorted_vehicles):
                if i == j:
                    continue
                collision_time = predict_collision_time_2d(
                    (predicted_states[t-1, i, 0], predicted_states[t-1, i, 1]), 
                    (predicted_states[t-1, i, 2], predicted_states[t-1, i, 3]), 
                    (predicted_states[t-1, j, 0], predicted_states[t-1, j, 1]), 
                    (predicted_states[t-1, j, 2], predicted_states[t-1, j, 3])
                )
                print(f"Collision time: {collision_time}")
                if abs(collision_time) < safety_margin:
                    new_speed_x = max(min_speed, predicted_states[t-1, i, 2] - 
                                      0.5 * (predicted_states[t-1, i, 2] - predicted_states[t-1, j, 2]))
                    new_speed_y = max(min_speed, predicted_states[t-1, i, 3] - 
                                      0.5 * (predicted_states[t-1, i, 3] - predicted_states[t-1, j, 3]))
                    new_speed = math.sqrt(new_speed_x**2 + new_speed_y**2)
                    traci.vehicle.slowDown(veh_i, new_speed, int(dt * 1000))
                    print(f"Collision detected: Vehicle {veh_i} adjusted speed to avoid Vehicle {veh_j}")
                    collision_detected = True
                    break
            else:
                # No collision detected, ensure max speed
                current_speed = traci.vehicle.getSpeed(veh_i)
                if current_speed < max_speed:
                    traci.vehicle.setSpeed(veh_i, max_speed)
                    print(f"No collision: Vehicle {veh_i} maintaining/increasing speed to {max_speed:.2f} m/s")
    return

def run():
    """Main simulation loop."""
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()
        
        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
        
        if len(vehicles) > 0:
            calculate_speed_adjustments(vehicles, horizon=30, dt=0.01, max_speed=15, min_speed=2, safety_margin=2)

        # Print out the speed of each vehicle
        print(f"Step {step}:")
        for vehicle_id in vehicles:
            speed = traci.vehicle.getSpeed(vehicle_id)
            print(f"The Vehicle {vehicle_id}: Speed {speed:.2f} m/s")
            
        step += 1

    traci.close()
    sys.stdout.flush()

if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    traci.start([sumoBinary, '-c', 'tjunction_02.sumocfg', "--tripinfo-output", "tripinfor.xml"])
    run()
