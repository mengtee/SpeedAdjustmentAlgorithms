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
    """Calculate Euclidean distance between two points in 2D."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def predict_collision_time_2d(pos1, speed1, pos2, speed2):
    """
    Predict time to collision in 2D space based on relative positions and speeds.
    
    Parameters:
    pos1, pos2: Tuples representing (x, y) positions of the two vehicles.
    speed1, speed2: Tuples representing (vx, vy) velocities of the two vehicles.
    
    Returns:
    Time to collision if the vehicles are on a collision course, otherwise float('inf').
    """
    rel_pos = (pos2[0] - pos1[0], pos2[1] - pos1[1])
    rel_speed = (speed2[0] - speed1[0], speed2[1] - speed1[1])
    
    print(f"Relative Position: {rel_pos}, Relative Speed: {rel_speed}")
    
    a = rel_speed[0]**2 + rel_speed[1]**2
    b = 2 * (rel_pos[0] * rel_speed[0] + rel_pos[1] * rel_speed[1])
    c = rel_pos[0]**2 + rel_pos[1]**2
    
    print(f"a: {a}, b: {b}, c: {c}")
    
    if a == 0:  # No relative motion
        return float('inf')
    
    discriminant = b**2 - 4*a*c
    print(f"Discriminant: {discriminant}")
    if discriminant < 0:  # No real roots, no collision
        return float('inf')
    
    sqrt_discriminant = math.sqrt(discriminant)
    t1 = (-b - sqrt_discriminant) / (2 * a)
    t2 = (-b + sqrt_discriminant) / (2 * a)
    
    print(f"t1: {t1}, t2: {t2}")
    
    if t1 > 0 and t2 > 0:
        return min(t1, t2)
    elif t1 > 0:
        return t1
    elif t2 > 0:
        return t2
    else:
        return float('inf')  # Both times are negative, no future collision

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

    traci.start([sumoBinary, '-c', 'tjunction_03.sumocfg', "--tripinfo-output", "tripinfo.xml"])
    run()
