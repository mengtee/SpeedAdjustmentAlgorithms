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
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def predict_collision_time_2d(pos1, speed1, pos2, speed2):
    rel_pos_x = pos2[0] - pos1[0]
    rel_pos_y = pos2[1] - pos1[1]
    rel_speed_x = speed2[0] - speed1[0]
    rel_speed_y = speed2[1] - speed1[1]

    if rel_speed_x == 0 and rel_speed_y == 0:
        return float('inf')
    
    t_x = float('inf') if rel_speed_x == 0 else rel_pos_x / rel_speed_x
    t_y = float('inf') if rel_speed_y == 0 else rel_pos_y / rel_speed_y

    if t_x >= 0 and t_y >= 0:
        return min(t_x, t_y)
    elif t_x >= 0:
        return t_y
    elif t_y >= 0:
        return t_y

    return float('inf')

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

# Comparing the future state of the vehicle itself with the current state of the vehicles
def calculate_speed_adjustments(vehicle_id, horizon, dt, max_speed, min_speed, safety_margin):

    # Information for the vehicle (pos, speed)
    pos = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    angle = traci.vehicle.getAngle(vehicle_id)
    angle_rad = (90 - angle) * math.pi / 180.0
    speed_x = speed * math.cos(angle_rad)
    speed_y = speed * math.sin(angle_rad)
    distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])

    predicted_states = np.zeros((horizon, 4))
    predicted_states[0, 0] = pos[0]
    predicted_states[0, 1] = pos[1]
    predicted_states[0, 2] = speed_x
    predicted_states[0, 3] = speed_y

    # Predict the future state of the vehicle
    for t in range(1, horizon):
        predicted_states[t, 0] = predicted_states[t-1, 0] + predicted_states[t-1, 2] 
        predicted_states[t, 1] = predicted_states[t-1, 1] + predicted_states[t-1, 3] 
        predicted_states[t, 2] = predicted_states[t-1, 2]
        predicted_states[t, 3] = predicted_states[t-1, 3]

    # Predict 
    vehicles = traci.vehicle.getIDList()
    for t in range(1, horizon):
        for other_vehicle in vehicles:
            if other_vehicle == vehicle_id:
                continue
            other_pos = traci.vehicle.getPosition(other_vehicle)
            other_speed = traci.vehicle.getSpeed(other_vehicle)
            other_angle = traci.vehicle.getAngle(other_vehicle)
            other_angle_rad = (90 - other_angle) * math.pi / 180.0
            other_speed_x = other_speed * math.cos(other_angle_rad)
            other_speed_y = other_speed * math.sin(other_angle_rad)

            # Comparing every states within the horizon with the actual state of the vehicles
            collision_time = predict_collision_time_2d(
                (predicted_states[t-1, 0], predicted_states[t-1, 1]), 
                (predicted_states[t-1, 2], predicted_states[t-1, 3]), 
                (other_pos[0], other_pos[1]), 
                (other_speed_x, other_speed_y)
            )
            if abs(collision_time) < safety_margin:
                new_speed_x = max(min_speed, predicted_states[t-1, 2] - 0.5 * (predicted_states[t-1, 2] - other_speed_x))
                new_speed_y = max(min_speed, predicted_states[t-1, 3] - 0.5 * (predicted_states[t-1, 3] - other_speed_y))
                new_speed = math.sqrt(new_speed_x**2 + new_speed_y**2)
                traci.vehicle.slowDown(vehicle_id, new_speed, int(dt * 100))
                print(f'Collision detected: slowing down veh {vehicle_id} to speed {new_speed}')
                return
        current_speed = traci.vehicle.getSpeed(vehicle_id)
        if current_speed < max_speed:
            traci.vehicle.setSpeed(vehicle_id, max_speed)

def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()

        # For each of the vehicles, set the speed mode aside from the default speed mode and calculate their own respective speed adjustment
        for vehicle_id in vehicles:
            # print(f"This is the vehicle {vehicle_id}")
            set_vehicle_speed_mode(vehicle_id)
            calculate_speed_adjustments(vehicle_id, horizon=5, dt=1, max_speed=15, min_speed=2, safety_margin=2)

        step += 1

    traci.close()
    sys.stdout.flush()

if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    traci.start([sumoBinary, '-c', 'crossroad.sumocfg', "--tripinfo-output", "tripinfor.xml"])
    run()
