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

def predict_collision_elliptical(pos1, speed1, pos2, speed2, vehicle_length, vehicle_width):
    rel_pos_x = pos2[0] - pos1[0]
    rel_pos_y = pos2[1] - pos1[1]
    rel_speed_x = speed2[0] - speed1[0]
    rel_speed_y = speed2[1] - speed1[1]

    a = vehicle_length / 2.0
    b = vehicle_width / 2.0

    distance_squared = (rel_pos_x**2 / a**2) + (rel_pos_y**2 / b**2)
    speed_magnitude_squared = rel_speed_x**2 + rel_speed_y**2
    if speed_magnitude_squared == 0:
        return float('inf')
    
    time_to_collision = distance_squared / speed_magnitude_squared
    return time_to_collision

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_speed_adjustments(vehicle_id, horizon, dt, max_speed, min_speed, safety_margin):
    pos = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    angle = traci.vehicle.getAngle(vehicle_id)
    angle_rad = (90 - angle) * math.pi / 180.0
    speed_x = speed * math.cos(angle_rad)
    speed_y = speed * math.sin(angle_rad)
    length = traci.vehicle.getLength(vehicle_id)
    width = traci.vehicle.getWidth(vehicle_id)

    # This broadcast and store the information locally
    predicted_states = broadcast_future_states(vehicle_id, pos, speed_x, speed_y, horizon, dt)

    vehicles = traci.vehicle.getIDList()
    for t in range(1, horizon):
        for other_vehicle in vehicles:
            if other_vehicle == vehicle_id:
                continue

            other_predicted_states = receive_predicted_states(other_vehicle)
            if other_predicted_states is None:
                continue

            other_length = traci.vehicle.getLength(other_vehicle)
            other_width = traci.vehicle.getWidth(other_vehicle)

            # Compare the every current state and future state of the vehicle with other vehicles 
            collision_time = predict_collision_elliptical(
                (predicted_states[t-1, 0], predicted_states[t-1, 1]), 
                (predicted_states[t-1, 2], predicted_states[t-1, 3]), 
                (other_predicted_states[t-1, 0], other_predicted_states[t-1, 1]), 
                (other_predicted_states[t-1, 2], other_predicted_states[t-1, 3]),
                (length + other_length) / 2, 
                (width + other_width) / 2
            )

            if abs(collision_time) < safety_margin:
                new_speed = adjust_speed(predicted_states[t-1], other_predicted_states[t-1], min_speed, max_speed)
                traci.vehicle.slowDown(vehicle_id, new_speed, int(dt * 1))
                print(f'Collision detected: slowing down vehicle {vehicle_id} to speed {new_speed}')
                return

    current_speed = traci.vehicle.getSpeed(vehicle_id)
    if current_speed < max_speed:
        traci.vehicle.setSpeed(vehicle_id, max_speed)

def broadcast_future_states(vehicle_id, pos, speed_x, speed_y, horizon, dt):
    predicted_states = np.zeros((horizon, 4))
    predicted_states[0, :] = [pos[0], pos[1], speed_x, speed_y]

    for t in range(1, horizon):
        predicted_states[t, 0] = predicted_states[t-1, 0] + predicted_states[t-1, 2] * dt
        predicted_states[t, 1] = predicted_states[t-1, 1] + predicted_states[t-1, 3] * dt
        predicted_states[t, 2] = predicted_states[t-1, 2]
        predicted_states[t, 3] = predicted_states[t-1, 3]

    # Stored the predicted states for the vehicles
    traci.vehicle.setParameter(vehicle_id, "predictedStates", str(predicted_states.tolist()))
    return predicted_states

def receive_predicted_states(other_vehicle):
    try:
        states_str = traci.vehicle.getParameter(other_vehicle, "predictedStates")
        # print("Received predicted states")
        if states_str:
            return np.array(eval(states_str))
    except Exception as e:
        print(f'Error parsing predicted states for vehicle {other_vehicle}: {e}')
    return None

def adjust_speed(state1, state2, min_speed, max_speed):
    # Extract speeds in x and y directions
    speed1_x = state1[2]
    speed1_y = state1[3]
    speed2_x = state2[2]
    speed2_y = state2[3]
    
    # Adjust speeds separately for x and y directions
    new_speed_x = max(min_speed, speed1_x - 0.5 * (speed1_x - speed2_x))
    new_speed_y = max(min_speed, speed1_y - 0.5 * (speed1_y - speed2_y))
    
    # Combine the adjusted speeds
    new_speed = np.linalg.norm([new_speed_x, new_speed_y])
    print(f'This is the new speed {new_speed}')
    
    return min(new_speed, max_speed)

def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()

        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
            calculate_speed_adjustments(vehicle_id, horizon=10, dt=1, max_speed=15, min_speed=2, safety_margin=2)

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

'''Performance
tjunction_02: 9 collision vehicles, with 47 steps'''