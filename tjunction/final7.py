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

def calculate_time_to_junction(pos, speed, heading, junction_pos):
    distance_to_junction = calculate_distance(pos[0], pos[1], junction_pos[0], junction_pos[1])
    if speed == 0:
        return float('inf')
    return distance_to_junction / speed

def predict_collision_bicycle(pos1, speed1, heading1, pos2, speed2, heading2, vehicle_length, vehicle_width):
    rel_pos_x = pos2[0] - pos1[0]
    rel_pos_y = pos2[1] - pos1[1]
    rel_speed_x = speed2 * math.cos(heading2) - speed1 * math.cos(heading1)
    rel_speed_y = speed2 * math.sin(heading2) - speed1 * math.sin(heading1)

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

def get_lane_direction(lane_id):
    shape = traci.lane.getShape(lane_id)
    start_point = shape[0]
    end_point = shape[-1]
    
    dx = abs(end_point[0] - start_point[0])
    dy = abs(end_point[1] - start_point[1])
    
    return 'x' if dx > dy else 'y'

def has_passed_junction(pos, junction_pos, lane_id):
    direction = get_lane_direction(lane_id)
    if direction == 'x':
        return pos[0] > junction_pos[0]
    else:
        return pos[1] > junction_pos[1]
    
def calculate_speed_adjustments(vehicle_id, horizon, dt, max_speed, min_speed, safety_margin, junction_pos):
    pos = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    heading = traci.vehicle.getAngle(vehicle_id) * math.pi / 180.0
    length = traci.vehicle.getLength(vehicle_id)
    width = traci.vehicle.getWidth(vehicle_id)

    time_to_junction = calculate_time_to_junction(pos, speed, heading, junction_pos)

    predicted_states = broadcast_future_states(vehicle_id, pos, speed, heading, horizon, dt)

    vehicles = traci.vehicle.getIDList()
    nearest_collision_time = float('inf')
    nearest_collision_speed = max_speed

    lane_id = traci.vehicle.getLaneID(vehicle_id)
    vehicle_has_passed_junction = has_passed_junction(pos, junction_pos, lane_id)

    for t in range(1, horizon):
        for other_vehicle in vehicles:
            if other_vehicle == vehicle_id:
                continue

            other_predicted_states, other_time_to_junction = receive_predicted_states(other_vehicle)
            if other_predicted_states is None:
                continue

            other_length = traci.vehicle.getLength(other_vehicle)
            other_width = traci.vehicle.getWidth(other_vehicle)

            if not vehicle_has_passed_junction and time_to_junction > other_time_to_junction:
                collision_time = predict_collision_bicycle(
                    (predicted_states[t-1, 0], predicted_states[t-1, 1]), 
                    predicted_states[t-1, 2], predicted_states[t-1, 3],
                    (other_predicted_states[t-1, 0], other_predicted_states[t-1, 1]), 
                    other_predicted_states[t-1, 2], other_predicted_states[t-1, 3],
                    (length + other_length) / 2, 
                    (width + other_width) / 2
                )

                if abs(collision_time) < safety_margin and collision_time < nearest_collision_time:
                    nearest_collision_time = collision_time
                    nearest_collision_speed = adjust_speed(predicted_states[t-1], other_predicted_states[t-1], min_speed, max_speed)
                    speed = nearest_collision_speed
                    predicted_states = broadcast_future_states(vehicle_id, pos, speed, heading, horizon, dt)

            elif not vehicle_has_passed_junction and time_to_junction == other_time_to_junction:
                if vehicle_id > other_vehicle:
                    collision_time = predict_collision_bicycle(
                        (predicted_states[t-1, 0], predicted_states[t-1, 1]), 
                        predicted_states[t-1, 2], predicted_states[t-1, 3],
                        (other_predicted_states[t-1, 0], other_predicted_states[t-1, 1]), 
                        other_predicted_states[t-1, 2], other_predicted_states[t-1, 3],
                        (length + other_length) / 2, 
                        (width + other_width) / 2
                    )

                    if abs(collision_time) < safety_margin and collision_time < nearest_collision_time:
                        nearest_collision_time = collision_time
                        nearest_collision_speed = adjust_speed(predicted_states[t-1], other_predicted_states[t-1], min_speed, max_speed)
                        speed = nearest_collision_speed
                        predicted_states = broadcast_future_states(vehicle_id, pos, speed, heading, horizon, dt)

    if nearest_collision_time < float('inf'):
        traci.vehicle.setSpeed(vehicle_id, nearest_collision_speed)
    else:
        current_speed = traci.vehicle.getSpeed(vehicle_id)
        if current_speed < max_speed:
            traci.vehicle.setSpeed(vehicle_id, max_speed)

def broadcast_future_states(vehicle_id, pos, speed, heading, horizon, dt):
    predicted_states = np.zeros((horizon, 4))
    predicted_states[0, :] = [pos[0], pos[1], speed, heading]

    for t in range(1, horizon):
        predicted_states[t, 0] = predicted_states[t-1, 0] + predicted_states[t-1, 2] * math.cos(predicted_states[t-1, 3]) * dt
        predicted_states[t, 1] = predicted_states[t-1, 1] + predicted_states[t-1, 2] * math.sin(predicted_states[t-1, 3]) * dt
        predicted_states[t, 2] = predicted_states[t-1, 2]
        predicted_states[t, 3] = predicted_states[t-1, 3]

    traci.vehicle.setParameter(vehicle_id, "predictedStates", str(predicted_states.tolist()))
    return predicted_states

def receive_predicted_states(other_vehicle):
    try:
        states_str = traci.vehicle.getParameter(other_vehicle, "predictedStates")
        if states_str:
            states_str = states_str.replace('inf', 'float("inf")')
            predicted_states = np.array(eval(states_str))
            time_to_junction = predicted_states[0, 2] # Ensure we're accessing the correct index
            return predicted_states, time_to_junction
    except Exception as e:
        print(f'Error parsing predicted states for vehicle {other_vehicle}: {e}')
    return None, None

def adjust_speed(state1, state2, min_speed, max_speed):
    speed1 = state1[2]
    speed2 = state2[2]
    new_speed = max(min_speed, speed1 - 0.5 * (speed1 - speed2))
    return min(new_speed, max_speed)

def run():
    step = 0
    junction_pos = traci.junction.getPosition("J2")
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()

        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
            calculate_speed_adjustments(vehicle_id, horizon=15, dt=1, max_speed=15, min_speed=2, safety_margin=2, junction_pos=junction_pos)

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