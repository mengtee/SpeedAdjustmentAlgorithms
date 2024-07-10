'''combination of final 2 and final 5
improve on the final5 problem, where the vehicles state is not updated after the speed adjustment
'''
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

def calculate_time_to_junction(pos, speed_x, speed_y, junction_pos):
    distance_to_junction = calculate_distance(pos[0], pos[1], junction_pos[0], junction_pos[1])
    speed_magnitude = math.sqrt(speed_x**2 + speed_y**2)
    if speed_magnitude == 0:
        return float('inf')
    return distance_to_junction / speed_magnitude

def predict_collision_elliptical(pos1, speed1, pos2, speed2, vehicle_length, vehicle_width):
    rel_pos_x = pos2[0] - pos1[0]
    rel_pos_y = pos2[1] - pos1[1]
    rel_speed_x = speed2[0] - speed1[0]
    rel_speed_y = speed2[1] - speed1[1]

    a = vehicle_length / 2.0
    b = vehicle_width / 2.0

    distance_squared = (rel_pos_x**2 / a**2) + (rel_pos_y**2 / b**2)
    speed_magnitude_squared = rel_speed_x**2 + rel_speed_y**2

    # SMALL_VALUE_THRESHOLD = 1e-3
    # speed1 = [0 if abs(s) < SMALL_VALUE_THRESHOLD else s for s in speed1]
    # speed2 = [0 if abs(s) < SMALL_VALUE_THRESHOLD else s for s in speed2]

    # Check if vehicles are moving in the same direction
    same_direction = (speed1[0] * speed2[0] > 0) or (speed1[1] * speed2[1] > 0)

    if same_direction:
        print("Same direction")
        if speed_magnitude_squared == 0 or speed1[0] <= speed2[0] and speed1[1] <= speed2[1]:
            return float('inf')  # No collision if the leading vehicle is faster or the same speed
        distance = math.sqrt(rel_pos_x**2 + rel_pos_y**2)
        speed_magnitude = math.sqrt(rel_speed_x**2 + rel_speed_y**2)
        return distance / abs(speed_magnitude)
    
    else:
        if speed_magnitude_squared == 0:
            return float('inf')
        time_to_collision = distance_squared / speed_magnitude_squared
        print(f'Not same direction with the ttc: {time_to_collision}')
        return time_to_collision

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def get_lane_direction(lane_id):
    """
    Get the primary direction of travel for a lane.
    
    Parameters:
    lane_id: str - ID of the lane
    
    Returns:
    str - 'x' if the primary direction is along the x-axis, 'y' if along the y-axis
    """
    shape = traci.lane.getShape(lane_id)
    start_point = shape[0]
    end_point = shape[-1]
    
    dx = abs(end_point[0] - start_point[0])
    dy = abs(end_point[1] - start_point[1])
    
    return 'x' if dx > dy else 'y'

def has_passed_junction(pos, junction_pos, lane_id):
    """
    Check if the vehicle has passed the junction.
    
    Parameters:
    pos: Tuple[float, float] - current position of the vehicle (x, y)
    junction_pos: Tuple[float, float] - position of the junction (x, y)
    lane_id: str - ID of the lane the vehicle is on
    margin: float - margin around the junction to account for all lanes
    
    Returns:
    bool - True if the vehicle has passed the junction, False otherwise
    """
    direction = get_lane_direction(lane_id) 
    # print(f'This is the lane direction {direction}')   
    if direction == 'x':
        return pos[0] > junction_pos[0] 
    else:
        return pos[1] > junction_pos[1]
    
def calculate_speed_adjustments(vehicle_id, horizon, dt, max_speed, min_speed, safety_margin, junction_pos):
    pos = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    angle = traci.vehicle.getAngle(vehicle_id)
    angle_rad = (90 - angle) * math.pi / 180.0
    speed_x = speed * math.cos(angle_rad)
    speed_y = speed * math.sin(angle_rad)
    length = traci.vehicle.getLength(vehicle_id)
    width = traci.vehicle.getWidth(vehicle_id)

    # Calculate time to reach the junction for the current vehicle
    time_to_junction = calculate_time_to_junction(pos, speed_x, speed_y, junction_pos)

    # Broadcast and store the information locally, including time to junction
    predicted_states = broadcast_future_states(vehicle_id, pos, speed_x, speed_y, time_to_junction, horizon, dt)

    vehicles = traci.vehicle.getIDList()
    nearest_collision_time = float('inf')
    nearest_collision_speed = max_speed

    lane_id = traci.vehicle.getLaneID(vehicle_id)
    # print(f'This is the vehicle {vehicle_id} with the lane {lane_id}')
    vehicle_has_passed_junction = has_passed_junction(pos, junction_pos, lane_id)

    print(f'-------------------------------{vehicle_id}-------------------------------')
    print(f'Current state of vehicle {vehicle_id}, pos: {pos}, speed: {speed}')
    print(f'Predicted state of vehicle {predicted_states}')

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
                # Current vehicle reaches the junction later, so it should adjust its speed
                print(f'Predict the collision with {vehicle_id} with {other_vehicle}')
                collision_time = predict_collision_elliptical(
                    (predicted_states[t-1, 0], predicted_states[t-1, 1]), 
                    (predicted_states[t-1, 2], predicted_states[t-1, 3]), 
                    (other_predicted_states[t-1, 0], other_predicted_states[t-1, 1]), 
                    (other_predicted_states[t-1, 2], other_predicted_states[t-1, 3]),
                    (length + other_length) / 2, 
                    (width + other_width) / 2
                )

                if abs(collision_time) < safety_margin and collision_time < nearest_collision_time:
                    print(f'Potential collision detected between {vehicle_id} and {other_vehicle} at time {collision_time}. {t} Adjusting speed.')

                    nearest_collision_time = collision_time
                    nearest_collision_speed = adjust_speed(predicted_states[t-1], other_predicted_states[t-1], min_speed, max_speed)
                    # Recalculate and broadcast future states after slowing down
                    speed_x = nearest_collision_speed * math.cos(angle_rad)
                    speed_y = nearest_collision_speed * math.sin(angle_rad)
                    predicted_states = broadcast_future_states(vehicle_id, pos, speed_x, speed_y, time_to_junction, horizon, dt)
                    print(f'Updated states for {vehicle_id}: {predicted_states}')     

            elif not vehicle_has_passed_junction and time_to_junction == other_time_to_junction:
                # If both vehicles reach the junction at the same time, use vehicle ID to break tie
                if vehicle_id > other_vehicle:
                    # Current vehicle has lower priority (higher ID), so it should adjust its speed
                    collision_time = predict_collision_elliptical(
                        (predicted_states[t-1, 0], predicted_states[t-1, 1]), 
                        (predicted_states[t-1, 2], predicted_states[t-1, 3]), 
                        (other_predicted_states[t-1, 0], other_predicted_states[t-1, 1]), 
                        (other_predicted_states[t-1, 2], other_predicted_states[t-1, 3]),
                        (length + other_length) / 2, 
                        (width + other_width) / 2
                    )

                    if abs(collision_time) < safety_margin and collision_time < nearest_collision_time:
                        print(f'Potential collision detected between {vehicle_id} and {other_vehicle} at time {collision_time}. Adjusting speed.')

                        nearest_collision_time = collision_time
                        nearest_collision_speed = adjust_speed(predicted_states[t-1], other_predicted_states[t-1], min_speed, max_speed)
                        # Recalculate and broadcast future states after slowing down
                        speed_x = nearest_collision_speed * math.cos(angle_rad)
                        speed_y = nearest_collision_speed * math.sin(angle_rad)
                        predicted_states = broadcast_future_states(vehicle_id, pos, speed_x, speed_y, time_to_junction, horizon, dt)

    if nearest_collision_time < float('inf'):
        traci.vehicle.setSpeed(vehicle_id, nearest_collision_speed)
        print(f'Collision detected: setting speed of vehicle {vehicle_id} to {nearest_collision_speed}')
    else:
        current_speed = traci.vehicle.getSpeed(vehicle_id)
        if current_speed < max_speed:
            traci.vehicle.setSpeed(vehicle_id, max_speed)
    
    print(f'----------------------------------------------------------------------------')


def broadcast_future_states(vehicle_id, pos, speed_x, speed_y, time_to_junction, horizon, dt):
    predicted_states = np.zeros((horizon, 5))  # Added an extra column for time to junction
    predicted_states[0, :] = [pos[0], pos[1], speed_x, speed_y, time_to_junction]

    for t in range(1, horizon):
        predicted_states[t, 0] = predicted_states[t-1, 0] + predicted_states[t-1, 2] * dt
        predicted_states[t, 1] = predicted_states[t-1, 1] + predicted_states[t-1, 3] * dt
        predicted_states[t, 2] = predicted_states[t-1, 2]
        predicted_states[t, 3] = predicted_states[t-1, 3]
        predicted_states[t, 4] = float('inf') if speed_x == 0 and speed_y == 0 else time_to_junction

    # Store the predicted states for the vehicles
    traci.vehicle.setParameter(vehicle_id, "predictedStates", str(predicted_states.tolist()))
    return predicted_states

def receive_predicted_states(other_vehicle):
    try:
        states_str = traci.vehicle.getParameter(other_vehicle, "predictedStates")
        if states_str:
            states_str = states_str.replace('inf', 'float("inf")')
            predicted_states = np.array(eval(states_str))
            distance_to_junction = predicted_states[0, 4]  # Extract the distance to junction
            return predicted_states, distance_to_junction
    except Exception as e:
        print(f'Error parsing predicted states for vehicle {other_vehicle}: {e}')
    return None, None

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
    junction_pos = traci.junction.getPosition("J2")  
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()

        print(f'==================== This is the number of step {step} =====================')

        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
            calculate_speed_adjustments(vehicle_id, horizon=10, dt=1, max_speed=15, min_speed=2, safety_margin=2, junction_pos=junction_pos)

        print(f'=============================================================================')
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
