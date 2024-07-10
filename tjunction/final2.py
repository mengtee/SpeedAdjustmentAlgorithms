'''Self development, after the junction will not stop'''
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
    if speed_magnitude_squared == 0:
        return float('inf')
    
    time_to_collision = distance_squared / speed_magnitude_squared
    return time_to_collision

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

def has_passed_junction(pos, junction_pos, lane_id, margin=5):
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
    if direction == 'x':
        return pos[0] > junction_pos[0] 
    else:
        return pos[1] > junction_pos[1]

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

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

    # Broadcast and store the information locally
    predicted_states = broadcast_future_states(vehicle_id, pos, speed_x, speed_y, horizon, dt)

    # Get the current lane ID
    lane_id = traci.vehicle.getLaneID(vehicle_id)

    # Determine if the vehicle has passed the junction
    vehicle_has_passed_junction = has_passed_junction(pos, junction_pos, lane_id)

    # Current position and stage/ and predicted state
    print(f'Current state of vehicle {vehicle_id}, pos: {pos}, speed: {speed}')
    print(f'Predicted state of vehicle {predicted_states}')
    print(f'The vehicle {vehicle_id} has passed junction: {vehicle_has_passed_junction}')

    vehicles = traci.vehicle.getIDList()
    for t in range(1, horizon):
        for other_vehicle in vehicles:
            if other_vehicle == vehicle_id:
                continue

            other_predicted_states = receive_predicted_states(other_vehicle)
            if other_predicted_states is None:
                continue

            other_pos = other_predicted_states[t-1, 0:2]
            other_speed_x = other_predicted_states[t-1, 2]
            other_speed_y = other_predicted_states[t-1, 3]
            other_length = traci.vehicle.getLength(other_vehicle)
            other_width = traci.vehicle.getWidth(other_vehicle)
            other_lane_id = traci.vehicle.getLaneID(other_vehicle)

            # Calculate time to reach the junction for the other vehicle
            other_time_to_junction = calculate_time_to_junction(other_pos, other_speed_x, other_speed_y, junction_pos)
            other_has_passed_junction = has_passed_junction(other_pos, junction_pos, other_lane_id)

            # Determine if the time to junction for this vehicle is longer than the other vehicles
            if not vehicle_has_passed_junction and time_to_junction > other_time_to_junction:
                # Current vehicle reaches the junction later, so it should adjust its speed
                collision_time = predict_collision_elliptical(
                    (predicted_states[t-1, 0], predicted_states[t-1, 1]), 
                    (predicted_states[t-1, 2], predicted_states[t-1, 3]), 
                    (other_predicted_states[t-1, 0], other_predicted_states[t-1, 1]), 
                    (other_predicted_states[t-1, 2], other_predicted_states[t-1, 3]),
                    (length + other_length) / 2, 
                    (width + other_width) / 2
                )

                print(t)
                if abs(collision_time) < safety_margin:
                    new_speed = adjust_speed(predicted_states[t-1], other_predicted_states[t-1], min_speed, max_speed)
                    # traci.vehicle.setSpeed(vehicle_id, new_speed)
                    traci.vehicle.slowDown(vehicle_id, new_speed, int(dt * 100))

                    print(f'Collision detected 1: setting speed of vehicle {vehicle_id} to {new_speed}')
                    return
                
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

                    if abs(collision_time) < safety_margin:
                        new_speed = adjust_speed(predicted_states[t-1], other_predicted_states[t-1], min_speed, max_speed)
                        traci.vehicle.slowDown(vehicle_id, new_speed, int(dt * 100))

                        # traci.vehicle.setSpeed(vehicle_id, new_speed)
                        print(f'Collision detected 2: setting speed of vehicle {vehicle_id} to {new_speed}')
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

    # Store the predicted states for the vehicles
    traci.vehicle.setParameter(vehicle_id, "predictedStates", str(predicted_states.tolist()))
    return predicted_states

def receive_predicted_states(other_vehicle):
    try:
        states_str = traci.vehicle.getParameter(other_vehicle, "predictedStates")
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
    junction_pos = traci.junction.getPosition("J2")  # Replace "junctionID" with your junction ID
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()

        for vehicle_id in vehicles:
            print(f'This is the step {step}')
            set_vehicle_speed_mode(vehicle_id)
            calculate_speed_adjustments(vehicle_id, horizon=10, dt=1, max_speed=15, min_speed=2, safety_margin=2, junction_pos=junction_pos)

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


'''Performance
tjunction_02: 4 vehicles collision, with 46 steps
tjunction_03: 1 vehicles collision, with 10 steps'''
