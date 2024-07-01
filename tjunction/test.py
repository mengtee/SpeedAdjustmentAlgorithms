import optparse
import os
import sys
import math
import numpy as np

# Ensure SUMO environment is set
os.environ['SUMO_HOME'] = '/Users/TEEMENGKIAT/sumo'

if 'SUMO_HOME' in os.environ:
    tools_path = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools_path)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

# Constants
MAX_SPEED = 15.0  # constant max speed
MIN_SPEED = 2.0   # constant min speed

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def predict_collision_time(distance1, speed1, distance2, speed2):
    if speed1 == 0 or speed2 == 0:
        return float('inf')
    return (distance1 / speed1) - (distance2 / speed2)

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_future_positions(vehicle_props, horizon, dt):
    predicted_states = {}
    for veh, props in vehicle_props.items():
        predicted_states[veh] = np.zeros((horizon, 3))  # (distance to junction, speed, lane)
        predicted_states[veh][0] = [props["distance_to_junction"], props["speed"], props["lane"]]
        for t in range(1, horizon):
            predicted_states[veh][t, 0] = predicted_states[veh][t-1, 0] - dt * predicted_states[veh][t-1, 1]
            predicted_states[veh][t, 1] = predicted_states[veh][t-1, 1]
            predicted_states[veh][t, 2] = predicted_states[veh][t-1, 2]
    return predicted_states

def calculate_speed_adjustments(vehicles, horizon=20, dt=0.01, safety_margin=3):
    horizon = max(int(round(horizon)), 1)  # Ensure horizon is at least 1 and round to the nearest integer
    dt = max(float(dt), 0.01)  # Ensure dt is positive
    
    print(f"Parameters: horizon={horizon}, dt={dt}, max_speed={MAX_SPEED}, min_speed={MIN_SPEED}, safety_margin={safety_margin}")
    
    vehicle_props = {}
    junction_pos = traci.junction.getPosition("J2")
    
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        lane = traci.vehicle.getLaneIndex(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], junction_pos[0], junction_pos[1])
        time_to_junction = distance_to_junction / speed if speed > 0 else float('inf')
        vehicle_props[veh] = {"pos": pos, "speed": speed, "distance_to_junction": distance_to_junction, "time_to_junction": time_to_junction, "lane": lane}
    
    predicted_states = calculate_future_positions(vehicle_props, horizon, dt)
    
    for veh in vehicles:
        current_speed = vehicle_props[veh]["speed"]
        for t in range(horizon):
            for other_veh in vehicles:
                if veh == other_veh:
                    continue
                if vehicle_props[veh]["lane"] == vehicle_props[other_veh]["lane"]:
                    collision_time = predict_collision_time(predicted_states[veh][t, 0], predicted_states[veh][t, 1], 
                                                            predicted_states[other_veh][t, 0], predicted_states[other_veh][t, 1])
                    if abs(collision_time) < safety_margin:
                        new_speed = max(MIN_SPEED, current_speed - 0.5 * (current_speed - vehicle_props[other_veh]["speed"]))
                        traci.vehicle.slowDown(veh, new_speed, int(dt * 1000))
                        break
                else:
                    # Consider potential lane changes and merging
                    collision_time = predict_collision_time(predicted_states[veh][t, 0], predicted_states[veh][t, 1], 
                                                            predicted_states[other_veh][t, 0], predicted_states[other_veh][t, 1])
                    if abs(collision_time) < safety_margin * 2:  # Use a larger margin for lane changes
                        new_speed = max(MIN_SPEED, current_speed - 0.5 * (current_speed - vehicle_props[other_veh]["speed"]))
                        traci.vehicle.slowDown(veh, new_speed, int(dt * 1000))
                        break
            else:
                traci.vehicle.slowDown(veh, MAX_SPEED, int(dt * 1000))

def run_simulation():
    horizon = 20
    dt = 0.01
    safety_margin = 3
    
    options = get_options()
    sumoBinary = checkBinary("sumo-gui") if not options.nogui else checkBinary("sumo")
    traci.start([sumoBinary, '-c', 'tjunction_01.sumocfg', "--tripinfo-output", "tripinfo.xml"])
    
    step = 0
    total_collisions = 0
    total_travel_time = 0
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        vehicles = traci.vehicle.getIDList()
        
        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
        
        if vehicles:
            calculate_speed_adjustments(vehicles, horizon=horizon, dt=dt, safety_margin=safety_margin)

        for vehicle_id in vehicles:
            speed = traci.vehicle.getSpeed(vehicle_id)
            if speed == 0:
                total_collisions += 1

        step += 1

    traci.close()
    
    total_travel_time = step  # Assuming each step is a fixed time interval

    print(f"Total travel time: {total_travel_time}")
    print(f"Total collisions: {total_collisions}")

if __name__ == "__main__":
    run_simulation()
