import optparse
import os
import sys
import math

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

def predict_collision_time(distance1, speed1, distance2, speed2):
    if speed1 == 0 or speed2 == 0:
        return float('inf')  # or some other suitable value
    return (distance1 / speed1) - (distance2 / speed2)

def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

def calculate_speed_adjustments(vehicles):
    # Create a dictionary to store vehicle properties
    vehicle_props = {}
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])
        time_to_junction = distance_to_junction / speed if speed > 0 else float('inf')
        vehicle_props[veh] = {"pos": pos, "speed": speed, "distance_to_junction": distance_to_junction, "time_to_junction": time_to_junction}

    # Sort vehicles by time to junction
    sorted_vehicles = sorted(vehicle_props, key=lambda x: vehicle_props[x]["time_to_junction"])

    # Iterate over sorted vehicles and adjust speeds
    for i, veh in enumerate(sorted_vehicles):
        for j in range(i + 1, len(sorted_vehicles)):
            other_veh = sorted_vehicles[j]
            if vehicle_props[veh]["speed"] == 0 or vehicle_props[other_veh]["speed"] == 0:
                continue
            collision_time = predict_collision_time(vehicle_props[veh]["distance_to_junction"], vehicle_props[veh]["speed"], vehicle_props[other_veh]["distance_to_junction"], vehicle_props[other_veh]["speed"])
            if abs(collision_time) < 2:  # adjust this value to change the safety margin
                # Adjust speed of one of the vehicles to avoid collision
                if vehicle_props[veh]["time_to_junction"] < vehicle_props[other_veh]["time_to_junction"]:
                    traci.vehicle.slowDown(veh, vehicle_props[veh]["speed"] * 0.3, 5)
                else:
                    traci.vehicle.slowDown(other_veh, vehicle_props[other_veh]["speed"] * 0.3, 5)
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

    traci.start([sumoBinary, '-c', 'multiple_vehicles_tjunction.sumocfg', "--tripinfo-output", "tripinfor.xml"])
    run()