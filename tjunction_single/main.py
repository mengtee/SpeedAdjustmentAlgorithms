'''Similar to the traditional speed adjustment algorithms: focus on real time 
adjustment, did not consider the future state of the vehicles, this is not 
achieving the rule of inter vehicles communication'''

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

# Calculating hte Euclidean distance 
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def predict_collision_time(distance1, speed1, distance2, speed2):
    if speed1 == 0 or speed2 == 0:
        return float('inf')
    return (distance1 / speed1) - (distance2 / speed2)

# Prevent default speed controlling algorithms
def set_vehicle_speed_mode(vehicle_id):
    traci.vehicle.setSpeedMode(vehicle_id, 0)

# Speed adjustment algorithms
def calculate_speed_adjustments(vehicles):

    # Loop through all the vehicles and get their respective information
    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        normal_speed = traci.vehicle.getAllowedSpeed(veh)

        # Check if there are other vehicles within a 50m distance
        nearby_vehicles = []
        for other_veh in vehicles:
            if other_veh!= veh:
                other_pos = traci.vehicle.getPosition(other_veh)
                distance = calculate_distance(pos[0], pos[1], other_pos[0], other_pos[1])
                if distance < 100:  # adjust this value to change the detection range
                    nearby_vehicles.append(other_veh)

        if len(nearby_vehicles) > 0:
            # If there are nearby vehicles, adjust speed to avoid collision
            min_collision_time = float('inf')
            closest_veh = None
            for other_veh in nearby_vehicles:
                other_speed = traci.vehicle.getSpeed(other_veh)
                other_pos = traci.vehicle.getPosition(other_veh)
                distance_to_junction = calculate_distance(pos[0], pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1])
                time_to_junction = distance_to_junction / speed if speed > 0 else float('inf')
                collision_time = predict_collision_time(distance_to_junction, speed, calculate_distance(other_pos[0], other_pos[1], traci.junction.getPosition("J2")[0], traci.junction.getPosition("J2")[1]), other_speed)

                if abs(collision_time) < min_collision_time:
                    min_collision_time = abs(collision_time)
                    closest_veh = other_veh

            if min_collision_time < 2:  # adjust this value to change the safety margin
                # Gradually slow down the vehicle to avoid collision with the closest vehicle
                traci.vehicle.slowDown(veh, speed * 0.3, 5)
                print(f"Gradually slowing down vehicle {veh} to avoid collision with {closest_veh}")
            else:
                # If no collision is predicted, set speed to normal
                traci.vehicle.setSpeed(veh, normal_speed)
                print(f"No collision predicted for vehicle {veh}. Setting normal speed.")
        else:
            # If no nearby vehicles, set speed to normal
            traci.vehicle.setSpeed(veh, normal_speed)
            print(f"No nearby vehicles for {veh}. Setting normal speed.")


def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        print(f"Simulation step: {step}")

        vehicles = traci.vehicle.getIDList()
        print(f"Vehicles in simulation: {vehicles}")
        
        print(f"Step: {step}:")
        for vehicle_id in vehicles:
            set_vehicle_speed_mode(vehicle_id)
        
        if len(vehicles) > 0:
            print("Successfully run")
            calculate_speed_adjustments(vehicles)
          
        else:
            print("No vehicles in simulation.")

        # Print out the speed of each vehicle
        print(f"Step {step}:")
        for vehicle_id in vehicles:
            speed = traci.vehicle.getSpeed(vehicle_id)
            print(f"Vehicle {vehicle_id}: Speed {speed:.2f} m/s")
        
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