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
    if speed1 == speed2:
        return float('inf')
    return (distance1 / speed1) - (distance2 / speed2)

def calculate_speed_adjustments(vehicles):
    veh1 = vehicles[0]
    veh2 = vehicles[1]

    speed1 = traci.vehicle.getSpeed(veh1)
    speed2 = traci.vehicle.getSpeed(veh2)

    pos1 = traci.vehicle.getPosition(veh1)
    pos2 = traci.vehicle.getPosition(veh2)

    junction_pos = traci.junction.getPosition("J2")

    distance_to_junction1 = calculate_distance(pos1[0], pos1[1], junction_pos[0], junction_pos[1])
    distance_to_junction2 = calculate_distance(pos2[0], pos2[1], junction_pos[0], junction_pos[1])

    time_to_junction1 = distance_to_junction1 / speed1 if speed1 > 0 else float('inf')
    time_to_junction2 = distance_to_junction2 / speed2 if speed2 > 0 else float('inf')

    normal_speed1 = traci.vehicle.getAllowedSpeed(veh1)
    normal_speed2 = traci.vehicle.getAllowedSpeed(veh2)

    collision_time = predict_collision_time(distance_to_junction1, speed1, distance_to_junction2, speed2)

    print(f"Vehicle {veh1}: Speed = {speed1}, Distance to Junction = {distance_to_junction1}, Time to Junction = {time_to_junction1}")
    print(f"Vehicle {veh2}: Speed = {speed2}, Distance to Junction = {distance_to_junction2}, Time to Junction = {time_to_junction2}")
    print(f"Predicted Collision Time: {collision_time}")

    if abs(collision_time) < 2:  # Potential collision detected
        if time_to_junction1 < time_to_junction2:
            print(f"Slowing down vehicle {veh2}")
            traci.vehicle.slowDown(veh2, speed2 * 0.8, 1000)  # Slow down veh2
            traci.vehicle.setSpeed(veh1, normal_speed1)  # Speed up veh1 to normal speed
        else:
            print(f"Slowing down vehicle {veh1}")
            traci.vehicle.slowDown(veh1, speed1 * 0.8, 1000)  # Slow down veh1
            traci.vehicle.setSpeed(veh2, normal_speed2)  # Speed up veh2 to normal speed
    else:
        traci.vehicle.setSpeed(veh1, normal_speed1)  # Speed up veh1 to normal speed
        traci.vehicle.setSpeed(veh2, normal_speed2)  # Speed up veh2 to normal speed

def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        print(f"This is step {step}")
        vehicles = traci.vehicle.getIDList()
        
        if len(vehicles) >= 2:
            calculate_speed_adjustments(vehicles)
        
        step += 1
    traci.close()
    sys.stdout.flush()

if __name__ == "__main__":
    options = get_options()

    if options.nogui:
        sumoBinary = checkBinary("sumo")
    else:
        sumoBinary = checkBinary("sumo-gui")

    traci.start([sumoBinary, '-c', 'final_tjunction.sumocfg', "--tripinfo-output", "tripinfor.xml"])
    run()