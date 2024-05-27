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

def adjust_speeds(vehicles, distances, speeds):
    n = len(vehicles)
    collision_threshold = 2  # Time threshold for potential collision (in seconds)

    for i in range(n):
        for j in range(i + 1, n):
            veh1, veh2 = vehicles[i], vehicles[j]
            dist1, dist2 = distances[i], distances[j]
            speed1, speed2 = speeds[i], speeds[j]

            collision_time = predict_collision_time(dist1, speed1, dist2, speed2)
            if abs(collision_time) < collision_threshold:  # Potential collision detected
                if dist1 < dist2:
                    print("Vehicle 2 is slowing down")
                    traci.vehicle.slowDown(veh2, speed2 * 0.8, 1000)  # Slow down veh2
                    traci.vehicle.setSpeed(veh1, traci.vehicle.getAllowedSpeed(veh1))  # Speed up veh1
                else:
                    traci.vehicle.slowDown(veh1, speed1 * 0.8, 1000)  # Slow down veh1
                    traci.vehicle.setSpeed(veh2, traci.vehicle.getAllowedSpeed(veh2))  # Speed up veh2
            else:
                traci.vehicle.setSpeed(veh1, traci.vehicle.getAllowedSpeed(veh1))  # Speed up veh1
                traci.vehicle.setSpeed(veh2, traci.vehicle.getAllowedSpeed(veh2))  # Speed up veh2

def calculate_speed_adjustments(vehicles):
    junction_pos = traci.junction.getPosition("J2")
    distances = []
    speeds = []

    for veh in vehicles:
        pos = traci.vehicle.getPosition(veh)
        speed = traci.vehicle.getSpeed(veh)
        distance_to_junction = calculate_distance(pos[0], pos[1], junction_pos[0], junction_pos[1])
        
        distances.append(distance_to_junction)
        speeds.append(speed)

    adjust_speeds(vehicles, distances, speeds)

def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
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