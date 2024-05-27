# Simple implementable class, default implementation
import optparse
import os
import sys

os.environ['SUMO_HOME'] = '/Users/TEEMENGKIAT/sumo'

# Importing python module from $SUMO_HOME
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

# Defining if need to run on cmd line (without gui)
def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

# Main loop, stop when no more vehicle is expected to enter the simulation
def run():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        for veh_id in traci.vehicle.getIDList():
            position = traci.vehicle.getPosition(veh_id)
            print(position)
        print(step)
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
