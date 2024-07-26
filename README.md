# SUMO TraCI Simulation

This repository contains the necessary files and instructions to run a SUMO TraCI simulation.

## Prerequisites

Before you begin, ensure you have the following installed on your system:

- **SUMO (Simulation of Urban MObility)**: [Installation Guide](https://sumo.dlr.de/docs/Installing/index.html)
- **Python**: [Download Python](https://www.python.org/downloads/)
- **TraCI (Traffic Control Interface)**: This is included with SUMO, but you might need the Python TraCI library.

You can install the Python TraCI library using pip:
```bash
pip install sumo-traCI
```

## Repository Structure

1. Greedy Heuristic folder
- `cross.sumocfg`: The main configuration file for the SUMO simulation.
- `cross.net.xml`: The network file defining the 4 legs cross junction road layout.
- `cross.rou.xml`: The routes file defining vehicle routes.
- `main.py`: The Python script to run the TraCI simulation.
- `Scheduler.py`: Logic in scheduling vehicles.

2. MPC_algorithm folder
- `tjunction_01.sumocfg, tjunction_02.sumocfg, tjunction_03.sumocfg, tjunction_04.sumocfg, tjunction_05.sumocfg`: The main configuration file with different traffic intensities for the SUMO simulation.
- `tjunction.net.xml`: The network file defining the T junction road layout.
- `demand_01.rou.xml, demand_02.rou.xml, demand_03.rou.xml, demand_04.rou.xml, demand_05.rou.xml`: The routes file defining vehicle routes.
- `main.py`: The Python script to run the TraCI simulation with MPC algorithms.
- `Krauss.py`: Default signaling Krauss algorithms in SUMO.

## Instructions to Run the Simulation

### Step 1: Make sure the terminal is in the repository

```bash
cd GreedyHeuristic
```

or 

```bash
cd MPC_algorithms
```

#### Running With TraCI
To run the simulation using the TraCI Python script, execute:
```bash
python main.py
```

### Step 4: Visualize the Simulation 

To run the simulation, click the play button on the top left of the simulation GUI.
