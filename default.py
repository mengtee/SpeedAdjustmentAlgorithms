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

def run():
    # 时间槽调度参数
    time_slot = 5  # 每辆车通过交叉口的时间槽长度
    current_slot = 0
    vehicle_slots = {}
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()  # 进行一个仿真步
        
        # 获取所有车辆的ID
        vehicle_ids = traci.vehicle.getIDList()
        
        for vehicle_id in vehicle_ids:
            # 如果车辆没有分配时间槽，则分配一个
            if vehicle_id not in vehicle_slots:
                vehicle_slots[vehicle_id] = current_slot
                current_slot += time_slot
            
            # 获取车辆的位置和速度
            position = traci.vehicle.getPosition(vehicle_id)
            speed = traci.vehicle.getSpeed(vehicle_id)
            
            # 获取车辆即将进入的交叉口的ID
            next_tls = traci.vehicle.getNextTLS(vehicle_id)
            if next_tls:
                tls_id, tls_state, tls_distance = next_tls[0]
                if tls_distance < 10:  # 如果车辆接近交叉口
                    slot_time = vehicle_slots[vehicle_id]
                    sim_time = traci.simulation.getTime()
                    
                    if sim_time < slot_time:
                        # 减速到接近停止
                        traci.vehicle.slowDown(vehicle_id, 0.1, 1)
                    else:
                        # 恢复正常速度
                        traci.vehicle.slowDown(vehicle_id, speed, 1)
            
            print(f"Vehicle {vehicle_id}: Position: {position}, Speed: {speed}, Slot: {vehicle_slots[vehicle_id]}")

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


#===============================================================================
import traci
import sumolib

# 配置SUMO命令行参数
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-c", "simple_t_junction.sumocfg"]

# 启动SUMO仿真
traci.start(sumoCmd)

# 定义预测和速度调整函数
def predict_arrival_time(vehicle_id, distance):
    speed = traci.vehicle.getSpeed(vehicle_id)
    if speed > 0:
        return distance / speed
    return float('inf')

def adjust_speed(vehicle_id, target_time):
    current_time = traci.simulation.getTime()
    position = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    distance_to_junction = traci.vehicle.getDrivingDistance2D(vehicle_id, position[0], position[1])
    remaining_time = target_time - current_time

    if remaining_time > 0:
        new_speed = distance_to_junction / remaining_time
        traci.vehicle.setSpeed(vehicle_id, new_speed)
    else:
        traci.vehicle.setSpeed(vehicle_id, speed * 0.8)  # 降低速度，避免碰撞

# 主仿真循环
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()  # 进行一个仿真步
    
    # 获取所有车辆的ID
    vehicle_ids = traci.vehicle.getIDList()
    
    # 预测和调整每辆车的速度
    for vehicle_id in vehicle_ids:
        # 获取车辆即将进入的交叉口的ID和距离
        next_tls = traci.vehicle.getNextTLS(vehicle_id)
        if next_tls:
            tls_id, tls_state, tls_distance = next_tls[0]
            if tls_distance < 50:  # 如果车辆接近交叉口
                # 预测到达时间
                arrival_time = predict_arrival_time(vehicle_id, tls_distance)
                
                # 获取其他车辆的预测到达时间并调整速度
                for other_vehicle_id in vehicle_ids:
                    if vehicle_id != other_vehicle_id:
                        other_tls = traci.vehicle.getNextTLS(other_vehicle_id)
                        if other_tls:
                            other_tls_id, other_tls_state, other_tls_distance = other_tls[0]
                            if other_tls_id == tls_id and other_tls_distance < 50:
                                other_arrival_time = predict_arrival_time(other_vehicle_id, other_tls_distance)
                                if abs(arrival_time - other_arrival_time) < 2:  # 时间阈值，避免碰撞
                                    if arrival_time > other_arrival_time:
                                        adjust_speed(vehicle_id, arrival_time + 2)
                                    else:
                                        adjust_speed(vehicle_id, arrival_time - 2)
        
        # 输出车辆状态
        position = traci.vehicle.getPosition(vehicle_id)
        speed = traci.vehicle.getSpeed(vehicle_id)
        print(f"Vehicle {vehicle_id}: Position: {position}, Speed: {speed}")
    
# 关闭SUMO仿真
traci.close()