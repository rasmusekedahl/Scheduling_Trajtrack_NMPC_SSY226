# sch2mpc_robot_start.py
# This program stores the robots start position as per [x, y, theta] coordinate system.
# Here, theta is the orientation of the robot in radian, [-pi, +pi]

# robot_start.json
# {
#     "R_0" : [0.0, 1.0, +3.14],
#     "R_1" : [1.0, 0.0, -3.14]
# }


import json
import sys
from math import atan2

def parse_robot_start(scheduler_result, map_instance):
    robot_start = dict()

    for robot in scheduler_result['robots']:
        robot_name = f'R_{robot["name"]}'
        path = robot['path']
        start = f'd_{path[0]["name"]}'
        next = f'd_{path[1]["name"]}'

        robot_start[robot_name] = [start, next]
    
    nodes = map_instance["nodes"]
    for robot_name, pos_pair in robot_start.items():
        start = nodes[pos_pair[0]]
        next = nodes[pos_pair[1]]
        dx = next[0] - start[0]
        dy  = next[1] - start[1]
        robot_start[robot_name] = [start[0], start[1], atan2(dy, dx)]

    return robot_start

def write_json(json_data, file_name):
    # Open the file and write to the JSON data
    with open(file_name, 'w') as file:
        file.write(json.dumps(json_data, indent=4))

def main():
    # checking command line args
    if len(sys.argv) < 2:
        sys.exit("usage error: missing input file path example: ./output/mpc_mapInstance.json")
    
    if len(sys.argv) < 3:
        sys.exit("usage error: missing input file path example: ./output/scheduler_result.json")

    if len(sys.argv) < 4:
        sys.exit("usage error: missing output file path example: ./output/robot_start.json")

    # Specify the path to your JSON file
    map_instance_file_path = sys.argv[1]
    sch_result_file_path = sys.argv[2]
    robot_start_out_json_file_path = sys.argv[3]

    # Open the file and load the JSON data
    with open(sch_result_file_path, 'r') as file:
        sch_result = json.load(file)
    
    with open(map_instance_file_path, 'r') as file:
        map_instance = json.load(file)
    
    # parse the input graph json and dump to output json file 
    write_json(parse_robot_start(sch_result, map_instance), robot_start_out_json_file_path)
    print("Output dumped to: {}".format(robot_start_out_json_file_path))


if __name__ == '__main__':
    main()