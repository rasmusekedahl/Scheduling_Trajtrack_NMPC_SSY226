import json
import os
import shutil
from instance_maker import make_an_instance
from Compo_slim import Compo_slim
from support_functions import make_Sequence_Planner_json_for_online
import csv
from math import atan2

class SchedulerConfig:
    def __init__(self, nodes=0, vehicles=0, tasks=0, edge_reduction=0, time_horizon=0, seed=0, dir='./output'):
        self.nodes = nodes
        self.vehicles = vehicles
        self.tasks = tasks
        self.edge_reduction = edge_reduction
        self.time_horizon = time_horizon
        self.seed = seed
        self.output_folder = dir
        self.problem_filename = f'MM_{self.nodes}_{self.vehicles}_{self.tasks}_{self.edge_reduction}_{self.time_horizon}_{self.seed}'
        self.mpc_mapInstance_filename = 'graph.json'                # 'mpc_mapInstance.json'
        self.scheduler_result_filename = 'scheduler_result_v2.json'
        self.mpc_input_filename = 'schedule.csv'                    # 'mpc_input.csv'
        self.robot_start_filename = 'robot_start.json'

class Scheduler:
    def __init__(self, config:SchedulerConfig):
        self.config = config
        self.result = dict()

    def create_random_instance(self):
        make_an_instance(self.config.nodes, self.config.vehicles, self.config.tasks, self.config.edge_reduction, self.config.time_horizon, self.config.seed)
        print('Problem: {}'.format(self.config.problem_filename))

        # Create the output folder if it doesn't exist
        os.makedirs(self.config.output_folder, exist_ok=True)

        # Copy problem instance to output folder
        shutil.copyfile('test_cases/{}.json'.format(self.config.problem_filename), f'{self.config.output_folder}/{self.config.problem_filename}.json')
        print(self.config.problem_filename)
        return self.config.problem_filename

    def generate_graph_for_mpc(self):
        input_file_path = f'{self.config.output_folder}/{self.config.problem_filename}.json'
        output_file_path = f'{self.config.output_folder}/{self.config.mpc_mapInstance_filename}'

        # Ensure the input file exists
        if not os.path.exists(input_file_path):
            print(f"Error: Input file '{input_file_path}' not found.")
            return

        graph_data = self._parse_graph(input_file_path)

        # Create the output folder if it doesn't exist
        os.makedirs(f'{self.config.output_folder}', exist_ok=True)

        # Specify the path to the output JSON file
        self._write_json(graph_data, output_file_path)
        print("Output dumped to: {}".format(output_file_path))

    def _parse_graph(self, input_file_path):
        # Open the file and load the JSON data
        with open(input_file_path, 'r') as file:
            json_data = json.load(file)

        # Now 'data' contains the parsed JSON data
        nodes = json_data['test_data']['nodes']
        out_graph = {"nodes": {}, "edges": [], "start_list": {}, "charging_stations": []}

        # Parsing logic
        for _, node in enumerate(nodes):
            key = "d_{}".format(node)
            out_graph["nodes"][key] = [nodes[node]["x"], nodes[node]["y"]]

        edges = json_data['edges']
        for _, edge in enumerate(edges):
            n = edge.split(",")
            n1 = "d_{}".format(n[0])
            n2 = "d_{}".format(n[1])
            out_graph["edges"].append([n1, n2])

        start_list = json_data["test_data"]["start_list"]
        for _, robo in enumerate(start_list):
            node = "d_{}".format(start_list[robo])
            robo = "R_{}".format(robo)
            out_graph["start_list"][robo] = node

        charging_stations = json_data["test_data"]["charging_stations"]
        for _, cstation in enumerate(charging_stations):
            cs = "d_{}".format(cstation)
            out_graph["charging_stations"].append(cs)

        return out_graph

    def _write_json(self, json_data, file_name):
        # Open the file and write the JSON data
        with open(file_name, 'w') as file:
            file.write(json.dumps(json_data, indent=4))
    
    def _parse_schedule(self, robot_data):
        csv_data = []
        
        for robot in robot_data['robots']:
            robot_name = f'R_{robot["name"]}'
            path = robot['path']
            
            for i in range(len(path)):
                node_name = f'd_{path[i]["name"]}'
                eta = int(path[i]["time"])
                
                csv_data.append([robot_name, node_name, eta])
        
        return csv_data

    def _write_csv(self, csv_data, file_name='output.csv'):
        with open(file_name, 'w', newline='') as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(['robot_id', 'node_id', 'ETA'])
            csv_writer.writerows(csv_data)

    # This function stores the robots start position as per [x, y, theta] co-ordinate system.
    # Here, theta is the orientation of the robot in radian, [-pi, +pi]
    def _parse_robot_start(self, scheduler_result, map_instance):
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
    
    def generate_robot_start_for_mpc(self):
        output_file_path = f'{self.config.output_folder}/{self.config.robot_start_filename}'

        # Open the file and load the JSON data
        with open(f'{self.config.output_folder}/{self.config.scheduler_result_filename}', 'r') as file:
            sch_result = json.load(file)
        
        with open(f'{self.config.output_folder}/{self.config.mpc_mapInstance_filename}', 'r') as file:
            map_instance = json.load(file)
        
        # parse the input graph json and dump to output json file 
        self._write_json(self._parse_robot_start(sch_result, map_instance), output_file_path)
        print("Output dumped to: {}".format(output_file_path))
    
    def _problem_filename_to_config(self, problem_filename:str) -> SchedulerConfig:
        tokens = problem_filename.split("_")
        # for "MM_4_1_1_1.0_7000_7.json" --> [MM, 4, 1, 1, 1.0, 7000, 7.json]
        # for "MM_27_5_10_0.7_7000_7.json" --> [MM, 27, 5, 10, 0.7, 7000, 7.json]

        nodes = int(tokens[1])
        vehicles = int(tokens[2])
        tasks = int(tokens[3])
        edge_reduction = float(tokens[4])
        time_horizon = int(tokens[5])
        seed = int(tokens[6].split(".")[0])

        config = SchedulerConfig(nodes, vehicles, tasks, edge_reduction, time_horizon, seed)
        return config

    def set_problem_filename(self, problem_filename) -> bool:        
        if not (problem_filename.startswith('MM_') and problem_filename.endswith('.json')):
            return None
        
        self.config = self._problem_filename_to_config(problem_filename)
        return True
    
    def schedule(self, problem_dict:dict=None):
        # Run the SComSat scheduler
        problem = ""
        if problem_dict == None:
            problem = self.config.problem_filename
                
        instance, optimum, running_time, len_previous_routes, paths_changed, [node_sequence, current_routes] = \
            Compo_slim(problem=problem, dir=self.config.output_folder, problem_dict=problem_dict)
        if optimum != 'None':
            make_Sequence_Planner_json_for_online(node_sequence, current_routes, dir=self.config.output_folder)

        # Open the file and load the JSON data
        with open(f'{self.config.output_folder}/{self.config.scheduler_result_filename}', 'r') as file:
            self.result = json.load(file)
        
        # TODO: parse it as per MPC requirement and convert it to csv
        # csv_data = self._parse_schedule(json_data)
        # self._write_csv(csv_data, f'{self.config.output_folder}/{self.config.mpc_input_filename}')
        return self.result

def example_single_problem():
    config = SchedulerConfig(nodes=4, vehicles=1, tasks=1, edge_reduction=1.0, time_horizon=7000, seed=7)
    # config = SchedulerConfig(nodes=27, vehicles=5, tasks=10, edge_reduction=0.7, time_horizon=7000, seed=7)

    # Example usage with custom initialization
    scheduler = Scheduler(config)
    
    # Create instance
    instance_name = scheduler.create_random_instance()

    # Run the entire scheduling process and generates schedule.csv
    scheduler.schedule()

    # Generate graph.json file required by mpc
    # scheduler.generate_graph_for_mpc()

    # Generates a robot_start.json file required by mpc
    # scheduler.generate_robot_start_for_mpc()

def example_multiple_problem():
    def copy_files(source_dir, destination_folder_name):
        # Create destination folder if it doesn't exist
        destination_folder = os.path.join(os.getcwd(), destination_folder_name)
        if not os.path.exists(destination_folder):
            os.makedirs(destination_folder)
        
        # Get a list of all files in the source directory
        files = [f for f in os.listdir(source_dir) if os.path.isfile(os.path.join(source_dir, f))]
        
        # Iterate through the files and copy .json and .csv files to the destination folder
        for file in files:
            if file.endswith(('.json', '.csv')):
                source_path = os.path.join(source_dir, file)
                destination_path = os.path.join(destination_folder, file)
                shutil.copy2(source_path, destination_path)
                print(f"Copied: {file} to {destination_folder}")

    # Just anoter way to create a scheduler 
    scheduler = Scheduler(SchedulerConfig())
    
    # NOTE: These 'MM_*.json' files must already exist in ./output folder
    # predefined_problems = ['MM_4_1_1_1.0_7000_7.json', 'MM_27_5_10_0.7_7000_7.json']
    predefined_problems = ['MM_8_2_6_1.0_7000_7.json']

    for problem_filename in predefined_problems:
        # set new problem to solve
        if scheduler.set_problem_filename(problem_filename) == False:
            print(f"Invalid filename.\n")
            return
        
        # check if problem file exist if NOT then create one and store in output folder
        if not os.path.exists(f"{scheduler.config.output_folder}/{problem_filename}"):
            print(f"file doesn't exist in {scheduler.config.output_folder} directory, creating random instance with same specification.\n")
            make_an_instance(scheduler.config.nodes, scheduler.config.vehicles, scheduler.config.tasks, scheduler.config.edge_reduction, scheduler.config.time_horizon, scheduler.config.seed)
            shutil.copyfile('test_cases/{}.json'.format(scheduler.config.problem_filename), f'{scheduler.config.output_folder}/{scheduler.config.problem_filename}.json')
        
        scheduler.schedule()                        # Run the entire scheduling process and generates schedule.csv
        # scheduler.generate_graph_for_mpc()          # Generate graph.json file required by mpc
        # scheduler.generate_robot_start_for_mpc()    # Generates a robot_start.json file required by mpc
        
        # store all output in a folder with name as problem name
        # source_path = scheduler.config.output_folder
        # destination_folder_name = problem_filename.split(".")[0]
        # destination_path = f"{scheduler.config.output_folder}/{destination_folder_name}"
        # if not os.path.exists(destination_path):
        #     os.makedirs(destination_path)
        # copy_files(source_path, destination_path)

def main():
    # example_single_problem()
    example_multiple_problem()

if __name__ == "__main__":
    main()
