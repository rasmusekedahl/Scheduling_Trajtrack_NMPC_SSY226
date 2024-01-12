from Compo_slim import Compo_slim
from support_functions import make_Sequence_Planner_json
from instance_maker import make_an_instance
from Rviz_config_manipulator import set_up_Rviz_Config,set_up_Json_files
import os
import shutil

# def instance_writer(instance):
#     feasibility, optimum, solving_time, len_prev_routes, paths_changed, _ = Compo_slim(instance)
#     with open('data_storage.txt','a+') as write_file:
#         write_file.write('{},{},{},{},{},{} \n'.format(
#                                                             feasibility,
#                                                             optimum,
#                                                             instance,
#                                                             solving_time,
#                                                             len_prev_routes,
#                                                             paths_changed
#                                                             )
#         )
#         return instance,feasibility,solving_time

###################### HERE IS WHERE THE ACTUAL TESTING STARTS ######################

# the specific problem instance i am solving
# problem = 'MM_1538_0.2_20_21'
# problem = 'Volvo_gpss_2'
# problem = 'to_sabino'
# problem = 'MM_301010_0.5_7_7'
# current_dir = os.getcwd()
# gpss_directory = '/'.join(current_dir.split('/')[:-1])
# input_scenario = '{}/gpss/src/gpss_scenario/frames/*'.format(gpss_directory)

nodes = 27 # THIS CANNOT BE A PRIME NUMBER!!!
vehicles = 5
tasks = 10
edge_reduction = 0.7
time_horizon = 7000
seed = 7
make_an_instance(nodes, vehicles, tasks, edge_reduction, time_horizon, seed)
problem = f'MM_{nodes}_{vehicles}_{tasks}_{edge_reduction}_{time_horizon}_{seed}'
print('Problem: {}'.format(problem))
instance,optimum,running_time,len_previous_routes,paths_changed, [node_sequence,current_routes] = \
    Compo_slim(problem)
# for i in current_routes:
#     print(i.vehicle)
if optimum != 'None':
    make_Sequence_Planner_json(node_sequence, current_routes)

# Copy problem instance to output folder because gpss launch files need both
# the input and output files.
shutil.copyfile('test_cases/{}.json'.format(problem), './output/scheduler_input.json')

# print('\n\nFinished! Output is stored in:\n./output/scheduler_input.json\n./output/scheduler_result.json\n')
# print('Run ./copy_to_scenario.bash before launching gpss. (Needs to have sourced meta/set_paths.bash)\n')


################## A BUNCH OF STUFF TO MAKE THINGS INTERACTIVE #####################


# print(f"The current problem to be solved is {problem}")
# while False:
#     answer = input("do you want to change it? \n")
#     if answer == 'no':
#         break
#     elif answer == 'yes':
#         nodes = input('how many nodes? (integer)\n')
#         nodes = int(nodes)
#         vehicles = input('how many vehicles? (integer)\n')
#         vehicles = int(vehicles)
#         tasks = input('how many tasks? (integer)\n')
#         tasks = int(tasks)
#         edge_reduction = input('what is the EDGE REDUCTION? (real between 0 and 1)\n')
#         edge_reduction = float(edge_reduction)
#         time_horizon = input('what is the TIME HORIZON? (real)\n')
#         time_horizon = int(time_horizon)
#         seed = input('what is the SEED? (integer)\n')
#         seed = int(seed)
#         print('I AM MAKING THE INSTANCE')
#         make_an_instance(nodes, vehicles, tasks, edge_reduction, time_horizon, seed)
#         problem = f'MM_{str(nodes) + str(vehicles) + str(tasks)}_{edge_reduction}_{time_horizon}_{seed}'
#         break
#     else:
#         print('say either yes or no')
# while False:
#     # this is the part where I update the Rviz related files
#     answer2 = input("Do you want to updated the Rviz configuration files? \n")
#     if answer2 == 'no':
#         break
#     elif answer2 == 'yes':
#         set_up_Rviz('test_cases/{}.json'.format(problem))
#         ####### I am actually not able to launch Rviz....solve this!!!! ###########
#         # print("Files updated, launching Rviz")
#         # os.system('cd /home/sabino/Documents/gpss')
#         # os.system('source /opt/ros/humble/setup.bash')
#         # os.system('ros2 launch gpss_visualization_bringup gpss.launch.py')
#         break
#     else:
#         print('say either yes or no')
############# SOLVE THE PROBLEM AND CONVERT THE SCHEDULE INTO AN INPUT FOR SEQUENCE PLANNER ##############
# print("I am now solving the problem")
# instance,optimum,running_time,len_previous_routes,paths_changed, [node_sequence,current_routes] = \
#     Compo_slim(problem)
# if optimum != 'None':
#     while True:
#         generate_sp_file = input('The problem is SAT, do you wanna generate the input file for Sequence Planner?\n')
#         if generate_sp_file == 'yes':
#             make_Sequence_Planner_json(node_sequence, current_routes)
#             break
#         elif generate_sp_file == 'no':
#             break
#         else:
#             print('answer yes or no please :)')


