import os
import json
import pathlib

import numpy as np

from basic_motion_model.motion_model import UnicycleModel

from interfaces.interface_mpc_tracker import MpcInterface
from interfaces.interface_motion_plan import MotionPlanInterface
from pkg_scheduler.job_schedule import JobScheduler
from pkg_robot.robot import RobotManager

from configs import MpcConfiguration
from configs import CircularRobotSpecification

from visualizer.object import CircularVehicleVisualizer
from visualizer.mpc_plot import MpcPlotInLoop

ROOT_DIR = pathlib.Path(__file__).resolve().parents[1]
# DATA_DIR = os.path.join(ROOT_DIR, "data", "test_data")
DATA_DIR = os.path.join(ROOT_DIR, "data", "test_data")
CNFG_DIR = os.path.join(ROOT_DIR, "config")
VB = False
TIMEOUT = 1000

robot_ids = None # if none, read from schedule

### Configurations
config_mpc_path = os.path.join(CNFG_DIR, "mpc_fast.yaml")
config_robot_path = os.path.join(CNFG_DIR, "robot_spec.yaml")

config_mpc = MpcConfiguration.from_yaml(config_mpc_path)
config_robot = CircularRobotSpecification.from_yaml(config_robot_path)

### Map, graph, and schedule paths
map_path = os.path.join(DATA_DIR, "map.json")
graph_path = os.path.join(DATA_DIR, "graph.json")
schedule_path = os.path.join(DATA_DIR, "schedule.csv")
start_path = os.path.join(DATA_DIR, "robot_start.json")
with open(start_path, "r") as f:
    robot_starts = json.load(f)

### Set up scheduler
scheduler = JobScheduler.from_csv(schedule_path)
robot_ids = scheduler.robot_ids if robot_ids is None else robot_ids

### Set up robots
robot_manager = RobotManager()
for rid in robot_ids:
    robot = robot_manager.create_robot(config_robot, UnicycleModel(sampling_time=config_mpc.ts), rid)
    robot.set_state(np.asarray(robot_starts[rid]))
    planner = MotionPlanInterface(rid, config_mpc, config_robot, verbose=VB)
    planner.load_map_from_json(map_path, inflation_margin=config_robot.vehicle_width+0.2)
    planner.load_graph_from_json(graph_path)
    planner.set_schedule(*scheduler.get_robot_schedule(rid))
    planner.update_schedule(nomial_speed=1.5, sampling_method="linear")
    controller = MpcInterface(config_mpc, config_robot, verbose=VB)
    visualizer = CircularVehicleVisualizer(config_robot.vehicle_width, indicate_angle=True)
    robot_manager.add_robot(robot, controller, planner, visualizer)
    robot_manager.add_schedule(rid, np.asarray(robot_starts[rid]), scheduler.get_robot_schedule(rid))

### Run
main_plotter = MpcPlotInLoop(config_robot)
main_plotter.plot_in_loop_pre(planner.current_map, planner.current_graph)
color_list = ["b", "r", "g"]
for i, rid in enumerate(robot_ids):
    planner = robot_manager.get_planner(rid)
    controller = robot_manager.get_controller(rid)
    visualizer = robot_manager.get_visualizer(rid)
    main_plotter.add_object_to_pre(rid,
                                   planner.current_ref_trajectory,
                                   controller.state,
                                   controller.final_goal,
                                   color=color_list[i])
    visualizer.plot(main_plotter.map_ax, *robot.state)

# zoom_scope = planner.current_map.get_boundary_scope() # [xmin xmax ymin ymax]
# zoom_enlarge = config_mpc.N_hor * config_mpc.ts * config_robot.lin_vel_max
# zoom_in = list(planner.current_map.get_boundary_scope())

for kt in range(TIMEOUT):
    robot_states = []
    incomplete = False
    for i, rid in enumerate(robot_ids):
        robot = robot_manager.get_robot(rid)
        planner = robot_manager.get_planner(rid)
        controller = robot_manager.get_controller(rid)
        visualizer = robot_manager.get_visualizer(rid)
        other_robot_states = robot_manager.get_other_robot_states(rid, config_mpc)

        ref_states, ref_speed = planner.get_local_ref(kt*config_mpc.ts, robot.state)
        print(f"Robot {rid} ref speed: {ref_speed}") # XXX
        controller.set_ref_states(ref_states, ref_speed=ref_speed)
        actions, pred_states, cost, closest_obstacle_list, current_refs = controller.run_step(None, other_robot_states)

        ### Real run
        robot.step(actions[-1])
        robot_manager.set_pred_states(rid, np.asarray(pred_states))

        main_plotter.update_plot(rid, kt, actions[-1], robot.state, cost, np.asarray(pred_states), current_refs)
        visualizer.update(*robot.state)

        if not controller.check_termination_condition(external_check=planner.idle):
            incomplete = True

        robot_states.append(robot.state)

    # rs_xmin = min([x[0] for x in robot_states])
    # rs_xmax = max([x[0] for x in robot_states])
    # rs_ymin = min([x[1] for x in robot_states])
    # rs_ymax = max([x[1] for x in robot_states])
    
    # if (rs_xmin-zoom_enlarge > zoom_in[0]):
    #     zoom_in[0] = rs_xmin-zoom_enlarge
    # if (rs_xmax+zoom_enlarge < zoom_in[1]):
    #     zoom_in[1] = rs_xmax+zoom_enlarge
    # if (rs_ymin-zoom_enlarge > zoom_in[2]):
    #     zoom_in[2] = rs_ymin-zoom_enlarge
    # if (rs_ymax+zoom_enlarge < zoom_in[3]):
    #     zoom_in[3] = rs_ymax+zoom_enlarge
    
    main_plotter.plot_in_loop(time=kt*config_mpc.ts, autorun=False, zoom_in=None)
    if not incomplete:
        break
    
    
main_plotter.show()
input('Press anything to finish!')
main_plotter.close()
