import numpy as np
import networkx as nx

from basic_map.map_geometric import GeometricMap
from basic_map.graph import NetGraph
from basic_obstacle.geometry_plain import PlainPolygon

from pkg_motion_plan.global_path_plan import GlobalPathPlanner
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner

from configs import MpcConfiguration
from configs import CircularRobotSpecification
from matplotlib.axes import Axes
from typing import List, Tuple, Union

class MotionPlanInterface:
    def __init__(self, robot_id, config_mpc: MpcConfiguration, config_robot: CircularRobotSpecification, verbose:bool=False):
        self._robot_id = robot_id
        self.global_planner = GlobalPathPlanner(robot_id)
        self.local_planner = LocalTrajPlanner(config_mpc.ts, config_mpc.N_hor, config_robot.lin_vel_max, verbose)
        
        self._idle = True
        self.current_path_plan = None
        self.current_time_plan = None

        self._current_map = None
        self._inflated_map = None
        self._current_graph = None

    @property
    def robot_id(self):
        return self._robot_id

    @property
    def idle(self):
        return self._idle

    @property
    def current_node_coord(self) -> tuple:
        return self.local_planner.current_node
    
    @property
    def current_ref_trajectory(self) -> np.ndarray:
        return self.local_planner.ref_traj
    
    @property
    def current_map(self):
        return self._current_map
    
    @property
    def inflated_map(self):
        return self._inflated_map
    
    @property
    def current_graph(self):
        return self._current_graph
    
    def inflate_map(self, original_map: GeometricMap, inflation_margin: float):
        boundary_coords, obstacle_coords_list = original_map()
        for i, obs in enumerate(obstacle_coords_list):
            inflated_obs = PlainPolygon.from_list_of_tuples(obs).inflate(inflation_margin)
            obstacle_coords_list[i] = inflated_obs()
        boundary_coords = PlainPolygon.from_list_of_tuples(boundary_coords).inflate(-inflation_margin)
        boundary_coords = boundary_coords()
        return GeometricMap.from_raw(boundary_coords, obstacle_coords_list)

    def load_map(self, boundary_coords: List[tuple], obstacle_list: List[List[tuple]], inflation_margin:float=None):
        self._current_map = GeometricMap.from_raw(boundary_coords, obstacle_list)
        if inflation_margin is not None:
            self._inflated_map = self.inflate_map(self._current_map, inflation_margin)
            self.local_planner.load_map(*self._inflated_map())
        else:
            self.local_planner.load_map(boundary_coords, obstacle_list)

    def load_map_from_json(self, json_path: str, inflation_margin:float=None):
        self._current_map = GeometricMap.from_json(json_path)
        self.load_map(*self.current_map(), inflation_margin)

    def load_graph(self, graph: NetGraph):
        self._current_graph = graph
        self.global_planner.load_graph(self._current_graph)

    def load_graph_from_json(self, json_path: str):
        self._current_graph = NetGraph.from_json(json_path)
        self.global_planner.load_graph(self._current_graph)


    def set_schedule(self, path_nodes: list, path_times: Union[List[float], None], whole_path: bool):
        """Set the current path plan and time plan. Set the idle flag to False."""
        self._idle = False
        self.global_planner.set_schedule(path_nodes, path_times, whole_path)

    def update_schedule(self, nomial_speed: float, time_offset:float=0.0, sampling_method:str="linear"):
        """Update the current path plan and time plan.
        
        Arguments:
            time_offset: The delay time of the current schedule.
        """
        path_coords, path_times = self.global_planner.get_schedule(time_offset)
        self.current_path_plan = path_coords
        self.current_time_plan = path_times

        self.local_planner.load_path(self.current_path_plan, self.current_time_plan, nomial_speed, sampling_method)

    def get_local_ref(self, current_time: float, current_pos:tuple=None, idx_check_range:int=10) -> Tuple[np.ndarray, float]:
        """If done, the schedule is finished.

        Arguments:
            current_time: The current time (e.g. [sec]).
        
        Returns:
            ref_states: The reference states of the robot at the current time step.
            ref_speed: The reference speed of the robot at the current time step.
            """
        ref_states, ref_speed, done = self.local_planner.get_local_ref(current_time, current_pos, idx_check_range)
        if done:
            self._idle = True
        return ref_states, round(ref_speed,2)
    

    def plot_map(self, ax, inflated:bool=False, original_plot_args:dict={'c':'k'}, inflated_plot_args:dict={'c':'r'}):
        self.current_map.plot(ax, inflated, original_plot_args, inflated_plot_args)

    def plot_graph(self, ax, node_style='x', node_text:bool=True, edge_color='r'):
        self.current_graph.plot_graph(ax, node_style, node_text, edge_color)

    def plot_schedule(self, ax: Axes, plot_args:dict={'c':'r'}):
        ax.plot(self.current_ref_trajectory[:,0], self.current_ref_trajectory[:,1], 'o', markerfacecolor='none', **plot_args)