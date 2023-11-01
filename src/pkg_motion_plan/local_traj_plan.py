import math

import numpy as np
from scipy import interpolate # for refine ref states

from .path_plan_cspace import visibility
from ._traj_generation import TrajectoryGeneration

from typing import List


class LocalTrajPlanner:
    def __init__(self, sampling_time: float, horizon: int, max_speed: float, verbose:bool=False) -> None:
        """
        Descriptions:
            The local planner takes path nodes and ETAs as inputs, and outputs local reference.

        Arguments:
            sampling_time: The sampling time of the local planner.
            horizon: The horizon of the local planner.
            verbose: If True, the local planner will print out the debug information.
        """
        self.ts = sampling_time
        self.N_hor = horizon
        self.v_max = max_speed
        self.vb = verbose

        self.path_planner = None

        self._ref_path = None
        self._ref_path_time = None
        self._current_node = None
        self._current_idx = None

        self.traj_gen = TrajectoryGeneration()
        self.traj_gen.set_sample_time(self.ts)
        self._ref_speed = None
        self._base_traj = None
        self._base_traj_time = None
        self._base_traj_target = None
        self._base_idx = None
        self._sampling_method = None

    @property
    def current_node(self) -> tuple:
        return self._current_node
    
    @property
    def ref_traj(self) -> np.ndarray:
        return np.asarray(self._base_traj)
    
    @property
    def ref_speed(self) -> float:
        return self._ref_speed
    
    @staticmethod
    def _refine_ref_states(original_states: np.ndarray, original_speed: float, new_speed: float):
        """Refine the reference states to match the new speed."""
        n_states = original_states.shape[0]
        distances = np.cumsum(np.sqrt(np.sum(np.diff(original_states, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0)/distances[-1]
        fx = interpolate.interp1d(distances, original_states[:, 0], kind='linear')
        fy = interpolate.interp1d(distances, original_states[:, 1], kind='linear')

        num_points = int(original_speed/new_speed*n_states)  
        new_distances = np.linspace(0, 1, num_points)
        new_x = fx(new_distances)
        new_y = fy(new_distances)
        new_heading = np.arctan2(np.diff(new_y), np.diff(new_x))
        new_heading = np.append(new_heading, new_heading[-1])
        new_states = np.column_stack([new_x, new_y, new_heading])[:n_states, :]
        return new_states

    def load_map(self, boundary_coords: List[tuple], obstacle_list: List[List[tuple]]):
        self.path_planner = visibility.VisibilityPathFinder(boundary_coords, obstacle_list, verbose=self.vb)

    def load_path(self, path_coords: List[tuple], path_times: List[float], nomial_speed:float=None, method:str='linear'):
        """The reference speed is used to generate the base trajectory."""
        self._ref_path = path_coords
        self._ref_path_time = path_times
        self._current_node = self._ref_path[0]
        self._current_idx = 0

        self.traj_gen.set_reference_path(self._ref_path)
        self.traj_gen.set_reference_time(self._ref_path_time)
        self.traj_gen.set_nominal_speed(nomial_speed)
        self._base_traj, self._base_traj_time, self._base_traj_target = self.traj_gen.generate_trajectory(method=method)
        self._base_idx = 0
        self._sampling_method = method

    def get_local_ref(self, current_time: float, current_pos: tuple, idx_check_range:int=10):
        if self._sampling_method == 'time':
            raise NotImplementedError('Not finished yet.')
            ref_states, ref_speed, done = self.get_local_ref_from_time_sampling(current_time)
        elif self._sampling_method == 'linear':
            ref_states, ref_speed, done = self.get_local_ref_from_linear_sampling(current_time, current_pos, idx_check_range)
        else:
            raise ValueError('Sampling method not supported.')
        if ref_speed > 1e-6:
            ref_states = self._refine_ref_states(ref_states, self.traj_gen.speed, ref_speed)
        return ref_states, ref_speed, done

    def get_local_ref_from_linear_sampling(self, current_time: float, current_pos: tuple, idx_check_range:int=10):
        """
        Descriptions:
            The local planner takes the current position as input, and outputs the local reference.
        Arguments:
            current_pos: The current position of the agent (robot).
            idx_check_range: The range of the index to check the docking point.
        Returns:
            ref_states: The local state reference.
            ref_speed: The reference speed.
            done: If the current node is the last node in the reference path, return True. Otherwise, return False.
        """
        lb_idx = max(self._base_idx, 0)
        ub_idx = min(self._base_idx+idx_check_range, len(self._base_traj)-1)

        distances = [math.hypot(current_pos[0]-x[0], current_pos[1]-x[1]) for x in self._base_traj[lb_idx:ub_idx]]
        self._base_idx = lb_idx + distances.index(min(distances)) # this is the index of the docking point on the base trajectory

        distance_to_current_node = math.hypot(current_pos[0]-self.current_node[0], current_pos[1]-self.current_node[1])
        timediff_to_current_node = max(self._ref_path_time[self._current_idx] - current_time, 0) + 1e-6
        ref_speed = min(distance_to_current_node/timediff_to_current_node, self.v_max)

        if (self._base_idx+self.N_hor >= len(self._base_traj)):
            ref_states = np.array(self._base_traj[self._base_idx:] + [self._base_traj[-1]]*(self.N_hor-(len(self._base_traj)-self._base_idx)))
        else:
            ref_states = np.array(self._base_traj[self._base_idx:self._base_idx+self.N_hor])

        self._current_idx = self._ref_path.index(self._base_traj_target[self._base_idx])
        self._current_node = self._ref_path[self._current_idx]
        if self._current_idx == len(self._ref_path)-1:
            done = True
            # ref_speed = 0.0
        else:
            done = False

        return ref_states, ref_speed, done

    def get_local_ref_from_time_sampling(self, current_time: float):
        try:
            self._base_idx = np.where(current_time>np.asarray(self._base_traj_time))[0][-1] + 1
        except IndexError:
            self._base_idx = 0

        done = False
        if self._base_idx >= len(self._base_traj):
            done = True
            self._base_idx = len(self._base_traj) - 1

        if (self._base_idx+self.N_hor >= len(self._base_traj)):
            ref_states = np.array(self._base_traj[self._base_idx:] + [self._base_traj[-1]]*(self.N_hor-(len(self._base_traj)-self._base_idx)))
        else:
            ref_states = np.array(self._base_traj[self._base_idx:self._base_idx+self.N_hor])

        ref_speed = math.hypot(ref_states[0, 0]-ref_states[1,0], ref_states[0, 1]-ref_states[1,1]) / self.ts
        ref_speed = min(ref_speed, self.v_max)

        self._current_idx = self._ref_path.index(self._base_traj_target[self._base_idx])
        self._current_node = self._ref_path[self._current_idx]
        if self._current_idx == len(self._ref_path)-1:
            done = True
        else:
            done = False

        return ref_states, ref_speed, done

    def get_new_path(self, waypoints: List[tuple], time_list: List[float]) -> List[tuple]:
        """Get a new path from the given waypoints and time list to avoid static obstacles."""
        if len(waypoints) < 2:
            raise ValueError("Waypoints must have at least two points")
        new_path = [waypoints[0]]
        new_time = [time_list[0]]
        for i in range(len(waypoints)-1):
            start, end = waypoints[i], waypoints[i+1]
            new_segment = self.path_planner.get_ref_path(start, end)[1:]
            new_path.extend(new_segment)
            new_time.extend([None for _ in range(len(new_segment)-1)])
            new_time.append(time_list[i+1])
        return new_path, new_time
    
    

