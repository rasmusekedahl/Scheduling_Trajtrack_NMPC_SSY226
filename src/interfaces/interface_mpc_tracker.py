import itertools

import numpy as np
from scipy.spatial import ConvexHull

from basic_motion_model.motion_model import UnicycleModel
from pkg_mpc_tracker.trajectory_tracker import TrajectoryTracker

from configs import MpcConfiguration
from configs import CircularRobotSpecification
from typing import List, Tuple


class MpcInterface:
    def __init__(self, config_mpc: MpcConfiguration, config_robot: CircularRobotSpecification, verbose=True) -> None:
        self.cfg_mpc = config_mpc
        self.cfg_robot = config_robot

        self.traj_tracker = TrajectoryTracker(self.cfg_mpc, self.cfg_robot, verbose=verbose)
        self.traj_tracker.load_motion_model(UnicycleModel(self.cfg_robot.ts))

        self.static_obstacles = None
        self._pred_states = None

    @property
    def state(self) -> np.ndarray:
        return self.traj_tracker.state
    
    @property
    def final_goal(self) -> np.ndarray:
        return self.traj_tracker.final_goal
    
    @property
    def pred_states(self) -> np.ndarray:
        return self._pred_states

    @property
    def nomial_speed(self) -> float:
        return self.traj_tracker.base_speed

    def load_static_obstacles(self, obstacle_list: List[tuple]):
        self.static_obstacles = obstacle_list

    def load_init_states(self, current_state: np.ndarray, goal_state: np.ndarray):
        self.traj_tracker.load_init_states(current_state, goal_state)
    
    def set_current_state(self, current_state: np.ndarray):
        self.traj_tracker.set_current_state(current_state)

    def set_work_mode(self, mode: str):
        self.traj_tracker.set_work_mode(mode)

    def set_ref_states(self, ref_states: np.ndarray, ref_speed:float=None):
        """Set the reference states and speed. If ref_speed is None, use the default speed."""
        self.traj_tracker.set_ref_states(ref_states)
        if ref_speed is not None:
            self.traj_tracker.base_speed = ref_speed

    def check_termination_condition(self, external_check=True) -> bool:
        """Check if the robot has reached the final goal."""
        return self.traj_tracker.check_termination_condition(external_check)

    def run_step(self, full_dyn_obstacle_list:list=None, other_robot_states:list=None, map_updated:bool=True) -> Tuple[List[np.ndarray], List[np.ndarray], float, List[List[tuple]], np.ndarray]:
        """Run one step of MPC.
        
        Returns:
            actions: List of actions to be executed.
            pred_states: List of predicted states.
            cost: Cost of the current step.
            closest_obstacle_list: List of closest obstacles.
            current_refs: List of current reference states.
        """
        # full_dyn_obstacle_list should be [[[T=0],[T=1],[T=2],...], [...], ...]
        if map_updated:
            stc_constraints, closest_obstacle_list = self.get_stc_constraints()
        dyn_constraints = self.get_dyn_constraints(full_dyn_obstacle_list)
        actions, pred_states, current_refs, cost = self.traj_tracker.run_step(stc_constraints, dyn_constraints, other_robot_states)
        self._pred_states = pred_states
        return actions, pred_states, cost, closest_obstacle_list, current_refs

    def get_stc_constraints(self) -> Tuple[list, List[List[tuple]]]:
        n_stc_obs = self.cfg_mpc.Nstcobs * self.cfg_mpc.nstcobs
        stc_constraints = [0.0] * n_stc_obs
        map_obstacle_list = self.get_closest_n_stc_obstacles()
        for i, map_obstacle in enumerate(map_obstacle_list):
            b, a0, a1 = self.polygon_halfspace_representation(np.array(map_obstacle))
            stc_constraints[i*self.cfg_mpc.nstcobs : (i+1)*self.cfg_mpc.nstcobs] = (b+a0+a1)
        return stc_constraints, map_obstacle_list

    def get_dyn_constraints(self, full_dyn_obstacle_list=None):
        params_per_dyn_obs = (self.cfg_mpc.N_hor+1) * self.cfg_mpc.ndynobs
        dyn_constraints = [0.0] * self.cfg_mpc.Ndynobs * params_per_dyn_obs
        if full_dyn_obstacle_list is not None:
            for i, dyn_obstacle in enumerate(full_dyn_obstacle_list):
                dyn_constraints[i*params_per_dyn_obs:(i+1)*params_per_dyn_obs] = list(itertools.chain(*dyn_obstacle))
        return dyn_constraints

    def get_closest_n_stc_obstacles(self) -> List[List[tuple]]:
        short_obs_list = []
        dists_to_obs = []
        if len(self.static_obstacles) <= self.cfg_mpc.Nstcobs:
            return self.static_obstacles
        for obs in self.static_obstacles:
            dists = self.lineseg_dists(self.state[:2], np.array(obs), np.array(obs[1:] + [obs[0]]))
            dists_to_obs.append(np.min(dists))
        selected_idc = np.argpartition(dists_to_obs, self.cfg_mpc.Nstcobs)[:self.cfg_mpc.Nstcobs]
        for i in selected_idc:
            short_obs_list.append(self.static_obstacles[i])
        return short_obs_list

    def get_closest_n_dyn_obstacles(self, full_dyn_obstacle_list) -> List[List[tuple]]:
        pass

    @staticmethod
    def lineseg_dists(points: np.ndarray, line_points_1: np.ndarray, line_points_2: np.ndarray) -> np.ndarray:
        """Cartesian distance from point to line segment
        Edited to support arguments as series, from:
        https://stackoverflow.com/a/54442561/11208892

        Args:
            - p: np.array of shape (n_p, 2)
            - a: np.array of shape (n_l, 2)
            - b: np.array of shape (n_l, 2)
        Return:
            - o: np.array of shape (n_p, n_l)
        """
        p, a, b = points, line_points_1, line_points_2
        if len(p.shape) < 2:
            p = p.reshape(1,2)
        n_p, n_l = p.shape[0], a.shape[0]
        # normalized tangent vectors
        d_ba = b - a
        d = np.divide(d_ba, (np.hypot(d_ba[:, 0], d_ba[:, 1]).reshape(-1, 1)))
        # signed parallel distance components, rowwise dot products of 2D vectors
        s = np.multiply(np.tile(a, (n_p,1)) - p.repeat(n_l, axis=0), np.tile(d, (n_p,1))).sum(axis=1)
        t = np.multiply(p.repeat(n_l, axis=0) - np.tile(b, (n_p,1)), np.tile(d, (n_p,1))).sum(axis=1)
        # clamped parallel distance
        h = np.amax([s, t, np.zeros(s.shape[0])], axis=0)
        # perpendicular distance component, rowwise cross products of 2D vectors  
        d_pa = p.repeat(n_l, axis=0) - np.tile(a, (n_p,1))
        c = d_pa[:, 0] * np.tile(d, (n_p,1))[:, 1] - d_pa[:, 1] * np.tile(d, (n_p,1))[:, 0]
        return np.hypot(h, c).reshape(n_p, n_l)

    @staticmethod
    def polygon_halfspace_representation(polygon_points: np.ndarray):
        """Compute the H-representation of a set of points (facet enumeration).

        Returns:
            A   (L x d) array. Each row in A represents hyperplane normal.
            b   (L x 1) array. Each element in b represents the hyperpalne constant bi
        
        References:
            Link: https://github.com/d-ming/AR-tools/blob/master/artools/artools.py
        """
        hull = ConvexHull(polygon_points)
        hull_center = np.mean(polygon_points[hull.vertices, :], axis=0)  # (1xd) vector

        K = hull.simplices
        V = polygon_points - hull_center # perform affine transformation
        A = np.nan * np.empty((K.shape[0], polygon_points.shape[1]))

        rc = 0
        for i in range(K.shape[0]):
            ks = K[i, :]
            F = V[ks, :]
            if np.linalg.matrix_rank(F) == F.shape[0]:
                f = np.ones(F.shape[0])
                A[rc, :] = np.linalg.solve(F, f)
                rc += 1

        A:np.ndarray = A[:rc, :]
        b:np.ndarray = np.dot(A, hull_center.T) + 1.0
        return b.tolist(), A[:,0].tolist(), A[:,1].tolist()