# System import
import os
import sys
import math
# External import
import numpy as np
# Custom import 
from configs import MpcConfiguration, CircularRobotSpecification
# Type hint
from typing import Callable, List


class Solver(): # this is not found in the .so file (in ternimal: nm -D  navi_test.so)
    import opengen as og
    def run(self, p: list, initial_guess, initial_lagrange_multipliers, initial_penalty) -> og.opengen.tcp.solver_status.SolverStatus: pass


class TrajectoryTracker:
    """Generate a smooth trajectory tracking based on the reference path and obstacle information.
    
    Attributes:
        config: MPC configuration.
        robot_spec: Robot specification.

    Functions:
        run_step: Run one step of trajectory tracking.

    Comments:
        The solver needs to be built before running the trajectory tracking. \n
        To run the tracker: 
            1. Load motion model and init states; 
            2. Set reference path and trajectory (and states maybe);
            3. Run step.
    """
    def __init__(self, config: MpcConfiguration, robot_specification: CircularRobotSpecification, use_tcp:bool=False, verbose=False):
        self.vb = verbose
        self.config = config
        self.robot_spec = robot_specification

        # Common used parameters from config
        self.ts = self.config.ts
        self.ns = self.config.ns
        self.nu = self.config.nu
        self.N_hor = self.config.N_hor

        # Initialization
        self._idle = True
        self._obstacle_weights()
        self.set_work_mode(mode='safe')

        self.__import_solver(use_tcp=use_tcp)

    def __import_solver(self, root_dir:str='', use_tcp:bool=False):
        self.use_tcp = use_tcp
        solver_path = os.path.join(root_dir, self.config.build_directory, self.config.optimizer_name)

        import opengen as og
        if not use_tcp:
            sys.path.append(solver_path)
            built_solver = __import__(self.config.optimizer_name) # it loads a .so (shared library object)
            self.solver:Solver = built_solver.solver() # Return a Solver object with run method, cannot find it though
        else: # use TCP manager to access solver
            self.mng:og.opengen.tcp.OptimizerTcpManager = og.tcp.OptimizerTcpManager(solver_path)
            self.mng.start()
            self.mng.ping() # ensure RUST solver is up and runnings

    def _obstacle_weights(self):
        """
        Attributes:
            stc_weights [list]: penalty weights for static obstacles (only useful if soft constraints activated)
            dyn_weights [list]: penalty weights for dynamic obstacles (only useful if soft constraints activated)
        """
        if isinstance(self.config.qstcobs, list):
            self.stc_weights = self.config.qstcobs
        elif isinstance(self.config.qstcobs, (float,int)):
            self.stc_weights = [self.config.qstcobs]*self.N_hor
        else:
            raise TypeError(f'Unsupported datatype for obstacle weights, got {type(self.config.qstcobs)}.')
        if isinstance(self.config.qdynobs, list):
            self.dyn_weights = self.config.qdynobs
        elif isinstance(self.config.qdynobs, (float,int)):
            self.dyn_weights = [self.config.qdynobs]*self.N_hor
        else:
            raise TypeError(f'Unsupported datatype for obstacle weights, got {type(self.config.qdynobs)}.')

    @property
    def idle(self):
        return self._idle

    def load_motion_model(self, motion_model: Callable) -> None:
        """The motion model should be `s'=f(s,a,ts)` (takes in a state and an action and returns the next state).
        """
        self.motion_model = motion_model

    def load_init_states(self, current_state: np.ndarray, goal_state: np.ndarray):
        """Load the initial state and goal state.

        Arguments:
            current_state: Current state of the robot.
            goal_state: Goal state of the robot (used to decelerate if close to goal).

        Attributes:
            state: Current state of the robot.
            final_goal: Goal state of the robot.
            past_states: List of past states of the robot.
            past_actions: List of past actions of the robot.
            cost_timelist: List of cost values of the robot.
            solver_time_timelist: List of solver time of the robot.

        Comments:
            This function resets the `idx_ref_traj/path` to 0 and `idle` to False.
        """
        if (not isinstance(current_state, np.ndarray)) or (not isinstance(goal_state, np.ndarray)):
            raise TypeError(f'State should be numpy.ndarry, got {type(current_state)}/{type(goal_state)}.')
        self.state = current_state
        self.final_goal = goal_state

        self.past_states  = []
        self.past_actions: List[np.ndarray] = []
        self.cost_timelist = []
        self.solver_time_timelist = []

        self._idle = False
        self.finishing = False # If approaching the last node of the reference path

    def set_work_mode(self, mode:str='safe'): # change base speed and tuning parameters
        """Set the basic work mode (base speed and weight parameters) of the MPC solver.
        Arguments:
            `mode`: "aligning" (start) or "safe" (20% speed) or "work" (80% speed) or "super" (full speed)
        Attributes:
            `base_speed`: The reference speed
            `tuning_params`: Penalty parameters for MPC
        """
        ### Base/reference speed
        if mode == 'aligning':
            self.base_speed = self.robot_spec.lin_vel_max*0.5
            self.tuning_params = [0.0] * self.config.nq
            self.tuning_params[2] = 100
        else:
            self.tuning_params = [self.config.qpos, self.config.qvel, self.config.qtheta, self.config.lin_vel_penalty, self.config.ang_vel_penalty,
                                  self.config.qpN, self.config.qthetaN, self.config.qrpd, self.config.lin_acc_penalty, self.config.ang_acc_penalty]
            if mode == 'safe':
                self.base_speed = self.robot_spec.lin_vel_max*0.2
            elif mode == 'work':
                self.base_speed = self.robot_spec.lin_vel_max*0.8
            elif mode == 'super':
                self.base_speed = self.robot_spec.lin_vel_max*1.0
            else:
                raise ModuleNotFoundError(f'There is no mode called {mode}.')

    def set_current_state(self, current_state: np.ndarray):
        """To synchronize the current state of the robot with the MPC solver."""
        if not isinstance(current_state, np.ndarray):
            raise TypeError(f'State should be numpy.ndarry, got {type(current_state)}.')
        self.state = current_state

    def set_ref_states(self, ref_states: np.ndarray) -> np.ndarray:
        """Set the local reference states for the coming time step.

        Arguments:
            ref_states: Local (within the horizon) reference states
        """
        self.ref_states = ref_states

    def check_termination_condition(self, external_check=True) -> bool:
        if external_check:
            self.finishing = True
            if np.allclose(self.state[:2], self.final_goal[:2], atol=0.1, rtol=0) and abs(self.past_actions[-1][0]) < 0.1:
                self._idle = True
                if self.vb:
                    print(f"{self.__class__.__name__} MPC solution found.")
        return self._idle


    def run_step(self, stc_constraints: list, dyn_constraints: list, other_robot_states:list=None):
        """Run the trajectory planner for one step.

        Arguments:
            other_robot_states: A list with length "ns*N_hor*Nother" (E.x. [0,0,0] * (self.N_hor*self.config.Nother))
        
        Returns:
            actions: A list of future actions
            pred_states: A list of predicted states
            ref_states: Reference states
            cost: The cost of the predicted trajectory
        """
        if stc_constraints is None:
            stc_constraints = [0] * (self.config.Nstcobs*self.config.nstcobs)
        if dyn_constraints is None:
            dyn_constraints = [0] * (self.config.Ndynobs*self.config.ndynobs*(self.N_hor+1))
        if other_robot_states is None:
            other_robot_states = [-10] * (self.ns*(self.N_hor+1)*self.config.Nother)

        ### Get reference states ###
        ref_states = self.ref_states.copy()
        finish_state = ref_states[-1,:]
        current_refs = ref_states.reshape(-1).tolist()

        ### Get reference velocities ###
        dist_to_goal = math.hypot(self.state[0]-self.final_goal[0], self.state[1]-self.final_goal[1]) # change ref speed if final goal close
        if (dist_to_goal < self.base_speed*self.N_hor*self.ts) and self.finishing:
            speed_ref = dist_to_goal / self.N_hor / self.ts
            speed_ref = min(speed_ref, self.robot_spec.lin_vel_max)
            speed_ref_list = [speed_ref]*self.N_hor
        else:
            speed_ref_list = [self.base_speed]*self.N_hor

        last_u = self.past_actions[-1] if len(self.past_actions) else np.zeros(self.nu)
            
        ### Assemble parameters for solver & Run MPC###
        params = list(last_u) + list(self.state) + list(finish_state) + self.tuning_params + \
                 current_refs + speed_ref_list + other_robot_states + \
                 stc_constraints + dyn_constraints + self.stc_weights + self.dyn_weights
        # print("last_u", list(last_u), '\n',
        #       "state", list(self.state), '\n',
        #       "finish_state", list(finish_state), '\n',
        #       "tuning_params", self.tuning_params, '\n',
        #       "current_refs", current_refs, '\n',
        #       "speed_ref_list", speed_ref_list, '\n',
        #       "other_robot_states", other_robot_states, '\n',
        #     #   "stc_constraints", stc_constraints, '\n',
        #     #   "dyn_constraints", dyn_constraints, '\n',
        #       "stc_weights", self.stc_weights, '\n',
        #       "dyn_weights", self.dyn_weights, '\n',
        # )
        # print("last_u", len(list(last_u)), '\n',
        #       "state", len(list(self.state)), '\n',
        #       "finish_state", len(list(finish_state)), '\n',
        #       "tuning_params", len(self.tuning_params), '\n',
        #       "current_refs", len(current_refs), '\n',
        #       "speed_ref_list", len(speed_ref_list), '\n',
        #       "other_robot_states", len(other_robot_states), '\n',
        #       "stc_constraints", len(stc_constraints), '\n',
        #       "dyn_constraints", len(dyn_constraints), '\n',
        #       "stc_weights", len(self.stc_weights), '\n',
        #       "dyn_weights", len(self.dyn_weights), '\n',
        # )

        try:
            # self.solver_debug(stc_constraints) # use to check (visualize) the environment
            taken_states, pred_states, actions, cost, solver_time, exit_status = self.run_solver(params, self.state, self.config.action_steps)
        except RuntimeError as err:
            if self.use_tcp:
                self.mng.kill()
            print(f"Fatal: Cannot run solver. {err}.")
            return -1

        self.past_states.append(self.state)
        self.past_states += taken_states[:-1]
        self.past_actions += actions
        self.state = taken_states[-1]
        self.cost_timelist.append(cost)
        self.solver_time_timelist.append(solver_time)

        if exit_status in self.config.bad_exit_codes and self.vb:
            print(f"{self.__class__.__name__} Bad converge status: {exit_status}")

        return actions, pred_states, ref_states, cost

    def run_solver(self, parameters:list, state: np.ndarray, take_steps:int=1):
        """ Run the solver for the pre-defined MPC problem.

        Arguments:
            parameters: All parameters used by MPC, defined in 'build'.
            state: The current state.
            take_steps: The number of control step taken by the input (default 1).

        Returns:
            taken_states: List of taken states, length equal to take_steps.
            pred_states: List of predicted states at this step, length equal to horizon N.
            actions: List of taken actions, length equal to take_steps.
            cost: The cost value of this step
            solver_time: Time cost for solving MPC of the current time step
            exit_status: The exit state of the solver.

        Comments:
            The motion model (dynamics) is defined initially.
        """
        if self.use_tcp:
            return self.run_solver_tcp(parameters, state, take_steps)

        import opengen as og
        solution:og.opengen.tcp.solver_status.SolverStatus = self.solver.run(parameters)
        
        u           = solution.solution
        cost        = solution.cost
        exit_status = solution.exit_status
        solver_time = solution.solve_time_ms
        
        taken_states:List[np.ndarray] = []
        for i in range(take_steps):
            state_next = self.motion_model(state, np.array(u[(i*self.nu):((i+1)*self.nu)]), self.ts)
            taken_states.append(state_next)

        pred_states:List[np.ndarray] = [taken_states[-1]]
        for i in range(len(u)//self.nu):
            pred_state_next = self.motion_model(pred_states[-1], np.array(u[(i*self.nu):(2+i*self.nu)]), self.ts)
            pred_states.append(pred_state_next)
        pred_states = pred_states[1:]

        actions = u[:self.nu*take_steps]
        actions = np.array(actions).reshape(take_steps, self.nu).tolist()
        actions = [np.array(action) for action in actions]
        return taken_states, pred_states, actions, cost, solver_time, exit_status

    def run_solver_tcp(self, parameters:list, state: np.ndarray, take_steps:int=1):
        solution = self.mng.call(parameters)
        if solution.is_ok():
            # Solver returned a solution
            solution_data = solution.get()
            u           = solution_data.solution
            cost        = solution_data.cost
            exit_status = solution_data.exit_status
            solver_time = solution_data.solve_time_ms
        else:
            # Invocation failed - an error report is returned
            solver_error = solution.get()
            error_code = solver_error.code
            error_msg = solver_error.message
            self.mng.kill() # kill so rust code wont keep running if python crashes
            raise RuntimeError(f"MPC Solver error: [{error_code}]{error_msg}")

        taken_states:List[np.ndarray] = []
        for i in range(take_steps):
            state_next = self.motion_model( state, np.array(u[(i*self.nu):((i+1)*self.nu)]), self.ts )
            taken_states.append(state_next)

        pred_states:List[np.ndarray] = [taken_states[-1]]
        for i in range(len(u)//self.nu):
            pred_state_next = self.motion_model( pred_states[-1], np.array(u[(i*self.nu):(2+i*self.nu)]), self.ts )
            pred_states.append(pred_state_next)
        pred_states = pred_states[1:]

        actions = u[:self.nu*take_steps]
        actions = np.array(actions).reshape(take_steps, self.nu).tolist()
        actions = [np.array(action) for action in actions]
        return taken_states, pred_states, actions, cost, solver_time, exit_status
    



