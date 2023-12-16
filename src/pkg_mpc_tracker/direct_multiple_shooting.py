import sys

import numpy as np # type: ignore
import matplotlib.pyplot as plt # type: ignore
from opengen import opengen as og

import casadi as ca # type: ignore

from typing import Callable, Union, Optional, Any, cast
from pkg_mpc_tracker.build import mpc_cost
from basic_motion_model import motion_model
from pkg_mpc_tracker.build import mpc_helper




'''
This is used for solving problems with a horizon.
'''

### NLP
class MultipleShootingSolver:
    """Build a NLP problem with a horizon (i.e. MPC) in the form of multiple shooting.
    
    Mathematical form:

    ```
        min_{w} J(w) 
        s.t.    g(w) = 0, 
                h(w) <= 0
    ```

    `w` - decision variables, `g` - equality constraints, `h` - inequality constraints

    Comments:
        A variable with a subscript `_i` means it is a symbol, not a value (e.g. `x_0`).
        A variable with a number `i` means it is a value, not a symbol (e.g. `x0`). 
    """
    def __init__(self, ns: int, nu: int, ts: float, horizon: int, config, robot_specification, initial_guess) -> None:
        """
        Arguments:
            ns: number of states
            nu: number of controls
            horizon: the length of the horizon
        """
        self._ns = ns
        self._nu = nu
        self._ts = ts
        self._N = horizon
        self.config = config
        self.robot_config = robot_specification
        self.initial_guess = initial_guess

        self._create_placeholders()
        self._x0: Any = None    #Check this one out
        self._f_func: Any = None
        self._lbx = [[-ca.inf]*self._ns]*(self._N+1)
        self._ubx = [[ ca.inf]*self._ns]*(self._N+1)
        self._lbu = [[-ca.inf]*self._nu]*self._N
        self._ubu = [[ ca.inf]*self._nu]*self._N

        self._eq_func_list:list = []
        self._ineq_func_list:list = []

        self.problem: Optional[dict] = None
        self._built = False

    def _create_placeholders(self) -> None:
        """Create placeholders (variable and constraint lists) for the problem."""
        self._w_list: list[ca.SX] = [] # decision variables
        self._g_list: list[ca.SX] = [] # equality constraints
        self._h_list: list[ca.SX] = [] # inequality constraints

        self._lbw_list: list[float] = [] # lower bound of decision variables
        self._ubw_list: list[float] = [] # upper bound of decision variables
        self._lbg_list: list[float] = [] # lower bound of equality constraints
        self._ubg_list: list[float] = [] # upper bound of equality constraints
        self._lbh_list: list[float] = [] # lower bound of inequality constraints
        self._ubh_list: list[float] = [] # upper bound of inequality constraints

    def _build_check(self) -> None:
        """Check if the problem can be built. Raise `error` if not."""
        if self._x0 is None:
            raise RuntimeError("Initial state is not set.")
        if self._f_func is None:
            raise RuntimeError("Motion model is not set.")

    @property
    def ns(self) -> int:
        return self._ns
    
    @property
    def nu(self) -> int:
        return self._nu

    @property
    def ts(self) -> float:
        if not isinstance(self._ts, float):
            raise RuntimeError("The time step is not fixed.")
        return self._ts

    @property
    def N(self) -> int:
        return self._N

    def set_initial_state(self, x0: list[float]) -> None:
        self._x0 = x0

    def set_ref_states(self, ref_states: np.ndarray) -> np.ndarray:
        """Set the local reference states for the coming time step.

        Arguments:
            ref_states: Local (within the horizon) reference states
        """
        self.ref_states = ref_states

    def set_state_bound(self, lbx: list, ubx: list) -> None:
        """Set the lower and upper bounds of the states.
        
        Argument types:
            1: list[float], Each value is for each state, the whole list will repeat N_horizon times.
            2: list[list[float]], Each sub-list is for a time step in the N_horizon.
        """
        if not isinstance(lbx[0], list):
            self._lbx = [lbx] * (self._N+1)
        else:
            self._lbx = lbx
        if not isinstance(ubx[0], list):
            self._ubx = [ubx] * (self._N+1)
        else:
            self._ubx = ubx
            
    def set_stcobs_constraints(self):
        print('hej')
        
        
    def set_control_bound(self, lbu: list, ubu: list) -> None:
        """Set the lower and upper bounds of the controls."""
        if not isinstance(lbu[0], list):
            self._lbu = [lbu] * self._N
        else:
            self._lbu = lbu
        if not isinstance(ubu[0], list):
            self._ubu = [ubu] * self._N
        else:
            self._ubu = ubu

    def set_motion_model(self, func: Union[ca.Function, Callable], c2d:bool=False, sub_sampling:int=0) -> None:
        """Set the motion model of the system, which should be a mapping:
        
        ```
            (x_next, cost_now) = func(x_now, u_now)
        ```

        Arguments:
            func: the motion model of the system.
            c2d: whether the motion model is continuous or discrete. Default: False (discrete).
            sub_sampling: (if c2d is true) the number of sub-samples in each time interval.
        """
        if c2d:
            self._f_func = self.return_discrete_function(func, self._ns, self._nu, self._ts, sub_sampling=sub_sampling)
        else:
            self._f_func = func

    def add_equality_constraint(self, func: Union[ca.Function, Callable]) -> None:
        if self._built:
            raise RuntimeError("The problem has already been built. Cannot add more constraints.")
        pass # TODO

    def add_inequality_constraint(self, func: Union[ca.Function, Callable]) -> None:
        if self._built:
            raise RuntimeError("The problem has already been built. Cannot add more constraints.")
        self._ineq_func_list.append(func)

    def set_parameters(self,params):
        """
            Set the parameter value of all the help variables such as the reference states.
        """
        self.u_m1= params[0:2]          # Input at kt=-1
        self.s_0 = params[2:5]          # State at kt=0
        self.s_N = params[5:8]          # State of goal at kt=N_hor
        self.q = params[8:18]           # Penalty parameters for objective terms
        self.r_s = params[18:78]        # Reference states
        self.r_v = params[78:98]        # Reference speed
        self.c_0 = params[98:128]       # States of other robots at kt=0
        self.c = params[98:728]         # Predicted states of other robots
        self.o_s = params[728:848]      # Static obstacles
        self.q_stc = params[-40:-20]    # Static obstacle weights

    def cost_calculation(self):
        """Calculates the cost function using the states and inputs to minimize the cost:

        Returns:
            Cost: The total cost function used for the Casadi solver 
        """
        Cost_dict = {}
        
        (x, y, theta) = self.s_0
        (x_goal, y_goal, theta_goal) = self.s_N
        (v_init, w_init) = self.u_m1
        (qpos, qvel, qtheta, rv, rw, qN, qthetaN, qrpd, acc_penalty, w_acc_penalty) = self.q
        
        ref_states = ca.reshape(self.r_s, (self.ns, self.N)).T
        ref_states = ca.vertcat(ref_states, ref_states[[-1],:])[:, :2] #Remove theta
        
        #Intialize all costs
        cost = 0
        cost_ref_path = 0
        cost_refvalue_deviation_v = 0
        cost_refvalue_deviation_w = 0
        penalty_constraints = 0
        cost_control_actions = 0
        cost_fleet_collision = 0
        cost_fleet_collision_pred = 0
        cost_static_obstacles = 0
        Terminal_cost = 0
        cost_linear_acc = 0
        cost_angular_acc = 0


        for kt in range(0, self.N): # LOOP OVER PREDICTIVE HORIZON
            
            state_vec = self._w_list[2*kt]
            u_t = self._w_list[2*kt+1]
            
            ### Run step with motion model
            state = self._f_func(state_vec, u_t, self.ts) # Kinematic/dynamic model

            ### Reference deviation costs
            cost_ref_path += mpc_cost.cost_refpath_deviation(state.T, ref_states[kt:, :], weight=qrpd)   # [cost] reference path deviation cost
            cost_refvalue_deviation_v += mpc_cost.cost_refvalue_deviation(u_t[0], self.r_v[kt], weight=qvel)         # [cost] refenrence velocity deviation
            cost_refvalue_deviation_w += mpc_cost.cost_refvalue_deviation(u_t[1], 0, weight=qvel)                    # [cost] Angular velocity usage
            cost_control_actions += mpc_cost.cost_control_actions(u_t.T, weights=ca.horzcat(rv, rw))            # [cost] penalize control actions

            
            ### Fleet collision avoidance
            other_x_0 = self.c_0[ ::self.ns] # first  state
            other_y_0 = self.c_0[1::self.ns] # second state
            other_robots_0 = ca.hcat([other_x_0, other_y_0]) # states of other robots at time 0
            other_robots_0 = ca.transpose(other_robots_0) # every column is a state of a robot
            cost_fleet_collision += mpc_cost.cost_fleet_collision(state[:2].T, other_robots_0.T, 
                                                  safe_distance=2*(self.robot_config.vehicle_width+self.robot_config.vehicle_margin), weight=1000)
            
            ### Fleet collision avoidance [Predictive]
            other_robots_x = self.c[kt*self.ns  ::self.ns*self.N] # first  state
            other_robots_y = self.c[kt*self.ns+1::self.ns*self.N] # second state
            other_robots = ca.hcat([other_robots_x, other_robots_y]) # states of other robots at time kt (Nother*ns)
            other_robots = ca.transpose(other_robots) # every column is a state of a robot
            cost_fleet_collision_pred += mpc_cost.cost_fleet_collision(state[:2].T, other_robots.T, 
                                                  safe_distance=2*(self.robot_config.vehicle_width+self.robot_config.vehicle_margin), weight=10)
            
            ### Static obstacles
            for i in range(self.config.Nstcobs):
                eq_param = self.o_s[i*self.config.nstcobs : (i+1)*self.config.nstcobs]
                n_edges = int(self.config.nstcobs / 3) # 3 means b, a0, a1
                b, a0, a1 = ca.SX(eq_param[:n_edges]), ca.SX(eq_param[n_edges:2*n_edges]), ca.SX(eq_param[2*n_edges:])

                inside_stc_obstacle = mpc_helper.inside_cvx_polygon(state.T, b.T, a0.T, a1.T)
                penalty_constraints += ca.fmax(0, ca.vertcat(inside_stc_obstacle))
                cost_static_obstacles += mpc_cost.cost_inside_cvx_polygon(state.T, b.T, a0.T, a1.T, weight=self.q_stc[kt])
        
        ### Terminal cost
        Terminal_cost += qN*((state[0]-x_goal)**2 + (state[1]-y_goal)**2) + qthetaN*(state[2]-theta_goal)**2 # terminated cost  
        
        ### Max speed bound
        umin = [self.robot_config.lin_vel_min, -self.robot_config.ang_vel_max] * self.N
        umax = [self.robot_config.lin_vel_max,  self.robot_config.ang_vel_max] * self.N
        
        ### Acceleration bounds and cost
        v = u_t[0::2] # velocity
        w = u_t[1::2] # angular velocity
        acc   = (v-ca.vertcat(v_init, v[0:-1]))/self.ts
        w_acc = (w-ca.vertcat(w_init, w[0:-1]))/self.ts
        
        # Acceleration bounds
        acc_min   = [ self.robot_config.lin_acc_min] * self.N 
        w_acc_min = [-self.robot_config.ang_acc_max] * self.N
        acc_max   = [ self.robot_config.lin_acc_max] * self.N
        w_acc_max = [ self.robot_config.ang_acc_max] * self.N
        
        # Accelerations cost
        cost_linear_acc += ca.mtimes(acc.T, acc)*acc_penalty
        cost_angular_acc += ca.mtimes(w_acc.T, w_acc)*w_acc_penalty

         ### Add all costs     
        cost = cost_ref_path + cost_refvalue_deviation_v + cost_refvalue_deviation_w + cost_control_actions + cost_fleet_collision + cost_fleet_collision_pred + cost_static_obstacles + Terminal_cost + cost_linear_acc + cost_angular_acc
        
        ### Add all costs to dictionary
        Cost_dict.update({'ref_path': cost_ref_path})
        Cost_dict.update({'v_input_deviation': cost_refvalue_deviation_v})
        Cost_dict.update({'w_input_deviation': cost_refvalue_deviation_w})
        Cost_dict.update({'control_action': cost_control_actions})
        Cost_dict.update({'fleet_collision': cost_fleet_collision})
        Cost_dict.update({'fleet_collision_predictive': cost_fleet_collision_pred})
        Cost_dict.update({'static_obstacles': cost_static_obstacles})
        Cost_dict.update({'Terminal_cost': Terminal_cost})
        Cost_dict.update({'linear_acceleration': cost_linear_acc})
        Cost_dict.update({'angular_acceleration': cost_angular_acc})

                
        return cost, Cost_dict

    def build_problem(self) -> dict:
        """Build the NLP problem in the form of multiple shooting.
        
        Returns:
            problem: a dictionary (x, f, g, lbx, ubx, lbg, ubg) containing the NLP problem.
        """
        self._build_check()

        self._w_list.append(ca.SX.sym('x_0', self._ns))
        self._lbw_list += self._lbx[0]
        self._ubw_list += self._ubx[0]
        self._g_list.append(self._w_list[0] - self._x0)
        self._lbg_list += [0]*self._ns
        self._ubg_list += [0]*self._ns

        x_k = self._w_list[0]
        J = 0 # objective
        for k in range(self._N):
            x_next = ca.SX.sym('x_' + str(k+1), self._ns)
            u_k = ca.SX.sym('u_' + str(k), self._nu)
            
            fk = ca.Function('x_next_hat', [x_k,u_k], [self._f_func(x_k,u_k,self.ts)])
            x_next_hat = fk(x_k,u_k)

            self._w_list += [u_k, x_next]
            self._lbw_list += self._lbu[k] + self._lbx[k+1]
            self._ubw_list += self._ubu[k] + self._ubx[k+1]

            self._g_list += [x_next - x_next_hat]
            self._lbg_list += [0]*self._ns
            self._ubg_list += [0]*self._ns

            for ineq_func in self._ineq_func_list:
                self._h_list += [ineq_func(x_k, u_k)]
                self._lbh_list += [-ca.inf]
                self._ubh_list += [0]

           
            x_k = x_next
        
        J, Cost_dict = self.cost_calculation()
        constraint_list = self._g_list + self._h_list
        constraint_lb_list = self._lbg_list + self._lbh_list
        constraint_ub_list = self._ubg_list + self._ubh_list
        
        # Added parameters to problem definition
        self.problem = {'f': J, 'x': ca.vertcat(*self._w_list), 'g': ca.vertcat(*constraint_list),
                        'lbx': self._lbw_list, 'ubx': self._ubw_list,
                        'lbg': constraint_lb_list, 'ubg': constraint_ub_list}
        self._built = True
        return self.problem, Cost_dict

    # Solver option will supress printout from the solver if not None
    def solve(self, solver_type:str='ipopt', solver_options:Optional[dict]={'ipopt.print_level':0, 'print_time':5},
              build_kwargs:Optional[dict]=None, run_kwargs:Optional[dict]=None) -> None:
        """Solve the NLP problem.
        
        Arguments:
            solver_type: the solver to use. Default: `ipopt`.
            build_kwargs: keyword arguments for build the solver.
            run_kwargs: keyword arguments for run the solver.
        
        Returns:
            sol: the solution of the NLP problem.
        """
        if not self._built:
            self.build_problem()
        self.problem = cast(dict, self.problem)
        problem = {k: self.problem[k] for k in ('f', 'x', 'g')}


        if solver_options is None:
            solver_options = {}
        if build_kwargs is None:
            build_kwargs ={}
        if run_kwargs is None:
            run_kwargs = {} 

        print(self.initial_guess) 
        solver = ca.nlpsol('solver', solver_type, problem, solver_options, **build_kwargs)
        sol: dict = solver(x0 = self.initial_guess, lbx=self.problem['lbx'], ubx=self.problem['ubx'],
                           lbg=self.problem['lbg'], ubg=self.problem['ubg'],**run_kwargs)
        sol_stats = solver.stats()
        time = 1000*sol_stats['t_wall_total']
        exit_status = sol_stats['return_status']
        sol_cost = float(sol['f'])
        return sol, time, exit_status, sol_cost
    
    def get_pred_states(self, sol: dict) -> list[list[float]]:
        """Get the predicted states from the solution.
        
        Arguments:
            sol: the solution of the NLP problem.
        
        Returns:
            x_pred: a list of predicted states. Each row is a state.
        """
        sol_x = sol['x']
        nu_factor = 1

        x_pred = []
        for i in range(self._ns):
            xi_pred:ca.DM = sol_x[i::(self._ns+self._nu*nu_factor)]
            xi_pred_np:np.ndarray = xi_pred.full()
            x_pred.append(xi_pred_np.flatten().tolist())
        return x_pred
    
    def get_opt_controls(self, sol: dict) -> list[list[float]]:
        """Get the optimal controls from the solution.
        
        Arguments:
            sol: the solution of the NLP problem.
        
        Returns:
            u_opt: a list of optimal controls. Each row is a control.
        """
        sol_x = sol['x']
        nu_factor = 1

        u_opt = []
        for i in range(self._nu*nu_factor):
            ui_opt:ca.DM = sol_x[self._ns+i::(self._ns+self._nu*nu_factor)]
            ui_opt_np:np.ndarray = ui_opt.full()
            u_opt.append(ui_opt_np.flatten().tolist())
        return u_opt

    @staticmethod
    def return_discrete_function(func_c: ca.Function, ns: int, nu: int, ts: float, 
                                method:str='rk4', sub_sampling:int=0) -> ca.Function:
        """Return a discrete function from a continuous function.
        
        Arguments:
            method: the method to use for discretization. Default/only: `rk4`.
            sub_sampling: the number of sub-sampling steps in each time interval. Default: 0.
        """
        x = ca.SX.sym('x', ns)
        u = ca.SX.sym('u', nu)
        M = sub_sampling + 1
        dt = ts/M # if sub_sampling == 0, dt = ts

        x_next = x
        J_next = 0
        for _ in range(M):
            k1, k1_q = func_c(x_next, u)
            k2, k2_q = func_c(x_next + dt/2*k1, u)
            k3, k3_q = func_c(x_next + dt/2*k2, u)
            k4, k4_q = func_c(x_next + dt*k3, u)
            x_next += dt/6*(k1 + 2*k2 + 2*k3 + k4)
            J_next += dt/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
        f = ca.Function('f', [x, u], [x_next, J_next])
        return f
    

if __name__ == '__main__':
    ### Parameters
    T = 10 # total time
    N = 50 # number of control steps
    ts = T/N
    x0 = [0.0, 1.0]

    ### Continuous time
    def return_continuous_function(ns: int, nu: int) -> ca.Function:
        xc = ca.SX.sym('xc', ns)
        uc = ca.SX.sym('uc', nu)
        xc_dot = ca.vertcat((1-xc[1]**2)*xc[0] - xc[1] + uc, xc[0])
        Jc_obj = xc[0]**2 + xc[1]**2 + uc**2
        fc = ca.Function('fc', [xc, uc], [xc_dot, Jc_obj])
        return fc

    fc = return_continuous_function(2, 1)

    ms_solver = MultipleShootingSolver(2, 1, ts, N)
    ms_solver.set_initial_state(x0)
    ms_solver.set_motion_model(fc, c2d=True, sub_sampling=3)
    ms_solver.set_control_bound([-1.0], [1.0])
    ms_solver.build_problem()
    sol = ms_solver.solve()

    # Plot the solution
    u_opt = ms_solver.get_opt_controls(sol)
    x_pred = ms_solver.get_pred_states(sol)

    tgrid = [T/N*k for k in range(N+1)]
    plt.figure(1)
    plt.clf()
    plt.plot(tgrid, x_pred[0], '--')
    plt.plot(tgrid, x_pred[1], '-')
    plt.step(tgrid, ca.vertcat(ca.DM.nan(1), ca.DM(u_opt[0])), '-.')
    plt.xlabel('t')
    plt.legend(['x1','x2','u'])
    plt.grid()
    plt.show()


