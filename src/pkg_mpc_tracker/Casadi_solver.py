import sys
import numpy as np
# import matplotlib.pyplot as plt # type: ignore

import casadi as ca # type: ignore
from pkg_mpc_tracker.direct_multiple_shooting import MultipleShootingSolver
import pkg_mpc_tracker.build.mpc_cost as cost


class CasadiSolver:
    def __init__(self, config, robot_specification, params):
        # Define control parameters 
        self.config = config
        self.robot_spec = robot_specification
        self.ns = self.config.ns      #Noumber of states
        self.nu = self.config.nu     #Noumber of inputs
        self.N = self.config.N_hor   #Control horizon
        self.ts = self.config.ts    #Sample time

        ### State bounds of the robot
        self.lin_vel_min = self.robot_spec.lin_vel_min  # Vehicle contraint on the minimal velocity possible
        self.lin_vel_max = self.robot_spec.ang_vel_max   # Vehicle contraint on the maximal velocity possible
        self.ang_vel_min = -self.robot_spec.ang_vel_max  # Vehicle contraint on the maximal angular velocity
        self.ang_vel_max = self.robot_spec.ang_vel_max   # Vehicle contraint on the maximal angular velocity

        # Initial states
        self.x0 = params[2:5]
        self.ref_state = params[5:8]
        
        #Define a static reference
        ref = ca.SX.sym('ref', 3, 1)

        # Create an expression using the symbolic matrix
        new_matrix = ca.vertcat(ref[0], ref[1],ref[2] )

        # Create a CasADi Function
        fun = ca.Function('fun', [ref], [new_matrix])

        # Define a specific 2x1 matrix as a reference
        self.static_ref =fun(self.ref_state)


        print("reference:", self.static_ref)


    def unicycle_model(self, state: ca.SX, action: ca.SX, ts: float, rk4:bool=True) -> ca.SX:
        """Unicycle model.
        
        Args:
            ts: Sampling time.
            state: x, y, and theta.
            action: speed and angular speed.
            rk4: If True, use Runge-Kutta 4 to refine the model.
        """
        def d_state_f(state, action):
            return ts * ca.vertcat(action[0]*ca.cos(state[2]), action[0]*ca.sin(state[2]), action[1])
            
        if rk4:
            k1 = d_state_f(state, action)
            k2 = d_state_f(state + 0.5*k1, action)
            k3 = d_state_f(state + 0.5*k2, action)
            k4 = d_state_f(state + k3, action)
            d_state = (1/6) * (k1 + 2*k2 + 2*k3 + k4)
        else:
            d_state = d_state(state, action)

        return state + d_state

    def return_continuous_function(self) -> ca.Function:
        """
        xc: x1 - distance [km], x2 - speed [km/h]
        uc: uc_neg & uc_pos, actuator acceleration [m/s^2]
        """
        x = ca.SX.sym('x', self.ns)
        u = ca.SX.sym('u', self.nu)
        f = self.unicycle_model(x, u, self.ts)
        J_obj = cost.cost_refstate_deviation(x, self.static_ref)
        cost_obj = J_obj[0]+J_obj[1]
        fk = ca.Function('fk', [x, u], [f, cost_obj])
        return fk, cost_obj

    def run(self):
        # Define a symbolic continious function
        fc, cost_obj = self.return_continuous_function()

        # Discretize the continious function 
        ms_solver = MultipleShootingSolver(self.ns, self.nu, self.ts, self.N)
        ms_solver.set_initial_state(self.x0)
        ms_solver.set_motion_model(fc, c2d=False)

        # Define state and output bounds
        ms_solver.set_control_bound([self.lin_vel_min, self.ang_vel_min], [self.lin_vel_max, self.ang_vel_max])
        ms_solver.set_state_bound([[0.0,0.0,0.0]]*(self.N+1), 
                                [[20.0,20.0,2*ca.pi]]*(self.N+1))
        ms_solver.build_problem()
        sol, solver_time, exit_status, solver_cost = ms_solver.solve()

        u_out = [output[0] for output in ms_solver.get_opt_controls(sol)]
        
        return u_out, solver_cost, exit_status, solver_time




