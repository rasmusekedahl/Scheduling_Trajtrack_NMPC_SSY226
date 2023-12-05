import sys
import numpy as np
# import matplotlib.pyplot as plt # type: ignore

import casadi as cs # type: ignore
from pkg_mpc_tracker.direct_multiple_shooting import MultipleShootingSolver
from pkg_mpc_tracker.build import mpc_cost




class CasadiSolver:
    def __init__(self, config, robot_specification, params):
        # Define control parameters 
        self.config = config
        self.robot_spec = robot_specification
        self.ns = self.config.ns      #Noumber of states
        self.nu = self.config.nu     #Noumber of inputs
        self.N = self.config.N_hor   #Control horizon
        self.ts = self.config.ts    #Sample time
        self.params = params

        ### State bounds of the robot
        self.lin_vel_min = self.robot_spec.lin_vel_min   # Vehicle contraint on the minimal velocity possible
        self.lin_vel_max = self.robot_spec.lin_vel_max   # Vehicle contraint on the maximal velocity possible
        self.ang_vel_min = -self.robot_spec.ang_vel_max  # Vehicle contraint on the maximal angular velocity
        self.ang_vel_max = self.robot_spec.ang_vel_max   # Vehicle contraint on the maximal angular velocity

        # Initial states
        self.x0 = params[2:5]
        self.ref_state = params[5:8]
        
       


    def unicycle_model(self, state: cs.SX, action: cs.SX, ts: float, rk4:bool=True) -> cs.SX:
        """Unicycle model.
        
        Args:
            ts: Sampling time.
            state: x, y, and theta.
            action: speed and angular speed.
            rk4: If True, use Runge-Kutta 4 to refine the model.
        """
        def d_state_f(state, action):
            return ts * cs.vertcat(action[0]*cs.cos(state[2]), action[0]*cs.sin(state[2]), action[1])
            
        if rk4:
            k1 = d_state_f(state, action)
            k2 = d_state_f(state + 0.5*k1, action)
            k3 = d_state_f(state + 0.5*k2, action)
            k4 = d_state_f(state + k3, action)
            d_state = (1/6) * (k1 + 2*k2 + 2*k3 + k4)
        else:
            d_state = d_state(state, action)

        return state + d_state
    
    def return_continuous_function(self) -> cs.Function:
        
        x = cs.SX.sym('x', self.ns)
        u = cs.SX.sym('u', self.nu)
        f = self.unicycle_model(x, u, self.ts)
       
        fk = cs.Function('fk', [x,u], [f])
        return fk
    

    def run(self):
        # Define a symbolic continious function
        fk = self.return_continuous_function()

        # Discretize the continious function 
        ms_solver = MultipleShootingSolver(self.ns, self.nu, self.ts, self.N, self.config,self.robot_spec)
        ms_solver.set_initial_state(self.x0)
        ms_solver.set_motion_model(self.unicycle_model,c2d=False)
        ms_solver.set_parameters(self.params)
        

        # Define state and output bounds
        ms_solver.set_control_bound([self.lin_vel_min, self.ang_vel_min], [self.lin_vel_max, self.ang_vel_max])
        ms_solver.set_state_bound([[0.0,0.0,0.0]]*(self.N+1), 
                                [[20.0,20.0,2*cs.pi]]*(self.N+1))
        ms_solver.build_problem()

        sol, solver_time, exit_status, solver_cost = ms_solver.solve()
        
        u_out_nest = ms_solver.get_opt_controls(sol)
        
        # Get the nested list down to a single list representing the input vector
        u_out = [u_out_nest[j][i] for i in range(len(u_out_nest[0])) for j in range(len(u_out_nest))]
        
        return u_out, solver_cost, exit_status, solver_time




