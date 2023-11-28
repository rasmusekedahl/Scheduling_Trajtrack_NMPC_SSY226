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

        ### State bounds of the robot
        self.lin_vel_min = self.robot_spec.lin_vel_min  # Vehicle contraint on the minimal velocity possible
        self.lin_vel_max = self.robot_spec.ang_vel_max   # Vehicle contraint on the maximal velocity possible
        self.ang_vel_min = -self.robot_spec.ang_vel_max  # Vehicle contraint on the maximal angular velocity
        self.ang_vel_max = self.robot_spec.ang_vel_max   # Vehicle contraint on the maximal angular velocity

        # Initial states
        self.x0 = params[2:5]
        self.ref_state = params[5:8]
        
        #Define a static reference
        ref = cs.SX.sym('ref', 3, 1)

        # Create an expression using the symbolic matrix
        new_matrix = cs.vertcat(ref[0], ref[1],ref[2] )

        # Create a CasADi Function
        fun = cs.Function('fun', [ref], [new_matrix])

        # Define a specific 2x1 matrix as a reference
        self.static_ref =fun(self.ref_state)


        print("reference:", self.static_ref)


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
    

    def cost_calculation(self):


        u = cs.SX.sym('u', self.nu*self.N)  # 0. Inputs from 0 to N_hor-1

        u_m1 = cs.SX.sym('u_m1', self.nu)       # 1. Input at kt=-1
        s_0 = cs.SX.sym('s_0', self.ns)         # 2. State at kt=0
        s_N = cs.SX.sym('s_N', self.ns)         # 3. State of goal at kt=N_hor
        q = cs.SX.sym('q', self.config.nq)      # 4. Penalty parameters for objective terms
        r_s = cs.SX.sym('r_s', self.ns*self.N)  # 5. Reference states
        r_v = cs.SX.sym('r_v', self.N)          # 6. Reference speed
        c_0 = cs.SX.sym('c_0', self.ns*self.config.Nother)          # 7. States of other robots at kt=0
        c = cs.SX.sym('c', self.ns*self.N*self.config.Nother)   # 8. Predicted states of other robots
        o_s = cs.SX.sym('os', self.config.Nstcobs*self.config.nstcobs)                  # 9. Static obstacles
        o_d = cs.SX.sym('od', self.config.Ndynobs*self.config.ndynobs*(self.N+1))   # 10. Dynamic obstacles
        q_stc = cs.SX.sym('qstc', self.N)               # 11. Static obstacle weights
        q_dyn = cs.SX.sym('qdyn', self.N)               # 12. Dynamic obstacle weights

        z = cs.vertcat(u_m1, s_0, s_N, q, r_s, r_v, c_0, c, o_s, o_d, q_stc, q_dyn)
        
        (x, y, theta) = (s_0[0], s_0[1], s_0[2])
        (x_goal, y_goal, theta_goal) = (s_N[0], s_N[1], s_N[2])
        (v_init, w_init) = (u_m1[0], u_m1[1])
        (qpos, qvel, qtheta, rv, rw)                    = (q[0], q[1], q[2], q[3], q[4])
        (qN, qthetaN, qrpd, acc_penalty, w_acc_penalty) = (q[5], q[6], q[7], q[8], q[9])
        
        ref_states = cs.reshape(r_s, (self.ns, self.N)).T
        ref_states = cs.vertcat(ref_states, ref_states[[-1],:])[:, :2] #Remove theta


        cost = 0
        penalty_constraints = 0
        state = cs.vcat([x,y,theta])

        #for kt in range(0, self.N): # LOOP OVER PREDICTIVE HORIZON
        ### Run step with motion model
        u_t = u  # inputs at time t
        state = self.unicycle_model(state, u_t, self.ts) # Kinematic/dynamic model

        ### Reference deviation costs
        cost += mpc_cost.cost_refvalue_deviation(state, r_s, weight=qvel)        
        

        return cost

    """"
    def return_continuous_function(self) -> cs.Function:
        
        xc: x1 - distance [km], x2 - speed [km/h]
        uc: uc_neg & uc_pos, actuator acceleration [m/s^2]
        
        x = cs.SX.sym('x', self.ns)
        u = cs.SX.sym('u', self.nu)
        f = self.unicycle_model(x, u, self.ts)
        #J_obj = self.cost_calculation()#mpc_cost.cost_refstate_deviation(x, self.static_ref) 
        #J_obj = J_obj[0]+J_obj[1]+J_obj[2]
        #print("J_obj", J_obj)
         # Taking 2 first states
        #fk = cs.Function('fk', [x,u], [f])
        return f
    """

    def run(self):
        # Define a symbolic continious function
        #fk = self.return_continuous_function()

        # Discretize the continious function 
        ms_solver = MultipleShootingSolver(self.ns, self.nu, self.ts, self.N, self.config)
        ms_solver.set_initial_state(self.x0)
        ms_solver.set_motion_model(self.unicycle_model)

        # Define state and output bounds
        ms_solver.set_control_bound([self.lin_vel_min, self.ang_vel_min], [self.lin_vel_max, self.ang_vel_max])
        ms_solver.set_state_bound([[0.0,0.0,0.0]]*(self.N+1), 
                                [[20.0,20.0,2*cs.pi]]*(self.N+1))
        ms_solver.build_problem()
        sol, solver_time, exit_status, solver_cost = ms_solver.solve()

        u_out = [output[0] for output in ms_solver.get_opt_controls(sol)]
        print("U_out",u_out)
        
        return u_out, solver_cost, exit_status, solver_time




