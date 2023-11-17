import sys
import numpy as np
import matplotlib.pyplot as plt # type: ignore

import casadi as ca # type: ignore

from direct_multiple_shooting import MultipleShootingSolver
import mpc_cost as cost



# Define control parameters 
ns = 3      #Noumber of states
nu = 2      #Noumber of inputs
N = 40      #Control horizon
ts = 0.2    #Sample time
u_min = 1
u_max = -1

### State bounds of the robot
lin_vel_min = -0.5  # Vehicle contraint on the minimal velocity possible
lin_vel_max = 1.5   # Vehicle contraint on the maximal velocity possible
ang_vel_min = -0.5  # Vehicle contraint on the maximal angular velocity
ang_vel_max = 0.5   # Vehicle contraint on the maximal angular velocity

# Initial states
x0 = [10, 0, ca.pi] 

#Define a static reference
ref = ca.SX.sym('ref', 3, 1)

# Create an expression using the symbolic matrix
new_matrix = ca.vertcat(ref[0], ref[1],ref[2] )

# Create a CasADi Function
fun = ca.Function('fun', [ref], [new_matrix])

# Define a specific 2x1 matrix as a reference
static_ref =fun([0.0, 2.0, ca.pi])


print("reference:", static_ref)



def unicycle_model(state: ca.SX, action: ca.SX, ts: float, rk4:bool=True) -> ca.SX:
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

def return_continuous_function(ns: int, nu: int) -> ca.Function:
    """
    xc: x1 - distance [km], x2 - speed [km/h]
    uc: uc_neg & uc_pos, actuator acceleration [m/s^2]
    """
    x = ca.SX.sym('x', ns)
    u = ca.SX.sym('u', nu)
    f = unicycle_model(x, u, ts)
    J_obj = cost.cost_refstate_deviation(x, static_ref)
    print(J_obj)
    fk = ca.Function('fk', [x, u], [f, J_obj[0]+J_obj[1]])
    return fk


state_traj_x = []
state_traj_y = []
for kt in range(10):

    # Define a symbolic continious function
    fc = return_continuous_function(ns, nu)

    # Discretize the continious function 
    ms_solver = MultipleShootingSolver(ns, nu, ts, N)
    ms_solver.set_initial_state(x0)
    ms_solver.set_motion_model(fc, c2d=False)

    # Define state and output bounds
    ms_solver.set_control_bound([lin_vel_min, ang_vel_min], [lin_vel_max, ang_vel_max])
    ms_solver.set_state_bound([[0.0,0.0,0.0]]*(N+1), 
                            [[10.0,10.0,2*ca.pi]]*(N+1))
    ms_solver.build_problem()
    sol = ms_solver.solve()

    u_out = ms_solver.get_opt_controls(sol)[0][:]
    print(u_out)
    x0 = unicycle_model(x0, u_out, ts)

    state_traj_x.append(ms_solver.get_pred_states(sol)[0])
    state_traj_y.append(ms_solver.get_pred_states(sol)[1])

    print(kt)


"""
x_pred = ms_solver.get_pred_states(sol)
x1_opt = x_pred[0]
x2_opt = x_pred[1]
x3_opt = x_pred[2]

u_out = ms_solver.get_opt_controls(sol)
u1 = u_out[0]
u2 = u_out[1]

print("x1",x1_opt)
print("u1", u1)
print("u2",u2)



"""
print('done')