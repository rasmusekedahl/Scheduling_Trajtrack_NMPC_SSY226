import sys
import numpy as np
import matplotlib.pyplot as plt # type: ignore

import casadi as ca # type: ignore

from direct_multiple_shooting import MultipleShootingSolver

from matplotlib.axes import Axes # type: ignore
from typing import Union, Callable, cast

'''
Longitudinal vehicle control
'''

### Parameters
m = 44000.0     # mass [kg]
g = 9.81        # gravity [m/s^2]
s_init = 0.0    # initial distance [km]
s_goal = 5.0    # goal distance [km]
t_init = 0.0    # initial time [h]
t_goal = 0.1    # goal time [h]
v_min = 30.0    # minimum speed [km/h]
v_max = 90.0    # maximum speed [km/h]
a_min = -2.0    # minimum actuator acceleration [m/s^2]
a_max = 2.0     # maximum actuator acceleration [m/s^2]
v_init = 50.0   # initial speed [km/h]
v_goal = 50.0   # goal speed [km/h]

c_aero = 6.0    # aerodynamic drag coefficient [kg/m]
c_roll = 0.005  # rolling resistance coefficient [-]

ita_e = 0.85     # efficiency of the motor [-]

N = 100          # number of samples
ts = t_goal/N    # time step [h]
x0 = [s_init, v_init] # initial state

### Environment
def slope(s: Union[float, ca.SX]):
    """Hill slope

    Arguments:
        s: distance [km]

    Returns:
        slope: slope [rad]
    """
    return -0.063 * ca.sin(2*ca.pi/5*s)

def power_unidirection_pos(x, u_pos):
    """
    x is [s,v] and u is a_a
    """
    power = 1/3.6e6/ita_e*m*x[1]*u_pos
    return power

def power_unidirection_neg(x, u_neg):
    power = 1/3.6e6*ita_e*m*x[1]*u_neg 
    return power

def v_dot_func(s: Union[float, ca.SX], v: Union[float, ca.SX], a_a: Union[float, ca.SX]):
    return 3.6*3600*(a_a - c_aero*v**2/(2*3.6**2*m) - g*ca.sin(slope(s)) - c_roll*g*ca.cos(slope(s)))

def return_continuous_function(ns: int, nu: int) -> ca.Function:
    """
    xc: x1 - distance [km], x2 - speed [km/h]
    uc: uc_neg & uc_pos, actuator acceleration [m/s^2]
    """
    xc = ca.SX.sym('xc', ns)
    uc = ca.SX.sym('uc', nu)
    v_dot = v_dot_func(xc[0], xc[1], uc[0]+uc[1])
    xc_dot = ca.vertcat(xc[1], 
                        v_dot)
    Jc_obj = (power_unidirection_neg(xc, uc[0])
              + power_unidirection_pos(xc, uc[1])
              + 0.3 * (v_dot/3.6/3600)**2)
    fc = ca.Function('fc', [xc, uc], [xc_dot, Jc_obj])
    print("J",Jc_obj)
    return fc

ns = 2
nu = 2 # u_neg and u_pos
fc = return_continuous_function(ns, nu)

ms_solver = MultipleShootingSolver(ns, nu, ts, N)
ms_solver.set_initial_state(x0)
ms_solver.set_motion_model(fc, c2d=True, sub_sampling=3)
ms_solver.set_control_bound([a_min, 0], [0, a_max])
ms_solver.set_state_bound([[s_init, v_init]]+[[-ca.inf, v_min]]*(N-1)+[[s_goal, v_goal]], 
                          [[s_init, v_init]]+[[ ca.inf, v_max]]*(N-1)+[[s_goal, v_goal]])
ms_solver.build_problem()
sol = ms_solver.solve()

u_opt_neg = ms_solver.get_opt_controls(sol)[0]
u_opt_pos = ms_solver.get_opt_controls(sol)[1]
u_opt = [x+y for (x,y) in zip(u_opt_pos, u_opt_neg)]
x_pred = ms_solver.get_pred_states(sol)
x1_opt = x_pred[0]
x2_opt = x_pred[1]

rms = np.sqrt(np.mean(np.square(np.diff(x2_opt)*1000/3600**2/ts)))
print(f'RMS of acceleration: {rms} m/s^2')
peak = np.max(np.abs(np.diff(x2_opt)*1000/3600**2/ts))
print(f'Peak of acceleration: {peak} m/s^2')

power_opt_pos = [power_unidirection_pos([None, x_], u_) for (x_, u_) in zip(x2_opt, u_opt)]
power_opt_pos = [max(0,x) for x in power_opt_pos]
power_opt_neg = [power_unidirection_neg([None, x_], u_) for (x_, u_) in zip(x2_opt, u_opt)]
power_opt_neg = [min(0,x) for x in power_opt_neg]
power_opt = [x+y for (x,y) in zip(power_opt_pos, power_opt_neg)]
energy = sum(power_opt)*ts*1000 # kWh
print(f'Energy: {energy} kWh')

tgrid = [ts*k for k in range(N+1)]
_, axes = plt.subplots(4, 1, sharex=True)
axes = cast(list[Axes], axes)
ax1, ax2, ax3, ax4 = axes
ax1.plot(tgrid, x1_opt, '.')
ax1.plot([tgrid[0], tgrid[-1]], [s_init, s_init], 'r--') # reference line of s_init
ax1.plot([tgrid[0], tgrid[-1]], [s_goal, s_goal], 'r--') # reference line of s_goal
ax2.plot(tgrid, x2_opt, '.')
ax2.plot([tgrid[0], tgrid[-1]], [v_min, v_min], 'r--') # reference line of vmin
ax2.plot([tgrid[0], tgrid[-1]], [v_max, v_max], 'r--') # reference line of vmax

ax3.step(tgrid, ca.vertcat(ca.DM.nan(1), ca.DM(u_opt_pos)), 'g--')
ax3.step(tgrid, ca.vertcat(ca.DM.nan(1), ca.DM(u_opt_neg)), 'm--')
ax3.step(tgrid, ca.vertcat(ca.DM.nan(1), ca.DM(u_opt)), '.', label='actuator acceleration')
ax3.step(tgrid, ca.vertcat(ca.DM.nan(1), ca.DM(np.diff(x2_opt))), '.', label='vehicle acceleration')
ax3.plot([tgrid[0], tgrid[-1]], [a_min, a_min], 'r--') # reference line of aa_min
ax3.plot([tgrid[0], tgrid[-1]], [a_max, a_max], 'r--') # reference line of aa_max
ax4.plot(tgrid, ca.vertcat(ca.DM(power_opt), ca.DM.nan(1)), '.')
[ax.grid() for ax in axes]
ax1.set_ylabel('distance [km]')
ax2.set_ylabel('speed [km/h]')
ax3.set_ylabel('acceleration [m/s^2]')
ax4.set_ylabel('actuator power [MW]')
axes[-1].set_xlabel('time [h]')
ax3.legend()
plt.show()
