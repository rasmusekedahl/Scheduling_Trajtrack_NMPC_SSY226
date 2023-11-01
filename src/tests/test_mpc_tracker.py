import unittest
### Import the module to be tested
from pkg_mpc_tracker.trajectory_tracker import TrajectoryTracker
from interfaces.interface_mpc_tracker import MpcInterface
### Import dependencies of the module to be tested or test functions
import numpy as np
import matplotlib.pyplot as plt
### Import other modules
import math
import tests.load_helpers as helpers


class TestPlanner(unittest.TestCase):
    def test_local_planner(self): # Test LocalTrajPlanner
        # Arrange
        config_mpc = helpers.load_mpc_config("mpc_fast.yaml")
        config_robot = helpers.load_robot_spec("robot_spec.yaml")
        
        static_obstacles = [[(4, 4), (4, 6), (6, 6), (6, 4)],
                            [(5, 5), (5, 7), (7, 7), (7, 5)]]
        boundary_coords = [(-1, -1), (-1, 11), (11, 11), (11, -1)]
        start_state_1 = np.array([0, 0, math.pi/4])
        goal_state_1 = np.array([10, 10, 0])
        start_state_2 = np.array([10, 10, 5*math.pi/4])
        goal_state_2 = np.array([0, 0, 0])
        
        other_robot_states = [0] * config_mpc.ns * (config_mpc.N_hor+1) * config_mpc.Nother

        # Act & Visualize
        tracker_1 = MpcInterface(config_mpc, config_robot, verbose=False)
        tracker_1.load_init_states(start_state_1, goal_state_1)
        tracker_1.load_static_obstacles(static_obstacles)
        ref_states_1 = np.tile(goal_state_1, (config_mpc.N_hor, 1))
        tracker_1.set_ref_states(ref_states_1, ref_speed=0.5)

        tracker_2 = MpcInterface(config_mpc, config_robot, verbose=False)
        tracker_2.load_init_states(start_state_2, goal_state_2)
        tracker_2.load_static_obstacles(static_obstacles)
        ref_states_2 = np.tile(goal_state_2, (config_mpc.N_hor, 1))
        tracker_2.set_ref_states(ref_states_2, ref_speed=0.5)

        fig, ax = plt.subplots()
        cnt = 0
        ax.set_aspect('equal')
        while not tracker_1.check_termination_condition() and not tracker_2.check_termination_condition():
            cnt += 1
            robot_1_states = list(np.tile(tracker_2.state, (config_mpc.N_hor+1, 1)).reshape(-1))
            robot_2_states = list(np.tile(tracker_1.state, (config_mpc.N_hor+1, 1)).reshape(-1))
            tracker_1.run_step(other_robot_states=other_robot_states)
            tracker_2.run_step(other_robot_states=other_robot_states)
            ax.cla()
            ax.set_title(f'{cnt}')
            ax.plot([-1, 11], [-1, 11], 'k--')
            ax.plot(np.array(boundary_coords+[boundary_coords[0]])[:,0], np.array(boundary_coords+[boundary_coords[0]])[:,1], 'k--')
            for obs in static_obstacles:
                ax.plot(np.array(obs+[obs[0]])[:,0], np.array(obs+[obs[0]])[:,1], 'k-')
            ax.plot(tracker_1.state[0], tracker_1.state[1], 'bo', markersize=15)
            ax.plot(ref_states_1[:,0], ref_states_1[:,1], 'b--')
            ax.plot(tracker_2.state[0], tracker_2.state[1], 'ro', markersize=15)
            ax.plot(ref_states_2[:,0], ref_states_2[:,1], 'r--')
            plt.pause(0.1)
            # while not plt.waitforbuttonpress(0.1):
            #     pass
        plt.show()
        
        # Assert
        expected_output = None

        # Visualization check
        # num_steps = 25
        # fig, ax = plt.subplots()
        # ax.set_aspect('equal')
        # for kt in range(num_steps):
        #     ax.cla()
        #     ax.plot(np.array(input_data_1["path_coords"])[:,0], np.array(input_data_1["path_coords"])[:,1], 's', markersize=15)
        #     ax.plot(np.array(planner._base_traj)[:,0], np.array(planner._base_traj)[:,1], 'o', markerfacecolor='none')
        #     ax.plot(planner.current_node[0], planner.current_node[1], 'x', markersize=15)

        #     ref_states, ref_speed, done = planner.get_local_ref(kt*0.1)
        #     ax.plot(ref_states[:,0], ref_states[:,1], '.')
        #     ax.set_title(f'{kt}/{num_steps-1}, speed={ref_speed}, done={done}')
        #     plt.pause(0.1)
        #     # while not plt.waitforbuttonpress(0.1):
        #     #     pass
        # plt.show()


if __name__ == '__main__':
    unittest.main()