import unittest
### Import the module to be tested
from pkg_motion_plan.local_traj_plan import LocalTrajPlanner
from pkg_motion_plan.global_path_plan import GlobalPathPlanner
from interfaces.interface_motion_plan import MotionPlanInterface
### Import dependencies of the module to be tested or test functions
import numpy as np
import matplotlib.pyplot as plt
### Import other modules
from basic_map.graph import NetGraph
import tests.load_helpers as helpers


class TestPlanner(unittest.TestCase):
    def test_local_planner(self): # Test LocalTrajPlanner
        # Arrange
        input_data = {"sampling_time": 0.1,
                      "horizon": 10,
                      "max_speed": 1,
                      "switch_node_distance": 0.5,}
        input_data_1 = {"path_coords": [(0,0), (1,1), (1,2)],
                        "path_times": [0,1,2],
                        "nomial_speed": 1,
                        "method": 'time'}
        
        # Act
        planner = LocalTrajPlanner(**input_data)
        planner.load_path(**input_data_1)
        
        # Assert
        expected_output = None

        # Visualization check
        num_steps = 25
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        for kt in range(num_steps):
            ax.cla()
            ax.plot(np.array(input_data_1["path_coords"])[:,0], np.array(input_data_1["path_coords"])[:,1], 's', markersize=15)
            ax.plot(np.array(planner._base_traj)[:,0], np.array(planner._base_traj)[:,1], 'o', markerfacecolor='none')
            ax.plot(planner.current_node[0], planner.current_node[1], 'x', markersize=15)

            ref_states, ref_speed, done = planner.get_local_ref(kt*0.1)
            ax.plot(ref_states[:,0], ref_states[:,1], '.')
            ax.set_title(f'{kt}/{num_steps-1}, speed={ref_speed}, done={done}')
            plt.pause(0.1)
            # while not plt.waitforbuttonpress(0.1):
            #     pass
        plt.show()

    def test_global_planner(self): # Test GlobalPathPlanner
        # Arrange
        test_graph_path = helpers.return_graph_path("test_data")
        test_graph = NetGraph.from_json(test_graph_path)
        input_data = {"path_nodes": ["d_1", "node_0"],
                      "path_times": [0, 3],
                      "whole_path": True}

        # Act
        planner = GlobalPathPlanner(robot_id=0)
        planner.load_graph(test_graph)
        planner.set_schedule(**input_data)
        path_coords, path_times = planner.get_schedule(time_offset=0.0)

        # Assert
        self.assertIsInstance(path_coords, list)
        self.assertIsInstance(path_coords[0], (tuple, list))
        self.assertIsInstance(path_times, list)
        self.assertEqual(len(path_coords), len(path_times))

    def test_interface(self): # Test MotionPlanInterface
        # Arrange
        test_map_path = helpers.return_map_path("test_data")
        test_graph_path = helpers.return_graph_path("test_data")
        config_mpc = helpers.load_mpc_config(helpers.return_cfg_path("mpc_fast.yaml"))
        config_robot = helpers.load_robot_spec(helpers.return_cfg_path("robot_spec.yaml"))
        input_data = {"robot_id": 0, 
                      "config_mpc": config_mpc, 
                      "config_robot": config_robot, 
                      "switch_node_distance": 1.0}
        
        input_data_1 = {"path_nodes": ["d_1", "node_0", "node_6"],
                        "path_times": [0, 3, 15],
                        "whole_path": True}

        # Act
        interface = MotionPlanInterface(**input_data)
        interface.load_map_from_json(test_map_path, inflation_margin=0.5)
        interface.load_graph_from_json(test_graph_path)
        interface.set_schedule(**input_data_1)
        interface.update_schedule(0.0, sampling_method='linear')
        ref_states, ref_speed = interface.get_local_ref(4.0)

        # Assert
        self.assertIsInstance(ref_states, np.ndarray)
        self.assertIsInstance(ref_speed, float)

        # Visualization check
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        interface.plot_map(ax, inflated=True)
        interface.plot_graph(ax)
        interface.plot_schedule(ax)
        ax.plot(interface.current_node_coord[0], interface.current_node_coord[1], 's', markersize=15)
        plt.show()

if __name__ == '__main__':
    unittest.main()