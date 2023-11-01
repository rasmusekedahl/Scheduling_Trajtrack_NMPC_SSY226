import unittest
### Import the module to be tested
from basic_map.map_geometric import GeometricMap
from basic_map.map_occupancy import OccupancyMap
from basic_map.graph import NetGraph
### Import dependencies of the module to be tested or test functions
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes
### Import other modules
import tests.load_helpers as helpers


class TestMapAndGraph(unittest.TestCase):
    def test_geometric_map(self): # Test GeometricMap
        # Arrange
        obstacle_list = [{"id_": 1, "vertices": [(1,1), (2,1), (2,2), (1,2)]}, 
                         {"id_": 2, "vertices": [(3,3), (4,3), (4,4), (3,4)]}]
        input_data = {"boundary_coords": [(0,0), (10,0), (10,10), (0,10)],
                      "obstacle_info_list": obstacle_list}
        
        # Act
        geo_map = GeometricMap(**input_data)
        output = geo_map.get_occupancy_map(rescale=100)
        
        # Assert
        expected_output = None
        self.assertIsInstance(output, np.ndarray)

        # Visualization check
        _, axes = plt.subplots(1,2)
        axes: list[Axes]
        axes[0].set_title('Geometric Map')
        geo_map.plot(axes[0])
        axes[0].set_aspect('equal')
        axes[1].set_title('Occupancy Map (1000x1000)') # 100*10
        axes[1].imshow(output)
        plt.show()

    def test_graph(self): # Test NetGraph
        # Arrange
        input_data = {"node_dict": {'a': (0,0), 'b': (0,1), 'c': (1,1), 1: (1,0), 2: (2,0)},
                      "edge_list": [('a',1), ('a','b'), ('b','c'), (1,2)]
                      }
        
        # Act
        G = NetGraph(**input_data)
        
        # Assert
        expected_output = None
        self.assertIsInstance(G, NetGraph)

        # Visualization check
        fig, ax = plt.subplots()
        G.plot_graph(ax)
        ax.set_title('Graph')
        plt.show()


if __name__ == '__main__':
    unittest.main()