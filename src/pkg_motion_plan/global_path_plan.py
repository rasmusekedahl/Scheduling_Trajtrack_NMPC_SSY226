import networkx as nx

from .path_plan_graph import dijkstra

from typing import Tuple, Any, List, Union


class GlobalPathPlanner:
    """Plan the global paths according to the detailed or rough schedule for a specific robot.
    The detailed schedule includes full path for the robot, while the rough schedule only includes the start and end nodes.

    Attributes:
        robot_id: The ID of the robot [Any].
        path_nodes: The path nodes (IDs) of the robot [List[Any]].
        path_times: The path times of the robot [List[float]].
        whole_path: Whether the path is complete [bool].
        _G: The graph of the environment [nx.Graph].
    """
    def __init__(self, robot_id: Any) -> None:
        self._robot_id = robot_id

        self._path_nodes = None
        self._path_times = None
        self._whole_path = False

        self._G = None

    @property
    def robot_id(self) -> Any:
        return self._robot_id
    
    @property
    def whole_path(self) -> bool:
        return self._whole_path

    def load_graph(self, G: nx.Graph):
        self._G = G

    def set_schedule(self, path_nodes: list, path_times: Union[List[float], None], whole_path: bool):
        """If path_times is None, the path has no temporal requirement."""
        if path_times is None:
            path_times = [None for _ in range(len(path_nodes))]
        self._path_nodes = path_nodes
        self._path_times = path_times
        self._whole_path = whole_path
        
    def get_schedule(self, time_offset:float=0.0, position_key="position") -> Tuple[List[tuple], List[Union[float, None]]]:
        """
        Arguments:
            time_offset: The delayed time offset of the schedule.
            
        Returns:
            path_coords (list): list of coordinates of the path nodes
            path_times (list): list of absolute time stamps
        """
        if self._G is None:
            raise ValueError("The graph is not loaded.")
        if self._path_nodes is None:
            raise ValueError("The schedule is not set.")
        
        if self.whole_path:
            path_nodes = self._path_nodes
            path_times = self._path_times
        else:
            source = self._path_nodes[0]
            target = self._path_nodes[1]
            edt = self._path_times[1]
            path_nodes, section_length_list = self.get_shortest_path(self._G, source, target)
            path_times = [x/sum(section_length_list)*edt for x in section_length_list]
        if None not in path_times:
            path_times = [time_offset + x for x in path_times]
        path_coords = [self._G.nodes[node_id][position_key] for node_id in path_nodes]
        return path_coords, path_times
    
    @staticmethod
    def get_shortest_path(graph: nx.Graph, source: Any, target: Any, algorithm:str='dijkstra'):
        """
        Arguments:
            source: The source node ID.
            target: The target node ID.
            algorithm: The algorithm used to find the shortest path. Currently only "dijkstra".
        Returns:
            shortest_path: The shortest path from source to target.
            section_lengths: The lengths of all sections in the shortest path.
        Notes:
            The weight key should be set to "weight" in the graph.
        """
        if algorithm == 'dijkstra':
            planner = dijkstra.DijkstraPathPlanner(graph)
            _, paths = planner.k_shortest_paths(source, target, k=1, get_coords=False)
            shortest_path = paths[0]
        else:
            raise NotImplementedError(f"Algorithm {algorithm} is not implemented.")
        if len(shortest_path) > 2:
            section_lengths = [graph.edges[shortest_path[i], shortest_path[i+1]]['weight'] for i in range(len(shortest_path)-1)]
        return shortest_path, section_lengths
    

