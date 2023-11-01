import os
import pathlib
from configs import MpcConfiguration, CircularRobotSpecification

ROOT_DIR = pathlib.Path(__file__).resolve().parents[2]

### Configs
def return_cfg_path(fname: str) -> str:
    cfg_path = os.path.join(ROOT_DIR, "config", fname)
    return cfg_path

def load_mpc_config(fname: str) -> MpcConfiguration:
    """Load the MPC configuration."""
    return MpcConfiguration.from_yaml(return_cfg_path(fname))

def load_robot_spec(fname: str) -> CircularRobotSpecification:
    """Load the robot specification."""
    return CircularRobotSpecification.from_yaml(return_cfg_path(fname))

### Data
def return_graph_path(folder_name: str) -> str:
    data_path = os.path.join(ROOT_DIR, "data", folder_name, "graph.json")
    return data_path

def return_map_path(folder_name: str) -> str:
    data_path = os.path.join(ROOT_DIR, "data", folder_name, "map.json")
    return data_path

def return_robot_start_path(folder_name: str) -> str:
    data_path = os.path.join(ROOT_DIR, "data", folder_name, "robot_start.json")
    return data_path

def return_schedule_path(folder_name: str) -> str:
    data_path = os.path.join(ROOT_DIR, "data", folder_name, "schedule.csv")
    return data_path


