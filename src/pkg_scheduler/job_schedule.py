import pandas as pd

from typing import Tuple, List, Any

class JobScheduler:
    """Schedule the jobs for the robots."""
    def __init__(self, total_schedule: pd.DataFrame) -> None:
        """
        Arguments:
            total_schedule: The total schedule for all robots.
        Notes:
            A total schedule is a dataframe with the following columns:
            - `robot_id`: The id of the robot.
            - `node_id`: The path node id of the robot.
            - `ETA`: The estimated time of arrival at the node.

            Or:
            - `robot_id`: The id of the robot.
            - `start`: The start node id of the robot.
            - `end`: The end node id of the robot.
            - `EDT`: The estimated duration of travel.
        """
        self._total_schedule = total_schedule
        self._robot_ids = total_schedule['robot_id'].unique().tolist()
        self.robot_schedule_dict = {}

        for robot_id in self._robot_ids:
            robot_schedule:pd.DataFrame = self.total_schedule[self.total_schedule['robot_id'] == robot_id]
            robot_schedule = robot_schedule.reset_index(drop=True)
            self.robot_schedule_dict[robot_id] = robot_schedule

    @property
    def total_schedule(self) -> pd.DataFrame:
        return self._total_schedule
    
    @property
    def robot_ids(self) -> list:
        return self._robot_ids
        
    @classmethod
    def from_csv(cls, csv_path: str, csv_sep:str=','):
        """Load the total schedule from a csv file."""
        total_schedule = pd.read_csv(csv_path, sep=csv_sep, header=0)
        return cls(total_schedule)
    
    def get_robot_schedule(self, robot_id: int) -> Tuple[list, List[float], bool]:
        """Get the schedule of a robot.
        
        Returns:
            path_nodes: The path nodes of the robot.
            path_times: The path times of the robot.
            whole_path: Whether the path is complete.
        """
        schedule:pd.DataFrame = self.robot_schedule_dict[robot_id]
        if 'ETA' in schedule.columns:
            path_nodes = schedule['node_id'].tolist()
            path_times = schedule['ETA'].tolist()
            whole_path = True
        elif 'EDT' in schedule.columns:
            path_nodes = [schedule['start_node'].iloc[0], schedule['end_node'].iloc[0]]
            path_times = [0.0, schedule['EDT'].iloc[0]]
            whole_path = False
        else:
            raise ValueError("The schedule must include ETA or EDT.")
        return path_nodes, path_times, whole_path


class _SequentialJobSchedule:
    def __init__(self) -> None:
        raise NotImplementedError
    
    def future_work(self):
        from collections import deque
        schedule_buffer = 10
        self._past_schedules = deque(maxlen=schedule_buffer)
        self._next_schedules = deque(maxlen=schedule_buffer)

    def _add_past_schedule(self, schedule: pd.DataFrame):
        self._past_schedules.append(schedule)

    def _add_next_schedule(self, schedule: pd.DataFrame, urgent:bool=False):
        import warnings
        if urgent:
            if len(self._next_schedules) == self._next_schedules.maxlen:
                warning_msg = "The next schedule buffer is full. The furthest schedule will be discard."
                warnings.warn(f"\033[33m{warning_msg}\033[0m")
            self._next_schedules.appendleft(schedule)
        if len(self._next_schedules) == self._next_schedules.maxlen:
            warning_msg = "The next schedule buffer is full. No more schedule can be added."
            warnings.warn(f"\033[33m{warning_msg}\033[0m")
        else:
            self._next_schedules.append(schedule)

    def load_schedule(self, new_schedule:pd.DataFrame=None, urgent:bool=False):
        import warnings
        if self.robot_id not in new_schedule['robot_id'].unique():
            warning_msg = f"The robot {self.robot_id} is not in the schedule."
            warnings.warn(f"\033[33m{warning_msg}\033[0m")
            return False
        
        if self._schedule is not None:
            self._add_next_schedule(new_schedule, urgent=urgent)
        else:   
            self._schedule = new_schedule[new_schedule['robot_id'] == self.robot_id]
            self._read_schedule(self._schedule)

        return True
    
    def remove_current_schedule(self):
        """Remove the current schedule and load the next schedule."""
        self._add_past_schedule(self._schedule)
        if self._next_schedules:
            self._schedule = self._next_schedules.popleft()
            self._read_schedule(self._schedule)
        else:
            self._schedule = None

    def remove_queued_schedule(self, index: int):
        import warnings
        if index >= len(self._next_schedules):
            warning_msg = f"The index {index} is out of range."
            warnings.warn(f"\033[33m{warning_msg}\033[0m")
            return False
        else:
            del self._next_schedules[index]
            return True


if __name__ == '__main__':
    csv_path = 'data/test_data/schedule.csv'
    job_scheduler = JobScheduler.from_csv(csv_path)
    robot_schedule = job_scheduler.get_robot_schedule("A_0")
    print(*robot_schedule, sep='\n')


