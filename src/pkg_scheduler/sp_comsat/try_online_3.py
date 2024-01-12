from OnlineScheduler import Scheduler, SchedulerConfig
import json
import time

class Path:
    # NOTE: json_path is obtained from scheduler_result.json -> "robots" -> [robot_item] -> "tasks" -> [task_item] -> "path" field
    def __init__(self, json_path:list) -> None:
        self.node_list = list()         # List of str
        self.reach_time_list = list()   # List of int
        self.setPath(json_path)
        return
    
    # NOTE: json_path is obtained from scheduler_result.json -> "robots" -> [robot_item] -> "tasks" -> [task_item] -> "path" field
    def setPath(self, json_path:list) -> None:
        for node in json_path:
            self.node_list.append(node["name"])         # NOTE: "name" field must already exist the mentioned json
            self.reach_time_list.append(node["time"])   # NOTE: "time" field must already exist the mentioned json
        return

# TODO: SCH_REQ: The format of the scheduler_result.json file must be change a bit so that we assign robot tasks (and not the combined path with includes multiple task)
# NOTE: To understand the above comment better please refer demo_scheduler_result.json file and compare it with scheduler_result.json
class Task:
    def __init__(self, id:str, status:str, path:Path) -> None:
        self.id = id
        self.status = status                # NOTE: ["pending", "ongoing", "complete", ""] either of these str values is valid
        self.path = path                    # NOTE: This needs to be set once the schedule is generated
        pass

    def updateStatus(self, sys_time) -> None:
        if not len(self.path.reach_time_list):
            self.status = ""
        elif sys_time >= self.path.reach_time_list[-1]:
            self.status = "complete"
        elif sys_time <= self.path.reach_time_list[0]:
            self.status = "pending"
        else:
            self.status = "ongoing"        
        return
        
    # NOTE: json_path is obtained from scheduler_result.json -> "robots" -> [robot_item] -> "tasks" -> [task_item] -> "path" field
    def updateState(self, sys_time) -> None:
        self.updateStatus(sys_time)
        # TODO: DISCUSS: Anything pending in update task state??
        return

class Robot:
    # NOTE: json_task_list is obtained from scheduler_result.json -> "robots" -> [robot_item] -> "tasks"
    def __init__(self, id:str, sys_time, json_task_list:list=list(), is_busy:bool=False) -> None:
        self.id = id
        self.task_list = list()         # NOTE: This will be a list of "Task" objects
        self.is_busy = is_busy
        self._current_task = 0
        self.next_location = ""
        self._prev_task_reach_time = 0
        self.setTaskList(json_task_list)
        self.updateCurrentTask(sys_time)
        self.updateLocation(sys_time)
        return
    
    # NOTE: json_task_list is obtained from scheduler_result.json -> "robots" -> [robot_item] -> "tasks"
    def setTaskList(self, json_task_list:list=list()) -> None:
        if len(json_task_list) == 0:
            return
        
        task_ids = [task.id for task in self.task_list]
        for json_task in json_task_list:
            if json_task['name'] not in task_ids:       # only add new tasks to task list
                self.task_list.append(Task(json_task["name"], "pending", Path(json_task["path"])))
        return
    
    def getCurrentTask(self) -> Task:
        return self.task_list[self._current_task]
    
    def updateIsBusy(self, sys_time) -> None:
        curr_task = self.getCurrentTask()

        # The robot is busy if the sys_time > previous task reach time (current task start time) AND sys_time < current task reach time 
        self.is_busy =  (sys_time >= self._prev_task_reach_time) and (sys_time <= curr_task.path.reach_time_list[-1])
        return
        
    def updateCurrentTask(self, sys_time) -> None:
        curr_task = self.getCurrentTask()
        
        # update the state of the current task
        curr_task.updateState(sys_time)

        # check if its time to change the task and are there more task in task_list?
        if sys_time >= curr_task.path.reach_time_list[-1] and self._current_task < len(self.task_list):
            self._prev_task_reach_time = curr_task.path.reach_time_list[-1]
            self._current_task = self._current_task + 1     # go to the next task in the task_list
        return
    
    def updateLocation(self, sys_time) -> None:
        curr_task = self.getCurrentTask()
        for index, reach_time in enumerate(curr_task.path.reach_time_list):
            if sys_time <= reach_time:
                self.next_location = curr_task.path.node_list[index]
                break
        return
        
    # NOTE: json_task_list is obtained from scheduler_result.json -> "robots" -> [robot_item] -> "tasks"
    def updateState(self, sys_time, json_task_list:list=list()) -> None:
        self.updateCurrentTask(sys_time)
        self.updateLocation(sys_time)
        self.updateIsBusy(sys_time)
        self.setTaskList(json_task_list)
        return

class RobotManager:
    def __init__(self) -> None:
        self.robot_list = list()
        pass
    
    def addRobot(self, robot:Robot):
        self.robot_list.append(robot)
        pass
    
    def getRobotWithId(self, id:str):
        for robot in self.robot_list:
            if robot.id == id:
                return robot
        return None
    
    def getRobots(self):
        return self.robot_list

class Coordinator:
    def __init__(self, sec_per_tick:int, problem_filename:str, output_dir:str='./output') -> None:
        self.ticks = 0                                      # stores the number of ticks from the time Coordinator started to run
        self.sec_per_tick = sec_per_tick                    # The period / timestep at which the coordinator runs
        self.problem = dict()                               # content of the problem file
        self.problem_filename = problem_filename
        self.rm = RobotManager()
        self.scheduler = Scheduler(SchedulerConfig(dir=output_dir))
        self.scheduler.set_problem_filename(self.problem_filename)
        self.pseudo_new_job_list = dict()
        
        # load the initial problem file (instance maker generated file)
        with open(f"{self.scheduler.config.output_folder}/{self.problem_filename}", 'r') as file:
            self.problem = json.load(file)
        
        # TODO: FUTURE: May be delete this step later once we have a system to load tasks dynamically at run time.
        # copy entire "jobs" field to pseudo_new_job_list and delete all the entries in the actual file
        self.pseudo_new_job_list = self.problem["jobs"]
        self.problem["jobs"] = dict()
        # self._write_json(self.problem, self.problem_filename)
        return
    
    def _write_json(self, json_data, file_name):
        # Open the file and write the JSON data
        with open(file_name, 'w') as file:
            file.write(json.dumps(json_data, indent=4))

    # TODO: FUTURE: Later this function implementation will have significant changes when we can load tasks dynamically at run time.
    # This function adds new tasks to the problem
    def _addNewTasks(self):
        for _ in range(2):      # choice of 2 tasks is at random. no specific reason
            if len(self.pseudo_new_job_list):
                job_name, job = self.pseudo_new_job_list.popitem()                      # get new job
                self.problem["jobs"][job_name] = job                                # add new job to problem
        return
    
    def getSystemTime(self):
        return (self.ticks * self.sec_per_tick)

    # This function runs periodically on a seperate thread after every 'sec_per_tick'
    def _run(self):
        sys_time = self.getSystemTime()         # time in seconds

        # generate a new schedule based on new scheduling problem - here we decide which idle robot gets which pending task
        # TODO: DISCUSS: for now the scheduler runs 5 time slower that coordinator. take this as user input??
        is_new_schedule = False
        if self.ticks % 5 == 0:
            # add new tasks to the problem
            self._addNewTasks()
            self.scheduler.schedule(self.problem)       # the scheduler now accepts dict() as direct input
            is_new_schedule = True

        # for each robot in scheduler_result:
        for json_robot in self.scheduler.result["robots"]:
            robot_name = json_robot["name"]
            json_tasks = list()
            if is_new_schedule:
                json_tasks = json_robot["tasks"]
            
            robot = self.rm.getRobotWithId(robot_name)
            if robot != None:
                # update the robot state
                robot.updateState(sys_time, json_tasks)
            else:
                # new robot detected. create a robot object and add to robot manager.
                robot = Robot(id=robot_name, sys_time=sys_time, json_task_list=json_robot["tasks"])
                self.rm.addRobot(robot)
            
            # update "start_list" -> [robot_id] (contains robot current/upcoming location)
            if len(robot.next_location):
                self.problem["test_data"]["start_list"][robot_name] = robot.next_location

            # update "ATRs" -> [robot_id] -> "isBusy"
            self.problem["ATRs"][robot_name]["isBusy"] = robot.is_busy
            
            # update "jobs" -> [task_id] -> "status"
            for task in robot.task_list:
                if task.id.startswith('task_'):
                    self.problem["jobs"][task.id]["status"] = task.status
        
        # TODO: DISCUSS: only update the problem json ?? we don't need to update the scheduler_result.json file with status since input of schedule generation has nothing to do with it.
        # update the problem and export the updated problem as a json
        # self.updateProblem()
        # self._write_json(self.problem, self.problem_filename)

        self.ticks = self.ticks + 1        # update the system tick
        return
        
    def run(self):
        self.ticks = 0     # reset the ticks before every run.
        while True:
            self._run()
            time.sleep(self.sec_per_tick)
    
def main():
    coor = Coordinator(sec_per_tick=1, problem_filename="MM_8_2_6_1.0_7000_7.json")
    coor.run()

if __name__ == "__main__":
    main()

