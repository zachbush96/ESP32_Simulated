from src.robot_api.client import RealRobotAPI
from src.robot_api.interfaces import RobotAPI
from src.robot_api.sim_client import SimulatedRobotAPI


def get_robot_api(config: dict) -> RobotAPI:
    mode = config.get("settings", {}).get("mode", "simulation")
    if mode == "real":
        base_url = config.get("settings", {}).get("base_url", "http://192.168.4.1")
        return RealRobotAPI(base_url=base_url)
    return SimulatedRobotAPI()
