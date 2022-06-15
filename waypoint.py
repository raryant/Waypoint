from typing import List, Optional
import rclpy
import json
import os
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose as ROSPose
from action_msgs.msg import GoalStatus
from time import sleep
from rclpy.executors import MultiThreadedExecutor

class Vector3(BaseModel):
    x: Optional[float] = 0.0
    y: Optional[float] = 0.0
    z: Optional[float] = 0.0
    
class Quaternion(BaseModel):
    x: Optional[float] = 0.0
    y: Optional[float] = 0.0
    z: Optional[float] = 0.0
    w: Optional[float] = 0.0

class Pose(BaseModel):
    position: Optional[Vector3] = Vector3()
    orientation: Optional[Quaternion] = Quaternion()  

class MoveAction(BaseModel):
    description: Optional[str] = "No Description"
    frame: Optional[str] = 'map'
    point: Pose

def LoadTargetPoints(path: str)->List[MoveAction]:
    if not os.path.isfile(path): return []
    with open(path, 'r') as fileHandler:
        points = json.load(fileHandler)
        movePoints = [MoveAction.parse_obj(point) for point in points]
        return movePoints
def Pose2ROSPose(pose: Pose)-> ROSPose:
    ret = ROSPose()
    ret.position.x = pose.position.x
    ret.position.y = pose.position.y
    ret.position.z = pose.position.z
    ret.orientation.x = pose.orientation.x
    ret.orientation.y = pose.orientation.y
    ret.orientation.z = pose.orientation.z
    ret.orientation.w = pose.orientation.w
    return ret

def createGoal(node: Node, pose: Pose, frame: str = 'map')->NavigateToPose.Goal:
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id=frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose = Pose2ROSPose(pose)
    return goal

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = rclpy.create_node("WaypointNode")
    executor.add_node(node)
    actionHandler = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    while True:
        if actionHandler.wait_for_server(): break
        print("waiting for server...")
        rclpy.spin_once(node, timeout_sec=0.5)
    executor.spin_once(timeout_sec=1)
    movePoints = LoadTargetPoints('points.json')
    print(f"Total Points: {len(movePoints)}")
    for move in movePoints:
        print(f"Executing : {move.description}")
        future = actionHandler.send_goal_async(createGoal(node, move.point, move.frame))
        executor.spin_once_until_future_complete(future)
        retry = 0
        while True:
            executor.spin_once(timeout_sec=1)
            result = future.result()
            if result is None: continue
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                print(f"Success going to {move.description}")
                break
            if result.status == GoalStatus.STATUS_ABORTED or result.status == GoalStatus.STATUS_CANCELED:
                print(f"Failed going to {move.description}")
                if retry < 3:
                    retry += 1
                    print(f"Retrying {move.description}")
                    future = actionHandler.send_goal_async(createGoal(node, move.point, move.frame))
                    executor.spin_once_until_future_complete(future)
                    print(f"Send Retrying Request {move.description}")
                else:
                    print(f"Skip {move.description}")
                    break
    rclpy.shutdown()

if __name__ == "__main__":
    main()