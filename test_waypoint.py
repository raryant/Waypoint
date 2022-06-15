import unittest
import waypoint
import rclpy
from geometry_msgs.msg import Pose as ROSPose
from nav2_msgs.action import NavigateToPose

class TestWaypoint(unittest.TestCase):
    def test_vector(self):
        self.assertEqual(waypoint.Vector3().json(), '{"x": 0.0, "y": 0.0, "z": 0.0}', 'Vector Invalid!')

    def test_quaternion(self):
        self.assertEqual(waypoint.Quaternion().json(), '{"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}', 'Quaternion Invalid!')

    def test_pose(self):
        self.assertEqual(waypoint.Pose().json(), '{"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}}', 'Pose Invalid!')

    def test_move(self):
        self.assertEqual(waypoint.MoveAction(point=waypoint.Pose()).json(), '{"description": "No Description", "frame": "map", "point": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}, "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0}}}', 'MoveAction model Invalid!')

    def test_poseConverter(self):
        self.assertIsInstance(waypoint.Pose2ROSPose(waypoint.Pose()), ROSPose, 'Pose Converter not working!')

    def test_loadPoint(self):
        self.assertEqual(len(waypoint.LoadTargetPoints('')), 0, 'Something wrong with file loader..')

    def test_createGoal(self):
        rclpy.init()
        node = rclpy.create_node('test_node')
        navGoal = waypoint.createGoal(node, waypoint.Pose())
        rclpy.shutdown()
        self.assertIsInstance(navGoal, NavigateToPose.Goal, 'Createing goal message type invalid')
        self.assertEqual(navGoal.pose.header.frame_id, 'map', 'default frame is not map!')

if __name__ == "__main__":
    unittest.main()