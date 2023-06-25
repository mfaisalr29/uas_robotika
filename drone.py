import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import sim
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
from moveit_msgs.msg import MotionPlanRequest, RobotTrajectory
from mavros_msgs.msg import PositionTarget

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.subscriber_ = self.create_subscription(String, 'my_topic', self.callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.range_publisher_ = self.create_publisher(Range, 'distance', 10)
        self.move_group_goal_ = MoveGroupGoal()
        self.motion_plan_request_ = MotionPlanRequest()
        self.position_target_ = PositionTarget()

    def callback(self, msg):
        print('Received:', msg.data)

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.5
        self.publisher_.publish(twist)

    def publish_range(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1
        msg.min_range = 0.0
        msg.max_range = 10.0
        msg.range = 5.0
        self.range_publisher_.publish(msg)

    def send_move_group_goal(self):
        self.move_group_client_.send_goal(self.move_group_goal_)
        self.move_group_client_.wait_for_result()

    def send_motion_plan_request(self):
        motion_plan_response = self.move_group_.plan(self.motion_plan_request_)
        robot_trajectory = RobotTrajectory()
        self.robot_trajectory_publisher_.publish(robot_trajectory)

    def send_position_target(self):
        self.position_target_publisher_.publish(self.position_target_)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    node.create_subscription(String, 'my_topic', node.callback, 10)
    node.create_timer(1.0, node.publish_twist)
    node.create_timer(1.0, node.publish_range)
    node.create_timer(1.0, node.send_move_group_goal)
    node.create_timer(1.0, node.send_motion_plan_request)
    node.create_timer(1.0, node.send_position_target)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
