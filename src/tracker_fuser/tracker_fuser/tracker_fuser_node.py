import math, time
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class TrackerFuser(Node):
    def __init__(self):
        super().__init__('tracker_fuser')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.legs_sub = self.create_subscription(PoseArray, '/legs', self.legs_cb, qos)
        self.goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.declare_parameter('standoff', 1.2)
        self.declare_parameter('reissue_period', 1.5)
        self.declare_parameter('frame_id', 'base_link')
        self._last_goal_sent = 0.0
        self._last_target: Optional[PoseStamped] = None
        self.create_timer(0.2, self.maybe_send_goal)
    def legs_cb(self, msg: PoseArray):
        best = None; best_d = 1e9
        for p in msg.poses:
            x, y = p.position.x, p.position.y
            if x < 0.0: continue
            d = (x*x + y*y) ** 0.5
            if d < best_d: best_d, best = d, (x, y)
        if best is None: return
        standoff = float(self.get_parameter('standoff').value)
        bx, by = best; theta = math.atan2(by, bx)
        tx = max(0.0, best_d - standoff) * math.cos(theta)
        ty = max(0.0, best_d - standoff) * math.sin(theta)
        frame_id = str(self.get_parameter('frame_id').value)
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = frame_id
        target.pose.position.x = tx; target.pose.position.y = ty
        target.pose.orientation.z = math.sin(theta/2.0)
        target.pose.orientation.w = math.cos(theta/2.0)
        self._last_target = target; self.target_pub.publish(target)
    def maybe_send_goal(self):
        if self._last_target is None: return
        now = time.time(); period = float(self.get_parameter('reissue_period').value)
        if now - self._last_goal_sent < period: return
        if not self.goal_client.server_is_ready(): return
        goal_msg = NavigateToPose.Goal(); goal_msg.pose = self._last_target
        _ = self.goal_client.send_goal_async(goal_msg); self._last_goal_sent = now
def main():
    rclpy.init(); node = TrackerFuser(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
