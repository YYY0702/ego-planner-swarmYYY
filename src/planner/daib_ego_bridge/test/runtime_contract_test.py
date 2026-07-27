#!/usr/bin/env python3

import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt64


class RuntimeContractTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._goals = []
        self._generations = []
        self._goal_pub = rospy.Publisher(
            "/daib_explorer/goal", PoseStamped, queue_size=1, latch=True
        )
        self._ready_pub = rospy.Publisher(
            "/daib_explorer/ready", Bool, queue_size=1, latch=True
        )
        self._odom_pub = rospy.Publisher(
            "/daib_slam/odom", Odometry, queue_size=1
        )
        self._planner_goal_sub = rospy.Subscriber(
            "/daib_ego/goal", PoseStamped, self._goal_callback, queue_size=1
        )
        self._generation_sub = rospy.Subscriber(
            "/daib_ego/accepted_generation",
            UInt64,
            self._generation_callback,
            queue_size=1,
        )

    def _goal_callback(self, message):
        with self._lock:
            self._goals.append(message)

    def _generation_callback(self, message):
        with self._lock:
            self._generations.append(message.data)

    def _publish_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "camera_init"
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

    @staticmethod
    def _goal(generation, frame="camera_init"):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = frame
        goal.header.seq = generation
        goal.pose.position.x = 5.0
        goal.pose.position.y = 1.0
        goal.pose.position.z = 1.5
        goal.pose.orientation.w = 1.0
        return goal

    def test_ready_frame_and_generation_gate(self):
        rate = rospy.Rate(20)
        self._ready_pub.publish(Bool(data=False))
        deadline = rospy.Time.now() + rospy.Duration(1.0)
        while rospy.Time.now() < deadline:
            self._publish_odom()
            self._goal_pub.publish(self._goal(7))
            rate.sleep()
        with self._lock:
            self.assertEqual(len(self._goals), 0)

        self._ready_pub.publish(Bool(data=True))
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while rospy.Time.now() < deadline:
            self._publish_odom()
            with self._lock:
                if self._goals and self._generations:
                    break
            rate.sleep()
        with self._lock:
            self.assertEqual(len(self._goals), 1)
            self.assertEqual(self._goals[0].header.seq, 7)
            self.assertEqual(self._generations[-1], 7)

        self._goal_pub.publish(self._goal(7))
        self._goal_pub.publish(self._goal(8, frame="wrong_frame"))
        deadline = rospy.Time.now() + rospy.Duration(0.5)
        while rospy.Time.now() < deadline:
            self._publish_odom()
            rate.sleep()
        with self._lock:
            self.assertEqual(len(self._goals), 1)


if __name__ == "__main__":
    rospy.init_node("daib_ego_bridge_runtime_contract_test")
    rostest.rosrun(
        "daib_ego_bridge",
        "daib_ego_bridge_runtime_contract_test",
        RuntimeContractTest,
    )
