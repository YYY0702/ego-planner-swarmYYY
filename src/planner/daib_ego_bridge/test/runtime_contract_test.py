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
        self._source_generation_pub = rospy.Publisher(
            "/daib_explorer/generation", UInt64, queue_size=1, latch=True
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

    def _publish_odom(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "camera_init"
        odom.pose.pose.orientation.w = 1.0
        self._odom_pub.publish(odom)

    @staticmethod
    def _goal(stamp, frame="camera_init"):
        goal = PoseStamped()
        goal.header.stamp = stamp
        goal.header.frame_id = frame
        goal.pose.position.x = 5.0
        goal.pose.position.y = 1.0
        goal.pose.position.z = 1.5
        goal.pose.orientation.w = 1.0
        return goal

    def test_ready_frame_and_generation_gate(self):
        rate = rospy.Rate(20)
        # Reproduce the bag evidence: sensor headers use a different epoch
        # from ros::Time::now(), while goal and odometry remain synchronized.
        sensor_stamp = rospy.Time.from_sec(946685437.0)
        source_goal = self._goal(sensor_stamp)
        self._source_generation_pub.publish(UInt64(data=7))
        self._goal_pub.publish(source_goal)
        self._ready_pub.publish(Bool(data=False))
        deadline = rospy.Time.now() + rospy.Duration(1.0)
        while rospy.Time.now() < deadline:
            self._publish_odom(sensor_stamp)
            rate.sleep()
        with self._lock:
            self.assertEqual(len(self._goals), 0)

        self._ready_pub.publish(Bool(data=True))
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while rospy.Time.now() < deadline:
            self._publish_odom(sensor_stamp)
            with self._lock:
                if self._goals and self._generations:
                    break
            rate.sleep()
        with self._lock:
            self.assertEqual(len(self._goals), 1)
            self.assertEqual(self._goals[0].header.stamp, source_goal.header.stamp)
            self.assertEqual(self._generations[-1], 7)

        self._source_generation_pub.publish(UInt64(data=7))
        self._goal_pub.publish(source_goal)
        self._source_generation_pub.publish(UInt64(data=8))
        self._goal_pub.publish(self._goal(sensor_stamp, frame="wrong_frame"))
        self._source_generation_pub.publish(UInt64(data=9))
        self._goal_pub.publish(
            self._goal(sensor_stamp - rospy.Duration(20.0))
        )
        deadline = rospy.Time.now() + rospy.Duration(0.5)
        while rospy.Time.now() < deadline:
            self._publish_odom(sensor_stamp)
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
