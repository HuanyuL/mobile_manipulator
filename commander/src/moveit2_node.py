import copy
import threading
from enum import Enum
from typing import Any, List, Optional, Tuple, Union

import numpy as np
from action_msgs.msg import GoalStatus
from moveit.planning import PlanningSceneInterface
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetMotionPlan,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from sensor_msgs.msg import JointState
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from std_msgs.msg import Header, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MoveIt2Node:
    def __init__(self):
        super().__init__("moveit2_node")
        self.node = Node("moveit2_node")
        self._action_client = ActionClient(self, MoveGroup, "move_group")
        self.joint_names = moveitcommander.RobotCommander().get_joint_names()
