"""
Plan goals and programs with MoveIt, without executing anything.

A program is split into segments (goals.segments): consecutive Pilz steps are planned together as
one blended Pilz sequence (action /sequence_move_group, plan only), an OMPL step as one MoveGroup
request (plan only). The service /plan_sequence_path is not used: in MoveIt 2.12.4 it fails every
blended sequence ("Blending failed"), while the action plans the same request.

Every segment starts where the previous one ends, so the whole program is planned before anything
moves: if one step cannot be planned, nothing moves.

Goals are turned into MoveIt goal constraints per motion type:

- joint goal: joint constraints (PTP, OMPL and LIN; Pilz computes the LIN pose itself)
- Cartesian goal, or a goal with offset_xyz (base frame, or tool frame with offset_frame: tcp):
  - LIN: tcp pose constraint (straight line from the previous pose)
  - PTP / OMPL: joint goal from IK, seeded with the goal's seed (the arm configuration when it was
    taught) or the start of the segment. Without that, the IK solver may pick another configuration
    (elbow up/down, wrist flipped) for the same pose.
"""

from dataclasses import dataclass
import math

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    DisplayTrajectory, MotionSequenceItem, MoveItErrorCodes, PlanningOptions, RobotState,
    RobotTrajectory)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from robotarm_motion.goals import CartesianGoal, GoalFileError, JOINTS, PILZ_MOTIONS, rotate
from robotarm_motion.moveit_client import (
    BASE_FRAME, duration_s, ERROR_NAMES, GROUP, MotionError, OMPL, PILZ, TCP_LINK, wait_for)
from trajectory_msgs.msg import JointTrajectory


@dataclass
class PlannedSegment:
    """A planned segment: its steps and one joint trajectory (time from the segment start)."""

    planner: str
    steps: list
    trajectory: JointTrajectory

    @property
    def duration(self):
        return duration_s(self.trajectory.points[-1].time_from_start)

    def end_joints(self):
        return dict(zip(self.trajectory.joint_names, self.trajectory.points[-1].positions))


def concatenate(trajectories):
    """Join the JointTrajectories of a Pilz sequence response into one (times continue)."""
    out = JointTrajectory()
    out.joint_names = list(trajectories[0].joint_names)
    offset = 0.0
    for traj in trajectories:
        for point in traj.points:
            t = duration_s(point.time_from_start)
            if out.points and t == 0.0:
                continue   # every part starts with the end point of the previous one
            t += offset
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int(round((t - int(t)) * 1e9))
            out.points.append(point)
        offset = duration_s(out.points[-1].time_from_start)
    return out


class Planner:
    """Plans program segments through move_group (plan only)."""

    def __init__(self, node, moveit):
        """Use moveit (MoveItClient) for requests, FK and the MoveGroup planning action."""
        self.node = node
        self.moveit = moveit
        self._ik_client = node.create_client(GetPositionIK, 'compute_ik')
        self._sequence_client = ActionClient(node, MoveGroupSequence, 'sequence_move_group')
        self._display_pub = node.create_publisher(DisplayTrajectory, 'display_planned_path', 1)

    def wait_for_services(self, timeout=30.0):
        if not self._ik_client.wait_for_service(timeout_sec=timeout):
            raise MotionError('move_group service /compute_ik not available')
        if not self._sequence_client.wait_for_server(timeout_sec=timeout):
            raise MotionError('move_group action /sequence_move_group not available')

    # ----- goal resolution -------------------------------------------------------------------

    def target_pose(self, goal, offset, offset_frame='base'):
        """
        Return the tcp pose of a goal (FK for joint goals) plus offset_xyz [m], base frame.

        offset_frame 'tcp': the offset is given in the tool frame of the goal (rotated by its
        orientation), e.g. (0, 0, -0.05) = 5 cm back along the tool's z axis.
        """
        if isinstance(goal, CartesianGoal):
            xyz, quat = goal.xyz, goal.quat
        else:
            p = self.moveit.fk(JOINTS, [goal.joints[j] for j in JOINTS])
            xyz = (p.position.x, p.position.y, p.position.z)
            quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        offset = offset or (0.0, 0.0, 0.0)
        if offset_frame == 'tcp':
            offset = rotate(quat, offset)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (a + b for a, b in zip(xyz, offset))
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        return pose

    def ik(self, pose, seed, what):
        """Joint values {joint: rad} for a tcp pose, starting from seed {joint: rad}."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = TCP_LINK
        r.avoid_collisions = True
        r.pose_stamped.header.frame_id = BASE_FRAME
        r.pose_stamped.pose = pose
        r.robot_state = self.moveit.robot_state({j: seed[j] for j in JOINTS})
        r.timeout.nanosec = 500_000_000
        res = wait_for(self._ik_client.call_async(req), timeout=5.0)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionError(f'{what}: no IK solution for the tcp pose '
                              f'({ERROR_NAMES.get(res.error_code.val, res.error_code.val)})')
        solution = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        jump = max(abs(solution[j] - seed[j]) for j in JOINTS)
        if jump > math.radians(90):
            self.node.get_logger().warn(
                f'{what}: IK solution is {math.degrees(jump):.0f} deg from its seed in one joint, '
                'it may be another arm configuration; add a seed (teach the goal)')
        return {j: solution[j] for j in JOINTS}

    def constraints(self, goal_file, step, seed):
        """Return the goal constraints of a step; seed {joint: rad} for IK (PTP/OMPL only)."""
        goal = goal_file.goal(step.goal)
        what = f'step to {step.goal!r}'
        if not isinstance(goal, CartesianGoal) and step.offset_xyz is None:
            return self.moveit.joint_goal(goal.joints)
        pose = self.target_pose(goal, step.offset_xyz, step.offset_frame)
        if step.motion == 'LIN':
            return self.moveit.pose_goal(pose)
        if isinstance(goal, CartesianGoal):
            ik_seed = goal.seed or seed
        else:
            ik_seed = goal.joints   # joint goal + offset: stay in the goal's configuration
        return self.moveit.joint_goal(self.ik(pose, ik_seed, what))

    # ----- planning --------------------------------------------------------------------------

    def _request(self, pipeline, planner, constraints, scaling, start):
        req = self.moveit.request(pipeline, planner, constraints, start_state=start)
        req.max_velocity_scaling_factor = scaling
        req.max_acceleration_scaling_factor = scaling
        return req

    def plan_segment(self, goal_file, segment, scaling, start_joints):
        """Plan one segment from start_joints {joint: rad}; returns a PlannedSegment."""
        start = self.moveit.robot_state({j: start_joints[j] for j in JOINTS})
        if segment.planner == 'ompl':
            step = segment.steps[0]
            req = self._request(OMPL, '', self.constraints(goal_file, step, start_joints),
                                step.scaling or scaling, start)
            res = self.moveit.run(req, plan_only=True)
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                raise MotionError(f'OMPL step to {step.goal!r}: '
                                  f'{ERROR_NAMES.get(res.error_code.val, res.error_code.val)}')
            return PlannedSegment('ompl', segment.steps, res.planned_trajectory.joint_trajectory)

        seq = MoveGroupSequence.Goal()
        seq.planning_options = PlanningOptions(plan_only=True)
        seq.planning_options.planning_scene_diff.is_diff = True
        seq.planning_options.planning_scene_diff.robot_state.is_diff = True
        for i, step in enumerate(segment.steps):
            item = MotionSequenceItem()
            item.req = self._request(PILZ, step.motion,
                                     self.constraints(goal_file, step, start_joints),
                                     step.scaling or scaling,
                                     start if i == 0 else RobotState())  # only the first item
            item.blend_radius = step.blend
            seq.request.items.append(item)
        handle = wait_for(self._sequence_client.send_goal_async(seq), timeout=10.0)
        if not handle.accepted:
            raise MotionError('move_group rejected the Pilz sequence request')
        res = wait_for(handle.get_result_async(), timeout=60.0).result.response
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            goals = ' -> '.join(f'{s.goal} ({s.motion})' for s in segment.steps)
            raise MotionError(f'Pilz sequence {goals}: '
                              f'{ERROR_NAMES.get(res.error_code.val, res.error_code.val)} '
                              '(see the move_group log for the reason)')
        trajectory = concatenate([t.joint_trajectory for t in res.planned_trajectories])
        return PlannedSegment('pilz', segment.steps, trajectory)

    def plan(self, goal_file, segments, scaling, start_joints, attempts=2):
        """
        Plan all segments, each from the end of the previous one; returns PlannedSegments.

        A segment that fails is planned once more (attempts=2): the IK solver (pick_ik) and OMPL
        are randomized, and rarely (about 1 in 30 plans of the example square) the IK of a Pilz LIN
        jumps to another arm configuration in the middle of the line. Pilz then rejects the plan
        ("Joint velocity limit ... violated"), the second attempt normally succeeds. Only planning
        is repeated; nothing moves before the whole job has been planned.
        """
        planned, start = [], dict(start_joints)
        for index, segment in enumerate(segments):
            for attempt in range(1, attempts + 1):
                try:
                    planned.append(self.plan_segment(goal_file, segment, scaling, start))
                    break
                except MotionError as e:
                    if attempt == attempts:
                        e.segment = index   # for the caller: which steps failed
                        raise
                    self.node.get_logger().warn(f'planning failed ({e}), planning again')
            start = planned[-1].end_joints()
        return planned

    def display(self, planned, start_joints):
        """Show the planned job in RViz (MotionPlanning display, topic display_planned_path)."""
        msg = DisplayTrajectory()
        msg.trajectory_start = self.moveit.robot_state({j: start_joints[j] for j in JOINTS})
        for seg in planned:
            robot_traj = RobotTrajectory()
            robot_traj.joint_trajectory = seg.trajectory
            msg.trajectory.append(robot_traj)
        self._display_pub.publish(msg)


def check_blends(planner, goal_file, steps):
    """
    Blend radii Pilz would refuse, before planning: [(step number, message), ...].

    Within a run of Pilz steps, as Pilz checks them: a blend radius must be smaller than the
    distance to the previous and to the next goal, and two neighbouring blend spheres must not
    overlap. tcp positions come from target_pose (FK for joint goals); the start of the first step
    is the current pose, which is unknown here, so that distance is not checked.
    """
    problems = []
    positions = [None] * len(steps)

    def position(i):
        if positions[i] is None:
            s = steps[i]
            p = planner.target_pose(goal_file.goal(s.goal), s.offset_xyz, s.offset_frame).position
            positions[i] = (p.x, p.y, p.z)
        return positions[i]

    def dist(i, j):
        return math.dist(position(i), position(j))

    for i, step in enumerate(steps):
        last = i == len(steps) - 1
        if not step.blend or last or step.motion not in PILZ_MOTIONS \
                or steps[i + 1].motion not in PILZ_MOTIONS:
            continue   # no blending here (segments() sets these radii to 0 and reports them)
        r, n = step.blend, i + 1
        if r >= dist(i, i + 1):
            problems.append((n, f'step {n}: blend {r * 1000:.0f} mm is not smaller than the '
                                f'distance to the next goal ({dist(i, i + 1) * 1000:.0f} mm)'))
        elif i > 0 and r >= dist(i - 1, i):
            d = dist(i - 1, i) * 1000
            problems.append((n, f'step {n}: blend {r * 1000:.0f} mm is not smaller than the '
                                f'distance from the previous goal ({d:.0f} mm)'))
        blends_next = i + 2 < len(steps) and steps[i + 2].motion in PILZ_MOTIONS
        nxt = steps[i + 1].blend if blends_next else 0.0
        if nxt and r + nxt >= dist(i, i + 1):
            d = dist(i, i + 1) * 1000
            problems.append((n, f'steps {n} and {n + 1}: blend radii {r * 1000:.0f} + '
                                f'{nxt * 1000:.0f} mm overlap (goals {d:.0f} mm apart)'))
    return problems


def check_goal_file_steps(goal_file, steps):
    """Fail early (before planning) if a step refers to an unknown goal."""
    for step in steps:
        goal_file.goal(step.goal)
    if not steps:
        raise GoalFileError('nothing to do: no steps')
