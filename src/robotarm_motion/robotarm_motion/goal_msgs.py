"""Conversions between the goals file model (goals.py) and the robotarm_interface messages."""

from robotarm_interface.msg import MotionGoal, MotionProgram, MotionStep
from robotarm_motion.goals import CartesianGoal, JointGoal, JOINTS, Step


def goal_to_msg(goal):
    """Convert a JointGoal or CartesianGoal into a MotionGoal."""
    msg = MotionGoal(name=goal.name)
    msg.source = MotionGoal.SOURCE_SRDF if goal.source == 'srdf' else MotionGoal.SOURCE_FILE
    if isinstance(goal, CartesianGoal):
        msg.type = MotionGoal.TYPE_CARTESIAN
        p, o = msg.pose.position, msg.pose.orientation
        p.x, p.y, p.z = (float(v) for v in goal.xyz)
        o.x, o.y, o.z, o.w = (float(v) for v in goal.quat)
        if goal.seed:
            msg.seed = [float(goal.seed[j]) for j in JOINTS]
    else:
        msg.type = MotionGoal.TYPE_JOINT
        msg.joints = [float(goal.joints[j]) for j in JOINTS]
    return msg


def goal_from_msg(msg):
    """Convert a MotionGoal into a JointGoal or CartesianGoal."""
    source = 'srdf' if msg.source == MotionGoal.SOURCE_SRDF else 'file'
    if msg.type == MotionGoal.TYPE_CARTESIAN:
        p, o = msg.pose.position, msg.pose.orientation
        seed = dict(zip(JOINTS, msg.seed)) if len(msg.seed) else None
        return CartesianGoal(msg.name, (p.x, p.y, p.z), (o.x, o.y, o.z, o.w), seed, source)
    return JointGoal(msg.name, dict(zip(JOINTS, msg.joints)), source)


def program_to_msg(name, steps):
    """Convert a program (list of Step) into a MotionProgram."""
    return MotionProgram(name=name, steps=[
        MotionStep(goal=s.goal, motion=s.motion, blend=float(s.blend),
                   scaling=float(s.scaling or 0.0),
                   offset_xyz=[float(v) for v in s.offset_xyz or ()])
        for s in steps])


def steps_from_msg(msg):
    """Convert a MotionProgram into its list of Step."""
    return [Step(s.goal, s.motion, s.blend, s.scaling or None,
                 tuple(s.offset_xyz) if len(s.offset_xyz) else None)
            for s in msg.steps]
