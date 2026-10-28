"""Conversions between the goals file model (goals.py) and the robotarm_interface messages."""

from robotarm_interface.msg import MotionGoal, MotionProgram, MotionStep
from robotarm_motion.goals import CartesianGoal, GoalFileError, JointGoal, JOINTS, Step

OFFSET_FRAMES = {'base': MotionStep.OFFSET_BASE, 'tcp': MotionStep.OFFSET_TCP}


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
    """Convert a MotionGoal into a JointGoal or CartesianGoal (GoalFileError on wrong lengths)."""
    source = 'srdf' if msg.source == MotionGoal.SOURCE_SRDF else 'file'
    if msg.type == MotionGoal.TYPE_CARTESIAN:
        if len(msg.seed) not in (0, len(JOINTS)):
            raise GoalFileError(f'goal {msg.name!r}: the seed needs {len(JOINTS)} values')
        p, o = msg.pose.position, msg.pose.orientation
        seed = dict(zip(JOINTS, msg.seed)) if len(msg.seed) else None
        return CartesianGoal(msg.name, (p.x, p.y, p.z), (o.x, o.y, o.z, o.w), seed, source)
    if msg.type != MotionGoal.TYPE_JOINT:
        raise GoalFileError(f'goal {msg.name!r}: unknown type {msg.type}')
    if len(msg.joints) != len(JOINTS):
        raise GoalFileError(f'goal {msg.name!r}: a joint goal needs {len(JOINTS)} values')
    return JointGoal(msg.name, dict(zip(JOINTS, msg.joints)), source)


def program_to_msg(name, steps):
    """Convert a program (list of Step) into a MotionProgram."""
    return MotionProgram(name=name, steps=[
        MotionStep(goal=s.goal, motion=s.motion, blend=float(s.blend),
                   scaling=float(s.scaling or 0.0),
                   offset_xyz=[float(v) for v in s.offset_xyz or ()],
                   offset_frame=OFFSET_FRAMES[s.offset_frame])
        for s in steps])


def steps_from_msg(steps):
    """Convert a list of MotionStep (e.g. MotionProgram.steps) into a list of Step."""
    frames = {v: k for k, v in OFFSET_FRAMES.items()}
    result = []
    for i, s in enumerate(steps, 1):
        if s.offset_frame not in frames:
            raise GoalFileError(f'step {i}: unknown offset_frame {s.offset_frame}', step=i)
        result.append(Step(s.goal, s.motion.upper(), s.blend, s.scaling or None,
                           tuple(s.offset_xyz) if len(s.offset_xyz) else None,
                           frames[s.offset_frame]))
    return result
