"""
Goals and programs of the robotarm: the goals file (YAML).

ROS free, so the file format can be tested without a robot. Format::

    goals:
      above_part:                      # joint goal, degrees (list in axis order or dict)
        joints_deg: [20, -60, 80, 0, 50, 0]
      place:                           # Cartesian goal of the tcp in the base frame
        xyz: [0.40, 0.10, 0.30]        # [m]
        rpy_deg: [180, 0, 90]          # roll, pitch, yaw [deg] (fixed axes x, y, z, as in URDF)
        seed_deg: [10, -55, 75, 0, 60, 0]   # optional: arm configuration the IK should keep
    programs:
      demo:
        - ready                        # short form: goal name, PTP
        - {goal: place, motion: LIN, blend: 0.02, scaling: 0.3, offset_xyz: [0, 0, 0.05]}
        - {goal: above_part, motion: OMPL}

Joint goals can also be given in radians (``joints: [...]``), Cartesian orientations as quaternion
(``quat: [x, y, z, w]``). The named states of the SRDF (home, ready) are available as joint goals
without being in the file; a goal in the file with the same name overrides them.
"""

from dataclasses import dataclass, field, replace
import math
import os

import yaml

JOINTS = ['axis1', 'axis2', 'axis3', 'axis4', 'axis5', 'axis6']
FRAME = 'base'
MOTIONS = ('PTP', 'LIN', 'OMPL')
PILZ_MOTIONS = ('PTP', 'LIN')

HEADER = """\
# robotarm goals file, see robotarm_motion/README.md ("Goals file").
# Written by `ros2 run robotarm_motion motion teach/remove`: comments added by hand are lost then.
"""


class GoalFileError(ValueError):
    """The goals file or a request against it is invalid."""


# ----- rotations ---------------------------------------------------------------------------------

def quat_from_rpy(roll, pitch, yaw):
    """Quaternion (x, y, z, w) of R = Rz(yaw) Ry(pitch) Rx(roll), angles in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rpy_from_quat(x, y, z, w):
    """Inverse of quat_from_rpy: (roll, pitch, yaw) in radians."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def normalized(q):
    norm = math.sqrt(sum(c * c for c in q))
    if norm < 1e-9:
        raise GoalFileError('quaternion of length 0')
    return tuple(c / norm for c in q)


# ----- model -------------------------------------------------------------------------------------

@dataclass
class JointGoal:
    """Joint values {joint: rad} for all JOINTS."""

    name: str
    joints: dict
    source: str = 'file'   # 'file' or 'srdf'

    def to_yaml(self):
        return {'joints_deg': [round(math.degrees(self.joints[j]), 3) for j in JOINTS]}


@dataclass
class CartesianGoal:
    """tcp pose in FRAME: position [m], orientation quaternion (x, y, z, w), optional IK seed."""

    name: str
    xyz: tuple
    quat: tuple
    seed: dict = None
    source: str = 'file'

    def to_yaml(self):
        entry = {'xyz': [round(v, 5) for v in self.xyz],
                 'rpy_deg': [round(math.degrees(a), 3) for a in rpy_from_quat(*self.quat)]}
        if self.seed:
            entry['seed_deg'] = [round(math.degrees(self.seed[j]), 3) for j in JOINTS]
        return entry


@dataclass
class Step:
    """One program step: move to goal with motion, blending into the next step by blend [m]."""

    goal: str
    motion: str = 'PTP'
    blend: float = 0.0
    scaling: float = None          # None: the job's scaling
    offset_xyz: tuple = None       # [m] in FRAME, added to the goal's tcp position

    def to_yaml(self):
        if (self.motion, self.blend, self.scaling, self.offset_xyz) == ('PTP', 0.0, None, None):
            return self.goal
        entry = {'goal': self.goal, 'motion': self.motion}
        if self.blend:
            entry['blend'] = self.blend
        if self.scaling is not None:
            entry['scaling'] = self.scaling
        if self.offset_xyz is not None:
            entry['offset_xyz'] = list(self.offset_xyz)
        return entry


@dataclass
class Segment:
    """Steps planned and executed as one trajectory: a blended Pilz sequence or one OMPL motion."""

    planner: str                  # 'pilz' or 'ompl'
    steps: list = field(default_factory=list)


# ----- parsing -----------------------------------------------------------------------------------

def _vector(value, length, what):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise GoalFileError(f'{what}: expected a list of {length} numbers, got {value!r}')
    try:
        return tuple(float(v) for v in value)
    except (TypeError, ValueError):
        raise GoalFileError(f'{what}: expected numbers, got {value!r}')


def _joint_values(value, what, degrees):
    """Joint values as list (JOINTS order) or dict -> {joint: rad}."""
    if isinstance(value, dict):
        unknown = set(value) - set(JOINTS)
        missing = set(JOINTS) - set(value)
        if unknown or missing:
            raise GoalFileError(f'{what}: needs exactly the joints {JOINTS} '
                                f'(unknown {sorted(unknown)}, missing {sorted(missing)})')
        values = [float(value[j]) for j in JOINTS]
    else:
        values = _vector(value, len(JOINTS), what)
    return {j: math.radians(v) if degrees else v for j, v in zip(JOINTS, values)}


def parse_goal(name, entry):
    """Goal from its YAML entry."""
    what = f'goal {name!r}'
    if not isinstance(entry, dict):
        raise GoalFileError(f'{what}: expected a mapping, got {entry!r}')
    keys = set(entry)
    if keys & {'joints', 'joints_deg'}:
        if keys - {'joints', 'joints_deg'} or keys == {'joints', 'joints_deg'}:
            raise GoalFileError(
                f'{what}: a joint goal has either joints or joints_deg, nothing else')
        degrees = 'joints_deg' in entry
        return JointGoal(name, _joint_values(entry['joints_deg' if degrees else 'joints'],
                                             what, degrees))
    if 'xyz' in keys:
        allowed = {'xyz', 'rpy_deg', 'quat', 'frame', 'seed_deg', 'seed'}
        if keys - allowed:
            raise GoalFileError(f'{what}: unknown keys {sorted(keys - allowed)}')
        if entry.get('frame', FRAME) != FRAME:
            raise GoalFileError(f'{what}: only frame {FRAME!r} is supported')
        if ('rpy_deg' in keys) == ('quat' in keys):
            raise GoalFileError(f'{what}: give the orientation as rpy_deg or quat (one of them)')
        if 'rpy_deg' in keys:
            quat = quat_from_rpy(*(math.radians(a) for a in _vector(entry['rpy_deg'], 3, what)))
        else:
            quat = normalized(_vector(entry['quat'], 4, what))
        seed = None
        if 'seed_deg' in keys or 'seed' in keys:
            degrees = 'seed_deg' in keys
            seed = _joint_values(entry['seed_deg' if degrees else 'seed'], f'{what} seed', degrees)
        return CartesianGoal(name, _vector(entry['xyz'], 3, what), quat, seed)
    raise GoalFileError(f'{what}: needs joints_deg/joints (joint goal) or xyz (Cartesian goal)')


def parse_step(entry, what):
    """Program step from its YAML entry (goal name or mapping)."""
    if isinstance(entry, str):
        return Step(entry)
    if not isinstance(entry, dict) or 'goal' not in entry:
        raise GoalFileError(
            f'{what}: expected a goal name or a mapping with "goal", got {entry!r}')
    allowed = {'goal', 'motion', 'blend', 'scaling', 'offset_xyz'}
    if set(entry) - allowed:
        raise GoalFileError(f'{what}: unknown keys {sorted(set(entry) - allowed)}')
    motion = str(entry.get('motion', 'PTP')).upper()
    if motion not in MOTIONS:
        raise GoalFileError(f'{what}: motion must be one of {MOTIONS}, got {motion!r}')
    blend = float(entry.get('blend', 0.0))
    if blend < 0:
        raise GoalFileError(f'{what}: blend must be >= 0')
    scaling = entry.get('scaling')
    if scaling is not None and not 0 < float(scaling) <= 1:
        raise GoalFileError(f'{what}: scaling must be in (0, 1]')
    offset = entry.get('offset_xyz')
    return Step(str(entry['goal']), motion, blend,
                None if scaling is None else float(scaling),
                None if offset is None else _vector(offset, 3, f'{what} offset_xyz'))


# ----- file --------------------------------------------------------------------------------------

def default_goals_file():
    """$ROS_HOME/robotarm_motion/goals.yaml (ROS_HOME defaults to ~/.ros)."""
    ros_home = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
    return os.path.join(ros_home, 'robotarm_motion', 'goals.yaml')


class GoalFile:
    """Goals and programs of one YAML file, plus the SRDF named states as joint goals."""

    def __init__(self, path, named_states=None):
        """Read path (missing file = empty); named_states {name: {joint: rad}} from the SRDF."""
        self.path = path
        self.goals = {}
        self.programs = {}
        for name, joints in (named_states or {}).items():
            self.goals[name] = JointGoal(name, dict(joints), source='srdf')
        data = {}
        if os.path.exists(path):
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        if not isinstance(data, dict) or set(data) - {'goals', 'programs'}:
            raise GoalFileError(f'{path}: top level must be a mapping with goals and programs')
        for name, entry in (data.get('goals') or {}).items():
            self.goals[str(name)] = parse_goal(str(name), entry)
        for name, steps in (data.get('programs') or {}).items():
            if not isinstance(steps, list) or not steps:
                raise GoalFileError(f'program {name!r}: expected a non-empty list of steps')
            self.programs[str(name)] = [parse_step(s, f'program {name!r} step {i + 1}')
                                        for i, s in enumerate(steps)]
        for name, steps in self.programs.items():
            for i, step in enumerate(steps):
                if step.goal not in self.goals:
                    raise GoalFileError(f'program {name!r} step {i + 1}: unknown goal '
                                        f'{step.goal!r}')

    def goal(self, name):
        if name not in self.goals:
            raise GoalFileError(f'unknown goal {name!r} (known: {", ".join(sorted(self.goals))})')
        return self.goals[name]

    def program(self, name):
        if name not in self.programs:
            raise GoalFileError(f'unknown program {name!r} '
                                f'(known: {", ".join(sorted(self.programs)) or "none"})')
        return self.programs[name]

    def set_goal(self, goal):
        self.goals[goal.name] = goal

    def remove_goal(self, name):
        if name not in self.goals or self.goals[name].source != 'file':
            raise GoalFileError(f'no goal {name!r} in {self.path}')
        users = [p for p, steps in self.programs.items() if any(s.goal == name for s in steps)]
        if users:
            raise GoalFileError(f'goal {name!r} is used by program(s) {", ".join(users)}')
        del self.goals[name]

    def save(self):
        """Write goals (without the SRDF ones) and programs to path."""
        data = {
            'goals': {n: g.to_yaml() for n, g in self.goals.items() if g.source == 'file'},
            'programs': {n: [s.to_yaml() for s in steps] for n, steps in self.programs.items()},
        }
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            f.write(HEADER)
            yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)
        os.replace(tmp, self.path)


# ----- segmentation ------------------------------------------------------------------------------

def segments(steps):
    """
    Split program steps into segments planned and executed as one trajectory each.

    Consecutive Pilz steps (PTP, LIN) form one Pilz sequence; their blend radii round the
    transitions. An OMPL step is a segment of its own, the robot stops before and after it.
    Returns (segments, notes); notes explain blend radii that cannot apply and were set to 0.
    """
    result, notes = [], []
    for step in (replace(s) for s in steps):   # copies: blend radii are adjusted below
        planner = 'pilz' if step.motion in PILZ_MOTIONS else 'ompl'
        if planner == 'pilz' and result and result[-1].planner == 'pilz':
            result[-1].steps.append(step)
        else:
            result.append(Segment(planner, [step]))
    for seg in result:
        last = seg.steps[-1]
        if last.blend:
            notes.append(f'blend {last.blend} m on the {"OMPL " if seg.planner == "ompl" else ""}'
                         f'step to {last.goal!r} ignored: the robot stops there '
                         f'(end of the program or an OMPL step follows)')
        for step in seg.steps:
            if seg.planner == 'ompl' or step is last:
                step.blend = 0.0
    return result, notes
