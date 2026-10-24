"""Tests of the goals file format and program segmentation (no ROS needed)."""

import math

import pytest

from robotarm_motion.goals import (
    CartesianGoal, GoalFile, GoalFileError, JointGoal, JOINTS, parse_goal, parse_step,
    quat_from_rpy, rpy_from_quat, segments, Step)

READY = dict(zip(JOINTS, [0.0, -1.3, 1.5, 0.0, 1.0, 0.0]))


def write(tmp_path, text):
    path = tmp_path / 'goals.yaml'
    path.write_text(text)
    return str(path)


def test_rpy_quat_round_trip():
    for rpy in [(0.1, -0.4, 2.0), (math.pi, 0.0, math.pi / 2), (0.0, 0.0, 0.0)]:
        q = quat_from_rpy(*rpy)
        assert math.isclose(sum(c * c for c in q), 1.0)
        back = quat_from_rpy(*rpy_from_quat(*q))
        # same rotation (q and -q are equivalent)
        assert math.isclose(abs(sum(a * b for a, b in zip(q, back))), 1.0, abs_tol=1e-9)


def test_quat_from_rpy_known_value():
    # rpy (180, 0, 90) deg: tool pointing down, rotated 90 deg about z
    q = quat_from_rpy(math.pi, 0.0, math.pi / 2)
    expected = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
    assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(q, expected))


def test_joint_goal_formats():
    g = parse_goal('a', {'joints_deg': [0, -90, 90, 0, 45, 0]})
    assert isinstance(g, JointGoal)
    assert math.isclose(g.joints['axis2'], -math.pi / 2)
    g = parse_goal('b', {'joints': {'axis1': 0.1, 'axis2': 0.2, 'axis3': 0.3,
                                    'axis4': 0.4, 'axis5': 0.5, 'axis6': 0.6}})
    assert g.joints['axis6'] == 0.6


@pytest.mark.parametrize('entry', [
    {'joints_deg': [1, 2, 3]},                         # too short
    {'joints_deg': {'axis1': 0}},                      # missing joints
    {'joints': [0] * 6, 'joints_deg': [0] * 6},        # both
    {'xyz': [0.4, 0, 0.3]},                            # no orientation
    {'xyz': [0.4, 0, 0.3], 'rpy_deg': [0, 0, 0], 'quat': [0, 0, 0, 1]},
    {'xyz': [0.4, 0, 0.3], 'rpy_deg': [0, 0, 0], 'frame': 'world'},
    {'speed': 1},                                      # neither type
])
def test_invalid_goals(entry):
    with pytest.raises(GoalFileError):
        parse_goal('x', entry)


def test_cartesian_goal_with_seed():
    g = parse_goal('p', {'xyz': [0.4, 0.1, 0.3], 'rpy_deg': [180, 0, 90],
                         'seed_deg': [10, -55, 75, 0, 60, 0]})
    assert isinstance(g, CartesianGoal)
    assert g.xyz == (0.4, 0.1, 0.3)
    assert math.isclose(g.seed['axis3'], math.radians(75))
    q = parse_goal('q', {'xyz': [0, 0, 0], 'quat': [0, 0, 2, 0]}).quat
    assert q == (0.0, 0.0, 1.0, 0.0)   # normalized


def test_steps():
    assert parse_step('ready', 's') == Step('ready')
    s = parse_step({'goal': 'p', 'motion': 'lin', 'blend': 0.02, 'scaling': 0.5,
                    'offset_xyz': [0, 0, 0.05]}, 's')
    assert (s.motion, s.blend, s.scaling, s.offset_xyz) == ('LIN', 0.02, 0.5, (0, 0, 0.05))
    for bad in [{'goal': 'p', 'motion': 'CIRC'}, {'goal': 'p', 'scaling': 2},
                {'goal': 'p', 'blend': -1}, {'motion': 'PTP'}, {'goal': 'p', 'speed': 1}]:
        with pytest.raises(GoalFileError):
            parse_step(bad, 's')


def test_file_load_srdf_states_and_override(tmp_path):
    path = write(tmp_path, """
goals:
  ready: {joints_deg: [0, 0, 0, 0, 10, 0]}
  place: {xyz: [0.4, 0.1, 0.3], rpy_deg: [180, 0, 90]}
programs:
  demo: [home, {goal: place, motion: LIN}, ready]
""")
    f = GoalFile(path, {'home': dict.fromkeys(JOINTS, 0.0), 'ready': READY})
    assert f.goal('home').source == 'srdf'
    assert f.goal('ready').source == 'file'           # file overrides the SRDF state
    assert math.isclose(f.goal('ready').joints['axis5'], math.radians(10))
    assert [s.goal for s in f.program('demo')] == ['home', 'place', 'ready']
    with pytest.raises(GoalFileError):
        f.goal('nope')
    with pytest.raises(GoalFileError):
        f.program('nope')


def test_missing_file_is_empty(tmp_path):
    f = GoalFile(str(tmp_path / 'none.yaml'), {'home': dict.fromkeys(JOINTS, 0.0)})
    assert list(f.goals) == ['home'] and f.programs == {}


def test_program_with_unknown_goal(tmp_path):
    with pytest.raises(GoalFileError, match='unknown goal'):
        GoalFile(write(tmp_path, 'programs:\n  p: [nowhere]\n'))


def test_save_round_trip(tmp_path):
    path = str(tmp_path / 'sub' / 'goals.yaml')
    f = GoalFile(path, {'home': dict.fromkeys(JOINTS, 0.0)})
    f.set_goal(JointGoal('a', dict(READY)))
    f.set_goal(CartesianGoal('p', (0.4, 0.1, 0.3), quat_from_rpy(math.pi, 0, 0.5),
                             seed=dict(READY)))
    f.programs['demo'] = [Step('a'), Step('p', 'LIN', 0.02, 0.3, (0, 0, 0.05)), Step('home')]
    f.save()
    g = GoalFile(path, {'home': dict.fromkeys(JOINTS, 0.0)})
    assert 'home' not in open(path).read().split('programs')[0]   # SRDF states not written
    assert all(math.isclose(g.goal('a').joints[j], READY[j], abs_tol=1e-4) for j in JOINTS)
    p = g.goal('p')
    assert all(math.isclose(a, b, abs_tol=1e-5) for a, b in zip(p.xyz, (0.4, 0.1, 0.3)))
    assert math.isclose(abs(sum(a * b for a, b in zip(p.quat, quat_from_rpy(math.pi, 0, 0.5)))),
                        1.0, abs_tol=1e-6)
    assert g.program('demo') == f.programs['demo']


def test_remove_goal(tmp_path):
    f = GoalFile(write(tmp_path, """
goals:
  a: {joints_deg: [0, 0, 0, 0, 0, 0]}
  b: {joints_deg: [0, 0, 0, 0, 0, 0]}
programs:
  p: [a]
"""), {'home': dict.fromkeys(JOINTS, 0.0)})
    f.remove_goal('b')
    with pytest.raises(GoalFileError, match='used by'):
        f.remove_goal('a')
    with pytest.raises(GoalFileError):
        f.remove_goal('home')          # SRDF states cannot be removed


def test_segments():
    steps = [Step('a'), Step('b', 'LIN', 0.02), Step('c', 'OMPL', 0.05), Step('d'),
             Step('e', 'LIN', 0.01), Step('f', 'PTP', 0.03)]
    segs, notes = segments(steps)
    assert [(s.planner, [x.goal for x in s.steps]) for s in segs] == [
        ('pilz', ['a', 'b']), ('ompl', ['c']), ('pilz', ['d', 'e', 'f'])]
    # blend kept inside a Pilz sequence, removed where the robot has to stop
    assert [x.blend for x in segs[0].steps] == [0.0, 0.0]
    assert [x.blend for x in segs[2].steps] == [0.0, 0.01, 0.0]
    assert len(notes) == 3                     # b (before OMPL), c (OMPL), f (end)
    assert steps[1].blend == 0.02              # the program itself is not modified
