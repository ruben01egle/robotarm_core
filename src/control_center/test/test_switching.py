"""Tests of the controller sets and switch requests (no ROS needed)."""

from control_center.switching import switch_request, with_feedforward
import pytest

FF = 'ff_controller'
JSC = 'joint_stream_controller'
CJC = 'cartesian_jog_controller'
JTC = 'joint_trajectory_controller'
GC = 'gc_controller'
IMP = 'cartesian_impedance_controller'
JSC_IMP = 'joint_stream_controller_impedance'
CJC_IMP = 'cartesian_jog_controller_impedance'


@pytest.mark.parametrize('requested, expected', [
    # the seven valid combinations of the firmware
    ([JSC], [FF, JSC]),
    ([CJC], [FF, CJC]),
    ([GC], [GC]),
    ([IMP], [IMP]),
    ([IMP, JSC_IMP], [IMP, JSC_IMP]),
    ([IMP, CJC_IMP], [IMP, CJC_IMP]),
    ([JTC], [FF, JTC]),
    # a client that already names ff_controller, and duplicates
    ([FF, JTC], [FF, JTC]),
    ([JTC, FF], [JTC, FF]),
    ([JTC, JTC], [FF, JTC]),
])
def test_with_feedforward(requested, expected):
    assert with_feedforward(requested) == expected


def test_start_from_armed_in_one_request():
    assert switch_request([], [FF, JTC]) == ([FF, JTC], [])


def test_stop_deactivates_upstream_and_ff_together():
    assert switch_request([FF, JTC], []) == ([], [JTC, FF])


def test_swap_upstreams_keeps_ff_active():
    assert switch_request([FF, JTC], [FF, JSC]) == ([JSC], [JTC])


def test_leave_position_control():
    assert switch_request([FF, JSC], [GC]) == ([GC], [JSC, FF])


def test_impedance_sets_untouched():
    assert switch_request([], [IMP, JSC_IMP]) == ([IMP, JSC_IMP], [])
    assert switch_request([IMP, CJC_IMP], []) == ([], [CJC_IMP, IMP])
