"""
Controller sets and switch requests of control_center (no ROS, testable on its own).

The position-controlled controllers (joint_stream_controller, cartesian_jog_controller,
joint_trajectory_controller) are chained onto ff_controller, which adds feedforward torque. They
can only be active together with it, and ff_controller must be switched in the SAME request as its
upstream (a separate request restarts it and the arm sags):

    start:  activate   [ff_controller, joint_trajectory_controller]
    stop:   deactivate [joint_trajectory_controller, ff_controller]
    swap:   deactivate [joint_trajectory_controller], activate [joint_stream_controller]
"""

FEEDFORWARD_CONTROLLER = 'ff_controller'
FEEDFORWARD_UPSTREAMS = ('joint_stream_controller', 'cartesian_jog_controller',
                         'joint_trajectory_controller')


def with_feedforward(controllers, ff=FEEDFORWARD_CONTROLLER, upstreams=FEEDFORWARD_UPSTREAMS):
    """
    Return the controller set a mission really needs (list, no duplicates, order kept).

    If one of the upstreams is in the set, ff is added in front of it (downstream first); other
    sets (impedance, gc_controller) are returned unchanged.
    """
    result = list(dict.fromkeys(controllers))
    if ff not in result and any(name in upstreams for name in result):
        result.insert(0, ff)
    return result


def switch_request(current, target):
    """
    Return (activate, deactivate) to go from the active set current to target in one request.

    Controllers in both sets stay active (e.g. ff_controller when swapping upstreams). Activation
    keeps the order of target (downstream first), deactivation the reverse order of current
    (upstream first).
    """
    activate = [name for name in target if name not in current]
    deactivate = [name for name in reversed(list(current)) if name not in target]
    return activate, deactivate
