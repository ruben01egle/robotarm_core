#!/usr/bin/env python3
"""
Compare Pilz trajectories with and without the jerk-limiting filter (trajectory_smoother).

Plans each program once (plan only, nothing moves) and takes, per Pilz segment, the raw Pilz
trajectory and the same trajectory after /smooth_trajectory, so both start from identical IK
samples. Both are evaluated the way joint_trajectory_controller executes them: quintic splines
through position, velocity and acceleration of the points. For every program it writes a figure
with position, velocity, acceleration and jerk of one joint (both versions overlaid) and prints how
often and by how much the jerk limits of joint_limits.yaml are exceeded, over all joints.

Standalone: starts its own move_group and trajectory_smoother (move_group.launch.py, no RViz, no
execution) in a private ROS domain limited to this machine, so it never sees or disturbs a running
robot system, and stops them at the end. --use-running uses the move_group of the current domain
instead (demo.launch.py or the full system).
  source install/setup.bash
  python3 src/robotarm_motion/scripts/compare_smoothing.py [PROGRAM ...] [--start ready]
      [--scaling 0.2] [--joint axis3] [--out DIR] [--show] [--goals FILE] [--tool axis_triad]
      [--use-running]
Default programs: all programs of the goals file (default: the motion_server's). The figures are
saved as compare_<program>.png in --out; --show also opens them.
"""

import argparse
import os
import random
import signal
import subprocess
import tempfile
import threading

from ament_index_python.packages import get_package_share_directory
import matplotlib
import sys
if '--show' not in sys.argv:
    matplotlib.use('Agg')   # files only, no display needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from robotarm_motion.goals import default_goals_file, GoalFile, segments  # noqa: E402
from robotarm_motion.moveit_client import (  # noqa: E402
    duration_s, load_named_states, MoveItClient)
from robotarm_motion.planner import Planner  # noqa: E402
import yaml  # noqa: E402

DT = 0.001  # [s] evaluation grid
RAW_COLOR = '#eb6834'       # categorical slot 2 (orange)
SMOOTH_COLOR = '#2a78d6'    # categorical slot 1 (blue)
INK = '#0b0b0b'
INK_SECONDARY = '#52514e'
GRID = '#e4e3df'


def joint_limits():
    path = os.path.join(get_package_share_directory('robotarm_moveit_config'),
                        'config', 'joint_limits.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['joint_limits']


def evaluate(trajectory, t_offset=0.0):
    """Sample a JointTrajectory like joint_trajectory_controller (quintic splines).

    Returns t [n], and position, velocity, acceleration, jerk [n x joints].
    """
    t_pts = np.array([duration_s(p.time_from_start) for p in trajectory.points])
    q = np.array([p.positions for p in trajectory.points])
    v = np.array([p.velocities for p in trajectory.points])
    a = np.array([p.accelerations for p in trajectory.points])
    t = np.arange(t_pts[0], t_pts[-1], DT)
    k = np.clip(np.searchsorted(t_pts, t, side='right') - 1, 0, len(t_pts) - 2)
    T = (t_pts[k + 1] - t_pts[k])[:, None]
    s = (t - t_pts[k])[:, None]
    p0, p1, v0, v1, a0, a1 = q[k], q[k + 1], v[k], v[k + 1], a[k], a[k + 1]
    c3 = (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T**2) / (2 * T**3)
    c4 = (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T**2) / (2 * T**4)
    c5 = (12 * (p1 - p0) - 6 * (v1 + v0) * T + (a1 - a0) * T**2) / (2 * T**5)
    pos = p0 + v0 * s + a0 / 2 * s**2 + c3 * s**3 + c4 * s**4 + c5 * s**5
    vel = v0 + a0 * s + 3 * c3 * s**2 + 4 * c4 * s**3 + 5 * c5 * s**4
    acc = a0 + 6 * c3 * s + 12 * c4 * s**2 + 20 * c5 * s**3
    jerk = 6 * c3 + 24 * c4 * s + 60 * c5 * s**2
    return t + t_offset, pos, vel, acc, jerk


def join(parts):
    """Concatenate evaluated segments (t continues)."""
    return tuple(np.concatenate([p[i] for p in parts]) for i in range(5))


STOP_VELOCITY = 1e-6  # [rad/s] all joints below: at rest (as trajectory_smoother detects stops)


def rest_points(trajectory, merge=0.05):
    """Inner points where all joints are at rest: [(time, positions)].

    The filtered trajectory approaches a stop smoothly, so the output point just before it can
    already be below STOP_VELOCITY: rest points within `merge` [s] count as one stop, the last one
    (the exact stop point) is kept.
    """
    rest = []
    for p in trajectory.points[1:-1]:
        if np.all(np.abs(p.velocities) <= STOP_VELOCITY):
            t = duration_s(p.time_from_start)
            if rest and t - rest[-1][0] < merge:
                rest.pop()
            rest.append((t, np.array(p.positions)))
    return rest


def stop_errors(raw_traj, smooth_traj):
    """Per stop of the raw trajectory: max joint error [rad] of the closest filtered rest point."""
    smooth_rest = [q for _, q in rest_points(smooth_traj)]
    errors = []
    for _, q in rest_points(raw_traj):
        errors.append(min((np.max(np.abs(q - qs)) for qs in smooth_rest), default=float('inf')))
    return errors


TCP_DT = 0.01        # [s] spacing of the FK samples
WINDOW_MARGIN = 0.05  # [s] extra search range around the causal window of the filter


def tcp_samples(moveit, names, trajectory, evaluated, t_offset):
    """TCP poses along an evaluated trajectory: every TCP_DT, at its rest points and at its end.

    Returns times [n], positions [n x 3] (m) and quaternions [n x 4] (x, y, z, w), both from FK.
    """
    t, pos = evaluated[0], evaluated[1]
    step = int(round(TCP_DT / DT))
    samples = [(t[i], pos[i]) for i in range(0, len(t), step)]
    # the rest points and the end exactly as in the trajectory, not the nearest grid sample
    samples += [(tr + t_offset, q) for tr, q in rest_points(trajectory)]
    samples.append((duration_s(trajectory.points[-1].time_from_start) + t_offset,
                    np.array(trajectory.points[-1].positions)))
    samples.sort(key=lambda sample: sample[0])
    times, joints = [s[0] for s in samples], [s[1] for s in samples]
    xyz, quat = [], []
    for q in joints:
        pose = moveit.fk(names, [float(v) for v in q])
        xyz.append((pose.position.x, pose.position.y, pose.position.z))
        o = pose.orientation
        quat.append((o.x, o.y, o.z, o.w))
    return np.array(times), np.array(xyz), np.array(quat)


def pieces(raw_traj, smooth_traj, raw_offset, smooth_offset):
    """Pair the pieces between stops: [(raw start, raw end, filtered start, filtered end)].

    The filter splits at the stops, so both trajectories have the same rest points in the same
    order. If they don't match (not filtered, e.g. OMPL), the segment is one piece.
    """
    def bounds(traj, offset):
        times = [duration_s(traj.points[0].time_from_start)] + \
            [tr for tr, _ in rest_points(traj)] + [duration_s(traj.points[-1].time_from_start)]
        return [tr + offset for tr in times]
    raw_b, smooth_b = bounds(raw_traj, raw_offset), bounds(smooth_traj, smooth_offset)
    if len(raw_b) != len(smooth_b):
        raw_b, smooth_b = [raw_b[0], raw_b[-1]], [smooth_b[0], smooth_b[-1]]
    return [(raw_b[i], raw_b[i + 1], smooth_b[i], smooth_b[i + 1]) for i in range(len(raw_b) - 1)]


def quat_angle(q1, q2):
    """Angle [rad] between unit quaternions (rows of q1 against q2, broadcast)."""
    dot = np.abs(np.sum(q1 * q2, axis=-1))
    return 2 * np.arccos(np.clip(dot, -1.0, 1.0))


def path_deviation(raw, smooth, piece_list):
    """Compute the distance of every filtered TCP sample to the Pilz TCP path.

    raw, smooth: (times, positions, quaternions) from tcp_samples. The filter is causal: within a
    piece, the filtered point at t depends only on the Pilz trajectory between
    t - offset - W and t - offset (offset: delay of the piece start, W: its total filter window),
    so the nearest point is searched there (plus WINDOW_MARGIN). That keeps a path that passes the
    same place twice from matching the wrong pass.
    Returns position deviation [m] and orientation deviation [rad] per filtered sample: the
    distance to the nearest point of the position path and the angle to the nearest orientation
    on the orientation path (each searched on its own).
    """
    rt, rp, rq = raw
    st, sp, sq = smooth
    dist = np.full(len(st), np.nan)
    angle = np.full(len(st), np.nan)
    u = np.linspace(0.0, 1.0, 11)[:, None, None]   # positions along a segment, for orientation
    for raw_start, raw_end, sm_start, sm_end in piece_list:
        offset = sm_start - raw_start
        window = (sm_end - sm_start) - (raw_end - raw_start)
        for i in np.nonzero((st >= sm_start - 1e-9) & (st <= sm_end + 1e-9))[0]:
            lo = max(st[i] - offset - window - WINDOW_MARGIN, raw_start - 1e-9)
            hi = min(st[i] - offset + WINDOW_MARGIN, raw_end + 1e-9)
            k = np.nonzero((rt >= lo) & (rt <= hi))[0]
            if len(k) == 0:
                k = np.array([int(np.argmin(np.abs(rt - (st[i] - offset))))])
            if len(k) == 1:
                k = np.array([k[0], min(k[0] + 1, len(rt) - 1)])
            a, b = rp[k[:-1]], rp[k[1:]]
            ab = b - a
            length2 = np.maximum(np.sum(ab * ab, axis=1), 1e-18)
            t_seg = np.clip(np.sum((sp[i] - a) * ab, axis=1) / length2, 0.0, 1.0)
            dist[i] = np.min(np.linalg.norm(a + t_seg[:, None] * ab - sp[i], axis=1))
            qa, qb = rq[k[:-1]], rq[k[1:]]
            qb = np.where(np.sum(qa * qb, axis=1, keepdims=True) < 0, -qb, qb)  # same hemisphere
            q_line = (1 - u) * qa + u * qb
            q_line /= np.linalg.norm(q_line, axis=-1, keepdims=True)
            angle[i] = np.min(quat_angle(q_line, sq[i]))
    return dist, angle


def plot_tcp(program, t, dist, angle, stop_times, summary, out_dir):
    """Figure: TCP position and orientation deviation of the filtered trajectory over time."""
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    for ax, values, label in ((axes[0], dist * 1000, 'TCP position deviation [mm]'),
                              (axes[1], np.degrees(angle), 'TCP orientation deviation [°]')):
        style(ax)
        for ts in stop_times:
            ax.axvline(ts, color=INK_SECONDARY, linewidth=1, linestyle=':', zorder=1)
        ax.plot(t, values, color=SMOOTH_COLOR, linewidth=2)
        ax.set_ylabel(label, color=INK, fontsize=10)
        ax.set_ylim(bottom=0)
    if stop_times:
        axes[0].text(0.995, 0.97, 'dotted lines: stops (exact)', transform=axes[0].transAxes,
                     ha='right', va='top', fontsize=8, color=INK_SECONDARY)
    axes[-1].set_xlabel('time of the filtered trajectory [s]', color=INK, fontsize=10)
    fig.suptitle(f'{program}: filtered TCP vs. Pilz path', color=INK, fontsize=13, x=0.06,
                 ha='left')
    fig.text(0.06, 0.925, summary, color=INK_SECONDARY, fontsize=9, ha='left', va='top',
             linespacing=1.5)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    path = os.path.join(out_dir, f'compare_{program}_tcp.png')
    fig.savefig(path, dpi=130, facecolor='#fcfcfb')
    return path


def ratio_text(ratio):
    """Ratio to the limit: percent up to 10x, factor above."""
    return f'{ratio * 100:.0f} % of limit' if ratio < 10 else f'{ratio:.0f}x the limit'


def jerk_stats(jerk, names, limits):
    """Per joint: crossings of the jerk limit, time above it, peak |jerk| / limit."""
    rows = []
    for i, name in enumerate(names):
        lim = limits[name]['max_jerk']
        above = np.abs(jerk[:, i]) > lim
        crossings = int(np.count_nonzero(above[1:] & ~above[:-1]) + above[0])
        peak = float(np.max(np.abs(jerk[:, i])))
        rows.append((name, crossings, above.sum() * DT, peak, peak / lim))
    return rows


def print_stats(program, label, rows, duration):
    total = sum(r[1] for r in rows)
    worst = max(rows, key=lambda r: r[4])
    print(f'  {label:<12} duration {duration:6.2f} s   crossings {total:4d}   '
          f'worst {worst[0]}: peak {worst[3]:9.1f} rad/s^3 = {ratio_text(worst[4])}')
    for name, crossings, t_above, peak, ratio in rows:
        print(f'      {name}: {crossings:4d} crossings, {t_above:6.3f} s above, '
              f'peak {peak:9.1f} rad/s^3 ({ratio_text(ratio)})')
    return total, worst


def style(ax):
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_SECONDARY)
    ax.tick_params(colors=INK_SECONDARY, labelsize=9)


def plot(program, raw, smooth, names, joint, limits, summary, out_dir):
    j = names.index(joint)
    lim = limits[joint]
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    panels = [('position [rad]', 1, None), ('velocity [rad/s]', 2, lim['max_velocity']),
              ('acceleration [rad/s²]', 3, lim['max_acceleration']),
              ('jerk [rad/s³]', 4, lim['max_jerk'])]
    for ax, (label, idx, limit) in zip(axes, panels):
        style(ax)
        ax.plot(raw[0], raw[idx][:, j], color=RAW_COLOR, linewidth=1.2, label='Pilz raw')
        ax.plot(smooth[0], smooth[idx][:, j], color=SMOOTH_COLOR, linewidth=2,
                label='with filter')
        if limit is not None:
            for sign in (1, -1):
                ax.axhline(sign * limit, color=INK_SECONDARY, linewidth=1, linestyle='--')
        ax.set_ylabel(label, color=INK, fontsize=10)
    jerk_ax = axes[3]
    # raw jerk spikes are orders of magnitude above the limit: linear inside +-limit, log outside
    jerk_ax.set_yscale('symlog', linthresh=lim['max_jerk'])
    jerk_ax.text(0.995, 0.97, f'dashed: limit ±{lim["max_jerk"]:g} (log scale outside)',
                 transform=jerk_ax.transAxes, ha='right', va='top', fontsize=8,
                 color=INK_SECONDARY)
    axes[-1].set_xlabel('time [s]', color=INK, fontsize=10)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right', bbox_to_anchor=(0.985, 0.975), frameon=False,
               fontsize=10, ncol=2)
    fig.suptitle(f'{program}: {joint}', color=INK, fontsize=13, x=0.06, ha='left')
    fig.text(0.06, 0.94, summary, color=INK_SECONDARY, fontsize=9, ha='left', va='top',
             linespacing=1.5)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    path = os.path.join(out_dir, f'compare_{program}.png')
    fig.savefig(path, dpi=130, facecolor='#fcfcfb')
    return path


def start_move_group(tool):
    """Start move_group + trajectory_smoother in a private, local ROS domain; returns the process.

    Sets the same domain for this process, so call it before rclpy.init().
    """
    domain = str(random.randint(150, 200))
    os.environ['ROS_DOMAIN_ID'] = domain
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    log = tempfile.NamedTemporaryFile(prefix='compare_smoothing_move_group_', suffix='.log',
                                      delete=False)
    print(f'starting move_group (domain {domain}, tool {tool}), log: {log.name}')
    process = subprocess.Popen(
        ['ros2', 'launch', 'robotarm_moveit_config', 'move_group.launch.py', 'rviz:=false',
         'allow_trajectory_execution:=false', f'tool:={tool}'],
        stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
    process.log_path = log.name
    return process


def stop_move_group(process):
    """Stop the launch started by start_move_group (whole process group)."""
    if process is None or process.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, sig)
            process.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue
        except ProcessLookupError:
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('programs', nargs='*', help='programs (default: all of the goals file)')
    parser.add_argument('--start', default='ready', help='named start state (default: ready)')
    parser.add_argument('--scaling', type=float, default=0.2,
                        help='scaling of steps without their own (default 0.2)')
    parser.add_argument('--joint', default=None,
                        help='joint to plot (default: highest raw jerk relative to its limit)')
    parser.add_argument('--out', default='.', help='directory for the figures (default: .)')
    parser.add_argument('--show', action='store_true', help='also open the figures')
    parser.add_argument('--goals', default=default_goals_file(),
                        help="goals file (default: the motion_server's)")
    parser.add_argument('--tool', default='axis_triad',
                        help='tool on the flange for the own move_group (default: axis_triad)')
    parser.add_argument('--no-tcp', action='store_true',
                        help='skip the TCP comparison (FK of every sample)')
    parser.add_argument('--use-running', action='store_true',
                        help='use the running move_group of the current ROS domain instead of '
                             'starting one')
    args = parser.parse_args()

    move_group = None if args.use_running else start_move_group(args.tool)
    try:
        run(args, move_group)
    finally:
        stop_move_group(move_group)
    sys.stdout.flush()
    os._exit(0)  # skip the slow rclpy shutdown of the spinning executor


def run(args, move_group):
    rclpy.init()
    node = rclpy.create_node('compare_smoothing')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    moveit = MoveItClient(node)
    planner = Planner(node, moveit)
    try:
        moveit.wait_for_servers(execute=False, timeout=60.0)
        planner.wait_for_services(timeout=30.0)
    except Exception:
        if move_group is not None:
            print(f'move_group did not come up, see {move_group.log_path}')
        else:
            print('no move_group in this ROS domain: start one, or leave out --use-running')
        raise

    # plan without the filter, keep the raw Pilz trajectories, filter them afterwards
    smooth_service = planner.smooth
    planner.smooth = lambda trajectory: trajectory
    goal_file = GoalFile(args.goals, load_named_states())
    programs = args.programs or list(goal_file.programs)
    start = load_named_states()[args.start]
    limits = joint_limits()
    os.makedirs(args.out, exist_ok=True)

    for program in programs:
        segs, notes = segments(goal_file.program(program))
        print(f'\n{program} (start "{args.start}", scaling {args.scaling})')
        for note in notes:
            print(f'  note: {note}')
        planned = planner.plan(goal_file, segs, args.scaling, start)
        raw_parts, smooth_parts, offset_raw, offset_smooth = [], [], 0.0, 0.0
        stops = []
        names = list(planned[0].trajectory.joint_names)
        raw_tcp, smooth_tcp, piece_list = [], [], []
        for seg in planned:
            raw_traj = seg.trajectory
            # OMPL segments are already smoothed by move_group (Ruckig): the same in both
            smooth_traj = smooth_service(raw_traj) if seg.planner == 'pilz' else raw_traj
            stops += stop_errors(raw_traj, smooth_traj)
            raw_parts.append(evaluate(raw_traj, offset_raw))
            smooth_parts.append(evaluate(smooth_traj, offset_smooth))
            if not args.no_tcp:
                raw_tcp.append(tcp_samples(moveit, names, raw_traj, raw_parts[-1], offset_raw))
                smooth_tcp.append(tcp_samples(moveit, names, smooth_traj, smooth_parts[-1],
                                              offset_smooth))
                piece_list += pieces(raw_traj, smooth_traj, offset_raw, offset_smooth)
            offset_raw += duration_s(raw_traj.points[-1].time_from_start)
            offset_smooth += duration_s(smooth_traj.points[-1].time_from_start)
        raw, smooth = join(raw_parts), join(smooth_parts)

        raw_rows = jerk_stats(raw[4], names, limits)
        smooth_rows = jerk_stats(smooth[4], names, limits)
        raw_total, raw_worst = print_stats(program, 'Pilz raw', raw_rows, offset_raw)
        smooth_total, smooth_worst = print_stats(program, 'with filter', smooth_rows,
                                                 offset_smooth)
        stop_text = 'stops: none inside segments' if not stops else (
            f'stops: {len(stops)}, max joint error filtered vs. Pilz {max(stops):.1e} rad')
        print(f'  {stop_text}')
        joint = args.joint or raw_worst[0]
        summary = (f'jerk limit crossings (all joints): Pilz raw {raw_total}, peak '
                   f'{ratio_text(raw_worst[4])} ({raw_worst[0]})  ·  with filter '
                   f'{smooth_total}, peak {ratio_text(smooth_worst[4])} ({smooth_worst[0]})  ·  '
                   f'duration {offset_raw:.2f} s → {offset_smooth:.2f} s\n{stop_text}')
        path = plot(program, raw, smooth, names, joint, limits, summary, args.out)
        print(f'  figure: {os.path.abspath(path)}')
        if not args.no_tcp:
            raw_all = tuple(np.concatenate([p[i] for p in raw_tcp]) for i in range(3))
            smooth_all = tuple(np.concatenate([p[i] for p in smooth_tcp]) for i in range(3))
            dist, angle = path_deviation(raw_all, smooth_all, piece_list)
            # where a piece ends and the next one starts (stops and segment boundaries)
            stop_times = [piece_list[i][3] for i in range(len(piece_list) - 1)]
            at_stops = [dist[int(np.argmin(np.abs(smooth_all[0] - ts)))] for ts in stop_times]
            tcp_text = (f'TCP deviation from the Pilz path: max {np.nanmax(dist) * 1000:.2f} mm, '
                        f'mean {np.nanmean(dist) * 1000:.2f} mm  ·  orientation max '
                        f'{np.degrees(np.nanmax(angle)):.2f}°')
            if at_stops:
                tcp_text += f'\nat the {len(at_stops)} stops: max {max(at_stops) * 1000:.3f} mm'
            print('  ' + tcp_text.replace('\n', '\n  '))
            path = plot_tcp(program, smooth_all[0], dist, angle, stop_times, tcp_text, args.out)
            print(f'  figure: {os.path.abspath(path)}')

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
