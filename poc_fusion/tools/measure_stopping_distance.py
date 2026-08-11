#!/usr/bin/env python3
"""Measure the robot's stopping distance at the operating speed.

WHY THIS EXISTS
---------------
`window_forward_m` cannot be set without this number. The feasible band is

    front_extent + d + half_cell  <  window  <  safety + lidar_off - d - half_cell

and `d` (stopping distance) enters with OPPOSITE SIGNS, closing the band from
both directions. With the measured geometry (front extent 0.1328 m, LiDAR
offset 0.0730 m, cell 0.05 m) the band is empty once d >= 0.0451 m. Task 9
Part 2 gives 0.009 m but at 0.05 m/s; a 4x speed extrapolation is not evidence.

WHAT IT MEASURES, AND WHY NOT THE OBVIOUS WAY
---------------------------------------------
Naively: record the wall range when zero is commanded, record it again once
stopped, subtract. That is WRONG here. The newest scan at the moment of the
stop command is up to one scan period old, and at 0.20 m/s a 0.1 s scan age is
0.02 m -- the same order as the quantity being measured. The bias would be
comparable to the result.

So instead the approach segment is fit as a straight line r(t) = r0 - v*t over
the second preceding the stop command, using each scan's OWN header stamp, and
that fit is evaluated AT the stop-command time. Scan age cancels. The fitted
`v` is a free cross-check: it must come out near 0.20 m/s, otherwise the robot
was not at steady speed and the run is void.

    stopping_distance = r_fit(t_cmd) - median(r during the settle window)

SAFETY
------
  * Publishes to /controller/cmd_vel, the vendor input topic five stock nodes
    already use. NOT the final /cmd_vel. No gate is bypassed -- the poc_fusion
    gate is not in this path at all and must not be running.
  * The STM32 latches the last commanded velocity forever. Every exit path --
    normal, abort, exception, SIGINT -- goes through _hard_stop(), which
    publishes a zero Twist repeatedly, not once.
  * Aborts on: wall closer than ABORT_RANGE_M, scan older than
    SCAN_STALE_S, or approach longer than MAX_APPROACH_S.
  * Refuses to move without --arm. Without it the script only measures and
    reports, so the operator can verify the LiDAR's forward direction against
    a tape measure before anything moves.

The wall is never reachable: zero is commanded at 0.80 m of LiDAR range, the
gripper sits 0.06 m ahead of the LiDAR, and the expected stopping distance is
under 0.05 m. Abort fires at 0.45 m, which is ~0.39 m of gripper clearance.
"""
import argparse
import atexit
import csv
import math
import os
import signal
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from poc_fusion.lib.stopping_distance import (  # noqa: E402
    analyse_run,
    speed_is_valid,
)

# --- Fixed test parameters ------------------------------------------------
# Not tunable from the command line on purpose: changing them silently would
# change what the recorded number means.

CMD_TOPIC = '/controller/cmd_vel'
SCAN_TOPIC = '/scan_raw'
ODOM_TOPIC = '/odom'

TEST_SPEED_MPS = 0.20        # the operating speed; the whole point of the run
CONTROL_HZ = 20.0

TRIGGER_RANGE_M = 0.80       # LiDAR range at which zero is commanded
ABORT_RANGE_M = 0.45         # any reading below this aborts immediately
MIN_START_RANGE_M = 1.40     # need room to reach steady speed before trigger
MAX_START_RANGE_M = 3.00     # beyond this the beam spread makes the fit noisy

MAX_APPROACH_S = 20.0        # approach longer than this is a fault, not a run
SCAN_STALE_S = 0.5           # no fresh scan for this long -> abort
FIT_WINDOW_S = 1.0           # approach segment used for the linear fit
HOLD_ZERO_S = 2.0            # keep publishing zeros after the stop command
SETTLE_SKIP_S = 1.0          # ignore this much after t_cmd, then measure
SETTLE_WINDOW_S = 1.5        # median of front range over this window

FRONT_HALF_ANGLE_RAD = math.radians(5.0)
MIN_FRONT_BEAMS = 3

PREFLIGHT_S = 3.0


class StoppingDistanceRun(Node):

    def __init__(self, armed):
        super().__init__('measure_stopping_distance')
        self.armed = armed
        self._stopped = False

        self.samples = []        # (stamp_s, front_range_m)
        self.odom = []           # (stamp_s, x, y, vx)
        self.last_scan_wall = None
        self.last_front = None
        self.scan_count = 0

        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 1)
        self.create_subscription(
            LaserScan, SCAN_TOPIC, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, qos_profile_sensor_data)

    # --- Sensing ----------------------------------------------------------

    def _on_scan(self, msg):
        """Median range over a +/-5 deg cone straight ahead.

        Median, not min: min would latch onto a single spurious short return
        and both trigger and abort early. Median over a narrow cone tracks a
        flat wall robustly. This is deliberately NOT the avoidance
        controller's 180 deg minimum -- that statistic is designed to be
        conservative about anything anywhere in front, which is the wrong
        instrument for measuring distance to one specific wall.
        """
        vals = []
        for i, r in enumerate(msg.ranges):
            ang = msg.angle_min + i * msg.angle_increment
            # Wrap into (-pi, pi] so a 0..2pi scan still finds forward.
            ang = math.atan2(math.sin(ang), math.cos(ang))
            if abs(ang) > FRONT_HALF_ANGLE_RAD:
                continue
            if not math.isfinite(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            vals.append(r)

        self.scan_count += 1
        if len(vals) < MIN_FRONT_BEAMS:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        front = statistics.median(vals)
        self.samples.append((stamp, front))
        self.last_front = front
        self.last_scan_wall = time.monotonic()

    def _on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.odom.append((stamp,
                          msg.pose.pose.position.x,
                          msg.pose.pose.position.y,
                          msg.twist.twist.linear.x))

    # --- Actuation --------------------------------------------------------

    def _publish(self, vx):
        t = Twist()
        t.linear.x = vx
        self.cmd_pub.publish(t)

    def hard_stop(self):
        """Idempotent, safe to call from anywhere, including atexit."""
        if self._stopped:
            return
        self._stopped = True
        for _ in range(30):
            self._publish(0.0)
            time.sleep(0.02)

    # --- Phases -----------------------------------------------------------

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def preflight(self):
        """No motion. Establishes that the sensors are live and that the
        forward cone actually points at the wall the operator measured."""
        print(f'Preflight: listening {PREFLIGHT_S:.0f} s (no motion)...')
        self.spin_for(PREFLIGHT_S)

        if not self.samples:
            return False, (f'no usable {SCAN_TOPIC} readings in the forward '
                           f'cone ({self.scan_count} scans seen)')
        if not self.odom:
            return False, f'no {ODOM_TOPIC} messages'

        rng = [r for _, r in self.samples]
        rate = len(self.samples) / PREFLIGHT_S
        med = statistics.median(rng)
        spread = max(rng) - min(rng)

        print(f'  forward range : {med:.4f} m  '
              f'(min {min(rng):.4f}, max {max(rng):.4f}, spread {spread:.4f})')
        print(f'  scan rate     : {rate:.1f} Hz over the forward cone')
        print(f'  odom messages : {len(self.odom)}')

        if med < MIN_START_RANGE_M:
            return False, (f'start range {med:.3f} m is below the required '
                           f'{MIN_START_RANGE_M} m -- the robot would not '
                           f'reach steady speed before the trigger')
        if med > MAX_START_RANGE_M:
            return False, (f'start range {med:.3f} m exceeds '
                           f'{MAX_START_RANGE_M} m -- move closer to the wall')
        if spread > 0.05:
            return False, (f'forward range is not stable while stationary '
                           f'(spread {spread:.3f} m) -- something is moving '
                           f'in the cone, or the wall is not flat/normal')
        return True, med

    def approach_and_stop(self):
        """Drive at TEST_SPEED_MPS until the trigger range, then command zero.

        Returns (t_cmd_wall, t_cmd_stamp_basis) or raises RuntimeError.
        """
        period = 1.0 / CONTROL_HZ
        t_start = time.monotonic()
        print(f'Driving at {TEST_SPEED_MPS} m/s; zero at '
              f'{TRIGGER_RANGE_M} m; abort at {ABORT_RANGE_M} m.')

        while True:
            rclpy.spin_once(self, timeout_sec=0.005)
            now = time.monotonic()

            if self.last_scan_wall is None or \
                    now - self.last_scan_wall > SCAN_STALE_S:
                raise RuntimeError('scan went stale during the approach')
            if now - t_start > MAX_APPROACH_S:
                raise RuntimeError(
                    f'approach exceeded {MAX_APPROACH_S} s without reaching '
                    f'the trigger range')
            if self.last_front is not None and self.last_front < ABORT_RANGE_M:
                raise RuntimeError(
                    f'ABORT: front range {self.last_front:.3f} m is inside '
                    f'the {ABORT_RANGE_M} m abort threshold')

            if self.last_front is not None and \
                    self.last_front <= TRIGGER_RANGE_M:
                break

            self._publish(TEST_SPEED_MPS)
            time.sleep(period)

        # The stop command instant. Everything downstream is referenced here.
        self._publish(0.0)
        t_cmd_wall = time.monotonic()
        t_cmd_stamp = self.samples[-1][0] + (t_cmd_wall - self.last_scan_wall)
        print(f'  zero commanded at front range {self.last_front:.4f} m')

        end = t_cmd_wall + HOLD_ZERO_S
        while time.monotonic() < end:
            self._publish(0.0)
            rclpy.spin_once(self, timeout_sec=0.005)
            time.sleep(period)
        self._stopped = True

        # Keep listening through the settle window.
        self.spin_for(SETTLE_SKIP_S + SETTLE_WINDOW_S)
        return t_cmd_stamp


SPEED_TOLERANCE_MPS = 0.03   # fitted vs commanded; beyond this the run is void


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--arm', action='store_true',
                    help='actually move the robot. Without this the script '
                         'only runs preflight and exits.')
    ap.add_argument('--out', default=None, help='CSV path for the raw trace')
    args = ap.parse_args()

    rclpy.init()
    node = StoppingDistanceRun(armed=args.arm)
    atexit.register(node.hard_stop)
    signal.signal(signal.SIGINT, lambda *_: (node.hard_stop(), sys.exit(130)))
    signal.signal(signal.SIGTERM, lambda *_: (node.hard_stop(), sys.exit(143)))

    exit_code = 0
    try:
        ok, info = node.preflight()
        if not ok:
            print(f'PREFLIGHT FAILED: {info}')
            return 2
        print(f'Preflight OK. Start range {info:.4f} m.')

        if not args.arm:
            print('\nNot armed -- no motion commanded. Compare the forward '
                  'range above against your tape measure. If they agree, '
                  're-run with --arm.')
            return 0

        t_cmd_stamp = node.approach_and_stop()
        try:
            result = analyse_run(node.samples, t_cmd_stamp,
                                 fit_window_s=FIT_WINDOW_S,
                                 settle_skip_s=SETTLE_SKIP_S,
                                 settle_window_s=SETTLE_WINDOW_S)
        except ValueError as exc:
            print(f'INCONCLUSIVE: {exc}')
            return 3

        print('\n--- RESULT ---')
        print(f'  fitted approach speed : '
              f'{result["approach_speed_mps"]:.4f} m/s '
              f'(commanded {TEST_SPEED_MPS})')
        print(f'  fit points / resid sd : {result["fit_points"]} / '
              f'{result["fit_residual_sd_m"]*1000:.1f} mm')
        print(f'  range at stop command : '
              f'{result["range_at_cmd_m"]:.4f} m (fit-corrected)')
        print(f'  settled range         : '
              f'{result["settled_range_m"]:.4f} m '
              f'(sd {result["settle_sd_m"]*1000:.1f} mm, '
              f'n={result["settle_points"]})')
        print(f'  STOPPING DISTANCE     : '
              f'{result["stopping_distance_m"]*1000:.1f} mm')

        if not speed_is_valid(result['approach_speed_mps'], TEST_SPEED_MPS,
                              SPEED_TOLERANCE_MPS):
            print(f'  ** VOID: fitted speed differs from the commanded '
                  f'{TEST_SPEED_MPS} m/s by more than '
                  f'{SPEED_TOLERANCE_MPS} m/s. The robot was not at steady '
                  f'speed; this is not a stopping distance at {TEST_SPEED_MPS} '
                  f'm/s. Discard the run. **')
            exit_code = 3
        else:
            under = result['stopping_distance_m'] < 0.0451
            print(f'  feasibility limit     : 45.1 mm '
                  f'({"UNDER -- option (a) is feasible" if under else "OVER -- option (a) is NOT feasible"})')
    except RuntimeError as exc:
        print(f'RUN ABORTED: {exc}')
        exit_code = 4
    finally:
        node.hard_stop()
        if args.out:
            with open(args.out, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['stamp_s', 'channel', 'value'])
                for t, r in node.samples:
                    w.writerow([f'{t:.6f}', 'front_range_m', f'{r:.5f}'])
                for t, x, y, vx in node.odom:
                    w.writerow([f'{t:.6f}', 'odom_x_m', f'{x:.5f}'])
                    w.writerow([f'{t:.6f}', 'odom_vx_mps', f'{vx:.5f}'])
            print(f'Raw trace written to {args.out} '
                  f'({len(node.samples)} scan rows)')
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
