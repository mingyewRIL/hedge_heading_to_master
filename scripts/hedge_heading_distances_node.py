#!/usr/bin/env python3
"""
Subscribe to /hedge_pos_ang (from marvelmind_nav hedge_rcv_bin).
Publish each drone's heading (yaw) and distance to every other drone.

When hedge_pairs is set (e.g. "22:21,22 12:11,12"), each logical drone uses the
midpoint of its paired beacons for position and the joint heading from the
primary beacon (first in the pair, e.g. 22 and 12).
"""

import math
import threading
import rospy
from std_msgs.msg import Header
from marvelmind_nav.msg import hedge_pos_ang
from hedge_heading_to_master.msg import (
    DroneHeadingDistances,
    DroneHeadingDistancesDebug,
    FleetHeadingDistances,
    FleetHeadingDistancesDebug,
)


def distance_xy(a, b):
    """2D distance in the x–y plane (ignore z)."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def normalize_angle_deg(a):
    """Wrap angle to [-180, 180] so we don't turn 300° when 60° is shorter."""
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a


DECIMAL_PLACES = 3


class ScalarKalman1D:
    """Simple 1D Kalman filter; for angles, residual/output are wrapped to [-180, 180]."""

    def __init__(self, q, r, is_angle=False):
        self.q = float(q)
        self.r = float(r)
        self.is_angle = is_angle
        self.x = None
        self.p = 1.0

    def update(self, z):
        z = float(z)
        if self.x is None:
            self.x = z
            return self.x

        # Predict
        self.p = self.p + self.q

        # Innovation / residual
        residual = z - self.x
        if self.is_angle:
            residual = normalize_angle_deg(residual)

        # Update
        k = self.p / (self.p + self.r)
        self.x = self.x + k * residual
        self.p = (1.0 - k) * self.p

        if self.is_angle:
            self.x = normalize_angle_deg(self.x)
        return self.x


def parse_hedge_pairs(s):
    """Parse '22:21,22 12:11,12' -> {22: [21, 22], 12: [11, 12]}. Logical id -> list of beacon addresses (primary first)."""
    out = {}
    for part in (s or "").strip().split():
        part = part.strip()
        if ":" not in part:
            continue
        logical_str, addrs_str = part.split(":", 1)
        logical_id = int(logical_str.strip())
        addrs = [int(x.strip()) for x in addrs_str.split(",") if x.strip()]
        if addrs:
            out[logical_id] = addrs
    return out


def main():
    rospy.init_node("hedge_heading_distances_node")

    hedge_topic = rospy.get_param("~hedge_pos_ang_topic", "/hedge_pos_ang")
    out_topic = rospy.get_param("~fleet_heading_distances_topic", "fleet_heading_distances")
    debug_out_topic = rospy.get_param("~fleet_heading_distances_debug_topic", "fleet_heading_distances_debug")
    publish_rate = rospy.get_param("~publish_rate", 10.0)  # Hz
    frame_id = rospy.get_param("~frame_id", "marvelmind")
    dummy_address = rospy.get_param("~dummy_address", 99)
    enable_kalman = rospy.get_param("~enable_kalman", True)
    kf_q_distance = rospy.get_param("~kf_q_distance", 0.01)
    kf_r_distance = rospy.get_param("~kf_r_distance", 0.6)
    kf_q_angle = rospy.get_param("~kf_q_angle", 0.3)
    kf_r_angle = rospy.get_param("~kf_r_angle", 8.0)

    # hedge_pairs: e.g. "22:21,22 12:11,12" => position = midpoint of pair, heading from primary (22, 12)
    hedge_pairs_param = rospy.get_param("~hedge_pairs", "")
    pairs = parse_hedge_pairs(hedge_pairs_param)

    if pairs:
        # All addresses that appear in any pair, plus dummy
        allowed_addresses = set()
        for addrs in pairs.values():
            allowed_addresses.update(addrs)
        allowed_addresses.add(dummy_address)
        logical_ids = sorted(pairs.keys())
    else:
        # Legacy: hedge_addresses only (one position per address)
        pairs = None
        logical_ids = None
        hedge_addresses = rospy.get_param("~hedge_addresses", [])
        if isinstance(hedge_addresses, str):
            hedge_addresses = [int(x.strip()) for x in hedge_addresses.split(",") if x.strip()]
        primary_hedge_address = rospy.get_param("~primary_hedge_address", -1)
        if not hedge_addresses and primary_hedge_address >= 0:
            hedge_addresses = [primary_hedge_address]
        allowed_addresses = set(hedge_addresses) | {dummy_address} if hedge_addresses else None

    # Cache: address -> (x_m, y_m, z_m, angle_deg)
    state = {}
    lock = threading.Lock()
    # Per directed pair filter state: (src_addr, dst_addr, metric)
    # metric in {"dist", "head_err", "other_head_rel"}.
    kalman_filters = {}

    def apply_filter(src_addr, dst_addr, metric, value, is_angle=False):
        if not enable_kalman:
            return value
        key = (int(src_addr), int(dst_addr), metric)
        if key not in kalman_filters:
            if is_angle:
                kalman_filters[key] = ScalarKalman1D(kf_q_angle, kf_r_angle, is_angle=True)
            else:
                kalman_filters[key] = ScalarKalman1D(kf_q_distance, kf_r_distance, is_angle=False)
        return kalman_filters[key].update(value)

    def cb(msg):
        if allowed_addresses is not None and msg.address not in allowed_addresses:
            return
        with lock:
            state[msg.address] = (msg.x_m, msg.y_m, msg.z_m, msg.angle)

    rospy.Subscriber(hedge_topic, hedge_pos_ang, cb)
    pub = rospy.Publisher(out_topic, FleetHeadingDistances, queue_size=10)
    debug_pub = rospy.Publisher(debug_out_topic, FleetHeadingDistancesDebug, queue_size=10)

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        with lock:
            snapshot = dict(state)
        if not snapshot:
            rate.sleep()
            continue

        if pairs:
            # Build logical drone state: midpoint position, heading from primary
            logical_state = {}
            for logical_id, addrs in pairs.items():
                positions = [snapshot[a] for a in addrs if a in snapshot]
                if not positions:
                    continue
                xs = [p[0] for p in positions]
                ys = [p[1] for p in positions]
                zs = [p[2] for p in positions]
                x = sum(xs) / len(xs)
                y = sum(ys) / len(ys)
                z = sum(zs) / len(zs)
                # Heading from primary (first in pair) if available, else first we have
                angle_deg = None
                for a in addrs:
                    if a in snapshot:
                        angle_deg = snapshot[a][3]
                        break
                if angle_deg is None:
                    continue
                logical_state[logical_id] = (x, y, z, angle_deg)
            addresses = sorted(logical_state.keys())
            state_for_fleet = logical_state
        else:
            addresses = sorted(snapshot.keys())
            state_for_fleet = snapshot

        fleet = FleetHeadingDistances()
        fleet.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        fleet_debug = FleetHeadingDistancesDebug()
        fleet_debug.header = Header(stamp=fleet.header.stamp, frame_id=frame_id)

        for addr in addresses:
            x, y, z, angle_deg = state_for_fleet[addr]
            others = [a for a in addresses if a != addr]
            dists = []
            dists_raw = []
            heading_errors_deg = []
            heading_errors_deg_raw = []
            other_headings_wrt_drone_deg = []
            other_headings_wrt_drone_deg_raw = []
            for other in others:
                ox, oy, oz, other_angle_deg = state_for_fleet[other]
                dx = ox - x
                dy = oy - y
                dist_val_raw = distance_xy((x, y), (ox, oy))
                dists_raw.append(round(dist_val_raw, DECIMAL_PLACES))
                dist_val = dist_val_raw
                dist_val = apply_filter(addr, other, "dist", dist_val, is_angle=False)
                dists.append(round(dist_val, DECIMAL_PLACES))
                global_angle_deg = math.degrees(math.atan2(dy, dx))
                heading_error_deg_raw = normalize_angle_deg(global_angle_deg - angle_deg)
                heading_errors_deg_raw.append(round(heading_error_deg_raw, DECIMAL_PLACES))
                heading_error_deg = heading_error_deg_raw
                heading_error_deg = apply_filter(addr, other, "head_err", heading_error_deg, is_angle=True)
                heading_errors_deg.append(round(heading_error_deg, DECIMAL_PLACES))
                # "other heading with respect to this drone":
                # heading difference between other drone and this drone.
                rel_other_heading_deg_raw = normalize_angle_deg(other_angle_deg - angle_deg)
                other_headings_wrt_drone_deg_raw.append(round(rel_other_heading_deg_raw, DECIMAL_PLACES))
                rel_other_heading_deg = rel_other_heading_deg_raw
                rel_other_heading_deg = apply_filter(addr, other, "other_head_rel", rel_other_heading_deg, is_angle=True)
                other_headings_wrt_drone_deg.append(round(rel_other_heading_deg, DECIMAL_PLACES))

            dhd = DroneHeadingDistances()
            dhd.address = addr
            dhd.other_addresses = others
            dhd.distances_to_others = dists
            dhd.heading_error_to_others_deg = heading_errors_deg
            dhd.other_heading_with_respect_to_drone_deg = other_headings_wrt_drone_deg
            fleet.drones.append(dhd)

            dhd_debug = DroneHeadingDistancesDebug()
            dhd_debug.address = addr
            dhd_debug.other_addresses = others
            dhd_debug.distances_to_others = dists
            dhd_debug.distances_to_others_raw = dists_raw
            dhd_debug.heading_error_to_others_deg = heading_errors_deg
            dhd_debug.heading_error_to_others_deg_raw = heading_errors_deg_raw
            dhd_debug.other_heading_with_respect_to_drone_deg = other_headings_wrt_drone_deg
            dhd_debug.other_heading_with_respect_to_drone_deg_raw = other_headings_wrt_drone_deg_raw
            fleet_debug.drones.append(dhd_debug)

        pub.publish(fleet)
        debug_pub.publish(fleet_debug)
        rate.sleep()


if __name__ == "__main__":
    main()
