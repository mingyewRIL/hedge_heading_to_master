#!/usr/bin/env python3
"""
Prepare Marvelmind environment via the Dashboard API: set beacon positions and optionally
freeze the map. Use instead of the Marvelmind Dashboard for scripted setup.

Requirements:
  - Marvelmind API library: libdashapi.so (Linux) from Marvelmind. Place it where
    the marvelmind_api_ros node can load it (e.g. LD_LIBRARY_PATH or same directory).
  - Run the API node first (it opens the modem port):
      rosrun marvelmind_nav marvelmind_api_ros /dev/serial/by-id/usb-Marvelmind_...
  - Then run this script with your beacon YAML:
      rosrun hedge_heading_to_master setup_marvelmind_env.py _beacon_config:=path/to/beacon_positions.yaml

After setup, stop the API node and use hedge_rcv_bin for normal streaming.
"""

from __future__ import print_function

import struct
import os          
import yaml        
import sys         
import rospy
from marvelmind_nav.srv import MarvelmindAPI, MarvelmindAPIRequest, MarvelmindAPIResponse

# Marvelmind API command IDs (from marvelmind_nav marvelmind_api.h)
MM_API_ID_ERASE_MAP = 30
MM_API_ID_SET_BEACON_LOCATION = 40
MM_API_ID_FREEZE_MAP = 32
MM_API_ID_UNFREEZE_MAP = 33


def call_api(command_id, request_bytes=None):
    if request_bytes is None:
        request_bytes = []
    rospy.wait_for_service("marvelmind_api", timeout=10.0)
    srv = rospy.ServiceProxy("marvelmind_api", MarvelmindAPI)
    req = MarvelmindAPIRequest()
    req.command_id = command_id
    req.request = list(request_bytes)
    try:
        res = srv(req)
        return res.success, res.error_code, list(res.response) if res.response else []
    except Exception as e:
        rospy.logerr("marvelmind_api call failed: %s", e)
        return False, -1, []


def set_beacon_location(address, x_mm, y_mm, z_mm):
    """Request bytes: address (1), x_mm (4 LE), y_mm (4 LE), z_mm (4 LE)."""
    req = [address] + list(struct.pack("<iii", int(x_mm), int(y_mm), int(z_mm)))
    return call_api(MM_API_ID_SET_BEACON_LOCATION, req)


def main():
    rospy.init_node("setup_marvelmind_env", anonymous=False)

    config_path = rospy.get_param("~beacon_config", "")
    if not config_path:
        try:
            import rospkg
            rp = rospkg.RosPack()
            pkg_path = rp.get_path("hedge_heading_to_master")
            config_path = os.path.join(pkg_path, "config", "beacon_positions_example.yaml")
        except Exception:
            config_path = ""
    if not config_path or not os.path.isfile(config_path):
        rospy.logerr("Beacon config YAML required. Example: _beacon_config:=$(rospack find hedge_heading_to_master)/config/beacon_positions_example.yaml")
        return 1

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    # Erase map (optional; comment out if you only want to add/update beacons)
    rospy.loginfo("Erasing map...")
    ok, err, _ = call_api(MM_API_ID_ERASE_MAP)
    if not ok:
        rospy.logwarn("Erase map returned success=%s error_code=%s", ok, err)
    else:
        rospy.loginfo("Map erased.")

    beacons = []
    if cfg.get("stationary_beacons"):
        beacons.extend(cfg["stationary_beacons"])
    if cfg.get("hedgehog_beacons"):
        beacons.extend(cfg["hedgehog_beacons"])

    for b in beacons:
        addr = b["address"]
        x, y, z = b["x_mm"], b["y_mm"], b["z_mm"]
        rospy.loginfo("Setting beacon %s to (%s, %s, %s) mm", addr, x, y, z)
        ok, err, _ = set_beacon_location(addr, x, y, z)
        if not ok:
            rospy.logwarn("Set beacon %s failed: error_code=%s", addr, err)

    # Freeze map so positions are used
    rospy.loginfo("Freezing map...")
    ok, err, _ = call_api(MM_API_ID_FREEZE_MAP)
    if not ok:
        rospy.logwarn("Freeze map returned success=%s error_code=%s", ok, err)
    else:
        rospy.loginfo("Map frozen. You can stop marvelmind_api_ros and run hedge_rcv_bin for normal use.")

    return 0


if __name__ == "__main__":
    try:
        exit(main() or 0)
    except rospy.ROSInterruptException:
        pass
