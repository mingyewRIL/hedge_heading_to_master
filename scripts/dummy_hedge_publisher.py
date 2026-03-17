#!/usr/bin/env python3
"""
Publish fake hedge_pos_ang messages for a dummy drone (for testing with 1 real drone).
Publishes on the same /hedge_pos_ang topic so the heading/distances node sees real + dummy.

Change position and angle via params (in launch or rosparam set); params are re-read each cycle.
"""

import rospy
from marvelmind_nav.msg import hedge_pos_ang


def main():
    rospy.init_node("dummy_hedge_publisher")

    topic = rospy.get_param("~topic", "/hedge_pos_ang")
    rate_hz = rospy.get_param("~rate", 10.0)

    pub = rospy.Publisher(topic, hedge_pos_ang, queue_size=2)
    rate = rospy.Rate(rate_hz)

    rospy.loginfo("Dummy hedge publisher: publishing to %s (address, x, y, z, angle via params)", topic)

    while not rospy.is_shutdown():
        try:
            address = rospy.get_param("~address", 99)
            x_m = rospy.get_param("~x_m", 2.0)
            y_m = rospy.get_param("~y_m", 0.0)
            z_m = rospy.get_param("~z_m", 0.0)
            angle_deg = rospy.get_param("~angle_deg", 45.0)
        except (KeyError, TypeError):
            address, x_m, y_m, z_m, angle_deg = 99, 2.0, 0.0, 0.0, 45.0

        msg = hedge_pos_ang()
        msg.address = address
        msg.timestamp_ms = int(rospy.Time.now().to_nsec() // 1_000_000)
        msg.x_m = x_m
        msg.y_m = y_m
        msg.z_m = z_m
        msg.flags = 0
        msg.angle = angle_deg
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
