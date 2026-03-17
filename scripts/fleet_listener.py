#!/usr/bin/env python3

import rospy
# Import your custom message! 
# (Assuming your package is still named 'hedge_heading_to_master')
from hedge_heading_to_master.msg import FleetHeadingDistances

def fleet_callback(msg):
    """
    This function runs every time a new message arrives on the topic.
    """
    # Print the timestamp just to show we received a new packet
    rospy.loginfo(f"--- New Fleet Data Received (Seq: {msg.header.seq}) ---")

    # 1. Loop through every drone reporting its status
    for drone in msg.drones:
        my_address = drone.address
        
        # 2. Loop through all the 'target' drones it is looking at
        # We use enumerate/range to grab the matching index for distance and heading
        for i in range(len(drone.other_addresses)):
            target_address = drone.other_addresses[i]
            distance = drone.distances_to_others[i]
            heading_error = drone.heading_error_to_others_deg[i]

            # Print or use the data!
            rospy.loginfo(f"[Drone {my_address}] relative to [Drone {target_address}]:")
            rospy.loginfo(f"    -> Distance: {distance} m")
            rospy.loginfo(f"    -> Turn Error: {heading_error} deg")
            
            # --- THIS IS WHERE YOUR CONTROL LOGIC GOES ---
            # if my_address == 22 and target_address == 2:
            #     if heading_error > 5.0:
            #         turn_left()
            # ---------------------------------------------

def main():
    # Initialize the node
    rospy.init_node('fleet_data_listener', anonymous=True)

    # Subscribe to the topic
    # Syntax: rospy.Subscriber('topic_name', MessageType, callback_function)
    rospy.Subscriber('/fleet_heading_distances', FleetHeadingDistances, fleet_callback)

    rospy.loginfo("Listening to /fleet_heading_distances...")

    # Keeps python from exiting until this node is stopped (Ctrl+C)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass