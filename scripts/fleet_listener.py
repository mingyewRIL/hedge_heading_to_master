#!/usr/bin/env python3

import rospy
# Import your custom message! 
# (Assuming your package is still named 'hedge_heading_to_master')
from hedge_heading_to_master.msg import FleetHeadingDistances

# Set this to the ID of the drone this script is running on
drone_ID = 22

def fleet_callback(msg):
    """
    This function runs every time a new message arrives on the topic.
    """
    # 1. Loop through every drone reporting its status
    for drone in msg.drones:
        
        # 2. Check if this data block belongs to THIS specific drone
        if drone.address == drone_ID:
            target_list = []
            
            # 3. Build the clean array for our targets
            for i in range(len(drone.other_addresses)):
                target_address = drone.other_addresses[i]
                distance = round(drone.distances_to_others[i], 3)
                heading_error = round(drone.heading_error_to_others_deg[i], 3)

                target_list.append([target_address, distance, heading_error])

            # 4. Print EXACTLY the array and nothing else
            rospy.loginfo(target_list)
            
            # --- THIS IS WHERE YOUR CONTROL LOGIC GOES ---
            # Now 'target_list' is a perfect Python list of lists ready to be used!
            # Example: 
            # for target in target_list:
            #     if target[0] == 22 and target[2] > 5.0:
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
