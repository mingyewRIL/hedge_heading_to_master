# Per-drone: address, other drones' addresses, horizontal distance, and heading error to face them.
uint8 address
uint8[] other_addresses     # addresses of other drones (same order as arrays below)
float64[] distances_to_others  # horizontal distance in meters (x-y plane) to each other drone
float64[] heading_error_to_others_deg  # relative yaw: turn this much to face that drone, [-180, 180]
float64[] other_heading_with_respect_to_drone_deg  # other drone heading - this drone heading, normalized [-180, 180]
