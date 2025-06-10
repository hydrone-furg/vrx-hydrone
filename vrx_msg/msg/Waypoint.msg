uint8 NAV_WAYPOINT=0     
uint8  NAV_STATION=1

# MANDATORY:
uint8 nav_type
geometry_msgs/Pose pose

# IF NAV_STATION:
float32 station_duration
