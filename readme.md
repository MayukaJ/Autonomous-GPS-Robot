# To Do

Move front sonar 5cm back 
Fix cylinder in front with the colour sensor
Code front black/white detection
Obstacle/check_distance thresh decide



# Done

## turn_to_angle()
runs gps. 
takes running average of location over 100 readings. 
calculates required heading to turn to. 
turns to that angle using compass heading. 

##wall_following()
uses PID.
Calculates distance to wall with side front sonar sensor. 
Maintains required distance. 

## obstacle(int thresh)
maintains thresh distance to obstacles.
uses the front 3 sonar sensors. 

