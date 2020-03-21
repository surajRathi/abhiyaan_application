## Controller

We have a few ways of getting position and surrounding data:
- Location
  - GPS - accurate but low precision
  - Accelerometer - less accurate but higher precision.
  - Surrounding features combined with an accurate map
- Surroundings
  - Stereoscopic Camera
  - Laser distance scan
  - Maps
  
  
There needs to be two goal in mind. The long term 'goal' and the short term obstacle avoidance.  

We need to first find a path to the location, and constantly change it based on the scans of the surroundings. 
ROS provides a system that implements this by the name `autonav`
