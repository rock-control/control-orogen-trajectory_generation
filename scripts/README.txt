test_interpolator.rb is a program to test the trajectory_generation::Task
component on a simulated robot. The robot simulation is provided by the 
trajectory_generation::TestPlant component.


=============
WHAT'S NEEDED
-------------
Mandatory:
  * control/orogen/trajectory_generation
  * control/reflexxes
  * base/types ('control_mode' branch)
  * base/orogen/types
  
Optional:
  * Vizkit
  * RobotModel Plugin for Vizkit
  
=============
CONFIGURATION
-------------
'start_viz.sh':
  * Change the only line in this file to 
      rock-roboviz <PATH_TO_URDF_FILE_OF_ARTEMIS> orogen_default_trajectory_generation__TestPlant
'robot.rb':
  * Change the ':model_file' attribute to "<PATH_TO_URDF_FILE_OF_ARTEMIS>"
  
==========
HOW TO USE
----------
* Start simulated robot:
    orogen_default_trajectory_generation__TestPlant
    
* Start interpolator:
    orogen_default_trajectory_generation__Task
    
* (Optional): Start visualization:
    . start_viz.sh
    
* Run the test program: 
    ruby test_interpolator.rb
  
  
