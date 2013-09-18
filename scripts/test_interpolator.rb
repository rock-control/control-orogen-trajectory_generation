require 'orocos'
require 'pry'
require 'vizkit'
$LOAD_PATH << '.'
$LOAD_PATH << 'lib'

require 'robot'
require 'roslog'
require 'type_helpers'
require 'orocos_helpers'

def rad(val)
  val*0.017453292519943295
end

def deg(val)
  val*57.295779513082323
end

@task_config = {:plant => 'orogen_default_trajectory_generation__TestPlant',
          :interpolator => 'orogen_default_trajectory_generation__Task'}

#Mean and variance of joint noise for [position, speed, effort]
@simulation_config = {:joint_noise_mean => [rad(0.0), rad(0.0), rad(0.0)],
                      :joint_noise_variance => [rad(5.0), rad(0.0), rad(0.0)]}

#
# initialize orocos. this requires all needed tasks (see @task_config) to run already!
#
@tasks = OrocosHelpers::init @task_config

#
# Create limits type
#
joint_names_in_trajectory = ['frontArm_joint_1', 'frontArm_joint_2', 'frontArm_joint_3', 'frontArm_joint_4', 'frontArm_joint_5', 'frontArm_joint_6']
limits = TypeHelpers::JointLimits::from_map @robot[:joint_limits]

#
# Create Trajectory type
#
# Extract raw data from log
file_path = "/media/data/Projects/BesMan/Software/RockDev/robots/artemis/data/my_joint_traj.txt"
ros_dump = File.read(file_path)

tr_cfg = {}
tr_cfg[:positions] = RosLog::Trajectory::parse_positions(ros_dump)
tr_cfg[:speeds] = RosLog::Trajectory::parse_velocities(ros_dump)
tr_cfg[:accelerations] = RosLog::Trajectory::parse_accelerations(ros_dump)

waypoints = TypeHelpers::JointsTrajectory::from_map(joint_names_in_trajectory, tr_cfg)

#
# Component Configuration
#
interpolator = @tasks[:interpolator]
plant = @tasks[:plant]

#Interpolator configuration
interpolator.limits = limits
puts "Configuring interpolator"
interpolator.configure

#Plant configuration
plant.limits = limits
plant.noise_mean = TypeHelpers::JointState::from_array(@simulation_config[:joint_noise_mean])
plant.noise_variance = TypeHelpers::JointState::from_array(@simulation_config[:joint_noise_variance])
puts "Configuring plant"
plant.configure

#
# Setup component interconnections
#
puts "Establishing connections"
interpolator.cmd.connect_to plant.cmd
plant.joint_state.connect_to interpolator.joint_state

#
# Setup logging
#
logger = Orocos.name_service.get @task_config[:plant]+"_Logger"
logger.file = Dir.pwd+"/joint_state.log"
logger.log(plant.joint_state,200)
logger.log(interpolator.cmd,200)
#logger.log(interpolator.target,200)

sleep(2)

#
# Start tasks
#
puts "Press Enter to start"
readline
plant.start
interpolator.start
logger.start

# Setpoint
puts "Press Enter write setpoint"
readline
interpolator.target.write waypoints

loop = true
while loop
  if interpolator.state == :REACHED
    puts "Final waypoint reached"
    loop = false
  end
  sleep(0.01)
end
      

puts "Press Enter to quit"
readline

plant.stop
interpolator.stop
logger.stop




