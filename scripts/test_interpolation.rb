def joint_state_from_array(arr)
  ret = Types::Base::JointState.new
  arr.size > 0 ? ret.position = arr[0] : ret.position
  arr.size > 1 ? ret.speed = arr[1] : ret.speed
  arr.size > 2 ? ret.effort = arr[2] : ret.effort
  arr.size > 3 ? ret.raw = arr[3] : ret.raw
  ret
end

def joint_limit_range_from_array(arr)
  if arr.size < 2
    return nil
  end

  ret = Types::Base::JointLimitRange.new    
  ret.min = arr[0]
  ret.max = arr[1]
    
  ret
end

def make_limits()
  names = ['j1', 'j2', 'j3']
  upper_pos = [10, 10, 15]
  lower_pos = [-10, -10, -15]
  upper_vel = [5, 5, 5]
  lower_vel = [-5, -5, -5]
  upper_eff = [100, 100, 100]
  lower_eff = [-100, -100, -100]
  
  limits = JointLimits.new

  upper = upper_pos.zip upper_vel, upper_eff
  lower = lower_pos.zip lower_vel, lower_eff

  limits_raw = upper.zip lower

  el = []
  limits_raw.each do |upper, lower|
    u = joint_state_from_array upper
    l = joint_state_from_array lower
    r = joint_limit_range_from_array [lower, upper]
    el += [r]
  end
  limits.elements = el
  limits.names = names
  limits
end

def make_waypoints(limits)
  traj = JointsTrajectory.new
  traj.names = limits.names
  
  pos = [[0, 0, 0], [5, 5, 5], [10, 10, 10], [2, 2, 2], [-5, -5, -5]]
  vel = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
  eff = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
  
  samples = pos.zip vel, eff
  
  elems = []
  samples.each do |trajs|
    el=[]
    trajs.transpose.each do |t|
      s = joint_state_from_array t
      el += [s]
    end
    elems += [el]
  end
  
  traj.elements = elems.transpose
  traj
end

def current_state_from_waypoints(waypoints)
  current_state = Samples::Joints.new
  current_state.time = Time.now
  el = []
  waypoints.elements.each do |v| el += [v[0]] end
  current_state.elements = el
  current_state.names = limits.names

  current_state
end

def reset_task(task_context)
  if not (task_context.state == :PRE_OPERATIONAL or task_context.state == :STOPPED)
    task_context.stop
  end
  task_context
end

def get_task(task_name)
  task_context = Orocos.name_service.get task_name
  reset_task(task_context)
end


#############################################################
# Initialization
Orocos.initialize
plant_task_name = 'orogen_default_trajectory_generation__TestPlant'
interpolator_task_name = 'orogen_default_trajectory_generation__Task'

limits = make_limits()
waypoints = make_waypoints limits

interpolator = get_task(interpolator_task_name)
plant = get_task(plant_task_name)

# Configuration
interpolator.limits = limits
plant.limits = limits

interpolator.configure
plant.configure

# Make connections
interpolator.cmd.connect_to plant.cmd
plant.joint_state.connect_to interpolator.joint_state









