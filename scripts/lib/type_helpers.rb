module TypeHelpers
  module JointState
    def self.from_array(arr)
      ret = Types::Base::JointState.new
      arr.size > 0 ? ret.position = arr[0] : ret.position
      arr.size > 1 ? ret.speed = arr[1] : ret.speed
      arr.size > 2 ? ret.effort = arr[2] : ret.effort
      arr.size > 3 ? ret.raw = arr[3] : ret.raw
      ret
    end
  end

  module JointLimitRange
    def self.from_array(arr)
      if arr.size < 2
        return nil
      end

      ret = Types::Base::JointLimitRange.new    
      ret.min = arr[0]
      ret.max = arr[1]
    
      ret
    end
  end
  
  module JointLimits
    def self.from_map(the_map)
      limits = Types::Base::JointLimits.new
  
      el = []
      the_map.each do |jname, lmap| 
        lower = lmap[:lower]
        lower = TypeHelpers::JointState::from_array lower
    
        upper = lmap[:upper]
        upper = TypeHelpers::JointState::from_array upper
    
        r = TypeHelpers::JointLimitRange::from_array([lower, upper])
        el += [r]
      end
  
      limits.elements = el
      limits.names = the_map.keys()
  
      limits
    end
  end
  
  module JointsTrajectory
    def self.from_map(names, the_map)
      traj = Types::Base::JointsTrajectory.new
      traj.names = names
      
      pos = the_map[:positions]
      spd = the_map[:speeds]
      eff = the_map[:accelerations]
  
      samples = pos.zip spd, eff

      elems = []
      samples.each do |trajs|
        el=[]
        trajs.transpose.each do |t|
          s = TypeHelpers::JointState::from_array t
          el += [s]
        end
        elems += [el]
      end
  
      traj.elements = elems
      traj
    end
  end
end
