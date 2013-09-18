module RosLog
  def self.extract_named_array(name, data)
    expr = /#{name}:\s*\[.*\]/
    data.scan(expr)
  end

  def self.extract_numbers(data)
    expr = /(-?\d?\.\d?e?-?\d*)/
    data.scan(expr).flatten.collect{|s| s.to_f}
  end

  def self.extract_named_float_array(name, data)
    str = extract_named_array(name, data)
    floats = str.collect{|val| extract_numbers(val)}
  end

  module Trajectory
    def self.parse_positions(ros_dump)
      RosLog::extract_named_float_array 'positions', ros_dump
    end

    def self.parse_velocities(ros_dump)
      RosLog::extract_named_float_array 'velocities', ros_dump
    end

    def self.parse_accelerations(ros_dump)
      RosLog::extract_named_float_array 'accelerations', ros_dump
    end
  end
end
