require 'orocos'

module OrocosHelpers
  def self.reset_task(task_context)
    if task_context.state == :EXCEPTION
      task_context.reset_exception
    end
    if not (task_context.state == :PRE_OPERATIONAL or task_context.state == :STOPPED)
      task_context.stop
    end
    task_context
  end

  def self.get_task(task_name)
    task_context = Orocos.name_service.get task_name
    reset_task(task_context)
  end
  
  def self.init tasks_name_map
    task_map = {}
    Orocos.initialize
    tasks_name_map.each do |constant, tname|
      tc = get_task(tname)
      task_map[constant] = tc
    end
    task_map
  end
  
end
