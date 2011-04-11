module CorridorPlanControls
    attr_accessor :corridor_view

    attr_accessor :plan
    attr_accessor :all_paths
    attr_accessor :current_path_idx
    attr_accessor :corridor_start
    attr_accessor :corridor_end

    def plan=(plan)
        @plan = plan
        pp plan
        @all_paths = plan.all_paths
        if !@all_paths.empty?
            status.setText("#{@all_paths.size} paths in plan")
            pathIdx.setRange(0, @all_paths.size - 1)
            setPath(0)
        else
            status.setText("Empty plan")
        end
    end

    def setPath(path_idx)
        pathIdx.setValue(path_idx)

        current_path = self.current_path
        startIdx.setRange(0, current_path.size - 1)
        endIdx.setRange(1, current_path.size)
        startIdx.setValue(0)
        endIdx.setValue(current_path.size)
        update_path
    end

    def current_path
        all_paths[pathIdx.value]
    end

    def setupControls
        pathIdx.connect(SIGNAL('valueChanged(int)')) do |idx|
            setPath(idx)
        end
        startIdx.connect(SIGNAL('valueChanged(int)')) do |idx|
            endIdx.setMinimum(idx + 1)
            update_path
        end
        endIdx.connect(SIGNAL('valueChanged(int)')) do |idx|
            startIdx.setMaximum(idx - 1)
            update_path
        end
    end

    def update_path
        if @plan
            begin
                path = current_path[startIdx.value, endIdx.value - startIdx.value]
                current_corridor = plan.path_to_corridor(path)
                corridor_view.updateCorridor(current_corridor)
            rescue Exception => e
                STDERR.puts "ERROR: cannot display path #{path.inspect}"
                STDERR.puts "ERROR:   #{e.message}"
            end
        end
    end
end


