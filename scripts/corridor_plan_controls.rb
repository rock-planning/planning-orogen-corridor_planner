module CorridorPlanControls
    attr_accessor :plan
    attr_accessor :all_paths
    attr_accessor :current_path_idx
    attr_accessor :corridor_start
    attr_accessor :corridor_end

    attr_reader :map_path
    attr_reader :mls_env
    attr_accessor :classes_path
    attr_reader :strong_edge_filter
    attr_reader :narrow_wide_filter
    attr_accessor :min_width
    attr_accessor :expand_factor

    attr_reader :vizkit_envire
    attr_reader :vizkit_corridors

    def plan=(plan)
        grpPath.setEnabled(true)
        grpAnnotations.setEnabled(true)
        @plan = plan
        @show_all = false
        @all_paths = plan.all_paths
        vizkit_corridors.clearCorridors(0)
        if !@all_paths.empty?
            status.setText("#{@all_paths.size} paths in plan")
            pathIdx.setRange(-1, @all_paths.size - 1)
            setPath(-1)
        else
            status.setText("Empty plan")
        end

        vizkit_corridors.updatePlan(plan)
        update_symbols
    end

    StrongEdgeFilterConfig = Types::CorridorPlanner::StrongEdgeFilterConfig
    NarrowWideFilterConfig = Types::CorridorPlanner::NarrowWideFilterConfig

    def enable_strong_edge_filter(path, map, band, threshold)
        @strong_edge_filter = StrongEdgeFilterConfig.new
        strong_edge_filter.env_path = path
        strong_edge_filter.map_id = map
        strong_edge_filter.band_name = band
        strong_edge_filter.threshold = threshold
    end

    def disable_strong_edge_filter
        @strong_edge_filter = nil
    end

    def enable_narrow_wide_filter(narrow_threshold, wide_threshold)
        @narrow_wide_filter = NarrowWideFilterConfig.new
        narrow_wide_filter.narrow_threshold = narrow_threshold
        narrow_wide_filter.wide_threshold = wide_threshold
    end

    def disable_strong_edge_filter
        @narrow_wide_filter = nil
    end

    def compute(planner_task, start_point, target_point)
        puts start_point.to_a.inspect
        puts target_point.to_a.inspect
        # First, update the viewpoint of the 3D view
        median_point = start_point.to_a.zip(target_point.to_a).map { |a, b| (a + b) / 2 }
        up_vector    = start_point.to_a.zip(target_point.to_a).map { |a, b| (b - a) }
        length = Math.sqrt(up_vector.inject(0) { |length, v| length + v * v })
        view3d.setCameraLookAt(median_point[0], median_point[1], 0)
        view3d.setCameraEye(median_point[0], median_point[1], length)
        view3d.setCameraUp(up_vector[0], up_vector[1], 0)

        # Update the status field
        status.setText("computing")

        result = compute_plan(planner_task, start_point, target_point)
        if result
            yield(result) if block_given?
            self.plan = result
        else
            status.setText("failed")
        end
    end

    def export_current_corridor
        if @current_corridor
            @corridor_log ||= Pocolog::Logfiles.create('exported_corridors')
            @corridor_stream ||= @corridor_log.stream('ui.exported_corridors', Types::Corridors::Corridor, true)
            @corridor_stream.write(Time.now, Time.now, @current_corridor)
        end
    end

    def compute_plan(task, start_point, target_point)
        task.terrain_classes = classes_path
        task.map = map_path

        task.start_point  = start_point
        task.target_point = target_point

        task.margin    = expand_factor || 1.1
        task.min_width = min_width || 0.75
        if mls_env
            task.strong_edge_filter { |p| p.env_path = mls_env }
        end
        if strong_edge_filter
            task.enable_strong_edge_filter = true
            task.strong_edge_filter = strong_edge_filter
        end
        if narrow_wide_filter
            task.enable_narrow_wide_filter = true
            task.narrow_wide_filter = narrow_wide_filter
        end

        begin
            task.configure
            task.start
        rescue Orocos::StateTransitionFailed
            status.setText("failed to configure or start")
            return
        end

        # To get the result when the task finished
        result_reader = task.plan.reader

        last_state = nil
        is_running = false
        while true
            while task.state_changed?
                last_state = task.state(false)
                STDERR.puts last_state
                status.setText(last_state.to_s)
                if !is_running
                    if last_state == :RUNNING
                        is_running = true
                    end
                end
            end
            if is_running
                if task.error?
                    STDERR.puts "Failed ..."
                    task.reset_exception
                    break
                elsif !task.running?
                    plan = result_reader.read
                    STDERR.puts "found #{plan.corridors.size} corridors (graph written to result.dot)"
                    File.open('result.dot', 'w') do |io|
                        io.write plan.to_dot
                    end
                    break
                end
            end
            $qApp.processEvents
            sleep 0.1
        end
        plan
    end


    def setPath(path_idx)
        pathIdx.setValue(path_idx)
        vizkit_corridors.clearCorridors(0)

        if path_idx == -1
            if @show_all
                @all_paths.each do |p|
                    begin
                        corridor = plan.path_to_corridor(p)
                        vizkit_corridors.displayCorridor(corridor)
                    rescue
                    end
                end
            end
        else
            current_path = self.current_path
            startIdx.setRange(0, current_path.size - 1)
            endIdx.setRange(1, current_path.size)
            startIdx.setValue(0)
            endIdx.setValue(current_path.size)

            if @corridor_segments
                computeCorridorSegments
            else
                endIdx.enabled = true
            end
            update_path
        end
    end

    def current_path
        all_paths[pathIdx.value]
    end

    def map_path=(path)
        map_view.load(path)
        @map_path = path
    end

    def mls_env=(path)
        vizkit_corridors.setMLS(path)
        @vizkit_envire ||= view3d.createPlugin('envire')
        vizkit_envire.load(path)
        @mls_env = path
    end

    def setupUI
        @vizkit_corridors = view3d.createPlugin('corridor_planner')
        vizkit_corridors.setAlpha(0.5)
        vizkit_corridors.setZOffset(0.05)

        map_view.extend RasterMapView

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
        btnExport.connect(SIGNAL('clicked()')) do
            export_current_corridor
        end

        lstSymbol.connect(SIGNAL('activated(QString const&)')) do |value|
            if value == "None"
                annotateSymbolIdx.enabled = false
                btnAnnotateCorridor.enabled = false
                vizkit_corridors.setDisplayedAnnotation("")
            else
                annotateSymbolIdx.enabled = true
                btnAnnotateCorridor.enabled = true
                btnSplit.enabled = true
                vizkit_corridors.setDisplayedAnnotation(value)
            end
        end

        btnAnnotationOnPlan.connect(SIGNAL('clicked(bool)')) do |checked|
            puts "btnAnnotationOnPlan: #{checked}"
            vizkit_corridors.displayAnnotationsOnPlan(checked)
            update_path
        end

        btnAnnotateCorridor.connect(SIGNAL('clicked()')) do
            plan.annotate_corridor_segments(
                lstSymbol.currentText, annotateSymbolIdx.value,
                lstSymbol.currentText)
            vizkit_corridors.updatePlan(plan)
        end
        btnSplit.connect(SIGNAL('clicked(bool)')) do |checked|
            if checked
                computeCorridorSegments
                update_path
            else
                endIdx.enabled = true
                @corridor_segments = nil
                update_path
            end
        end
    end

    def computeCorridorSegments
        annotation_idx = plan.find_annotation(lstSymbol.currentText)
        corridor = plan.path_to_corridor(current_path)
        @corridor_segments = corridor.split_annotation_segments(annotation_idx)
        endIdx.enabled = false
        startIdx.setValue(0)
        startIdx.setMaximum(@corridor_segments.size)
    end

    def update_symbols
        if lstSymbol.count != 0
            currentSymbol = lstSymbol.currentText
        end

        lstSymbol.clear
        lstSymbol.addItem("None", Qt::Variant.new(-1))
        plan.annotation_symbols.each_with_index do |symbol_name, idx|
            lstSymbol.addItem(symbol_name, Qt::Variant.new(idx))
        end
        if currentSymbol
            idx = lstSymbol.findText(currentSymbol)
            if idx != -1
                lstSymbol.setCurrentIndex(idx)
            end
        end
    end

    def update_path
        if @plan && pathIdx.value != -1
            begin
                if @corridor_segments
                    @current_corridor = @corridor_segments[startIdx.value].last
                else
                    path = current_path[startIdx.value, endIdx.value - startIdx.value]
                    @current_corridor = plan.path_to_corridor(path)
                end
                vizkit_corridors.clearCorridors(0)
                vizkit_corridors.displayCorridor(@current_corridor)
                btnExport.enabled = true
            rescue Exception => e
                btnExport.enabled = false
                STDERR.puts "ERROR: cannot display path #{path.inspect}"
                STDERR.puts "ERROR:   #{e.message}"
                e.backtrace.each do |line|
                    STDERR.puts "ERROR:     #{line}"
                end
            end
        else
            @current_corridor = nil
            btnExport.enabled = false
        end
    end
end


