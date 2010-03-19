require 'typelib'
Typelib.specialize "/wrappers/geometry/Spline" do
    def nurbs?; kind == :RATIONAL_BSPLINE || kind == :RATIONAL_BEZIER end
    def coordinate_stride
        if nurbs? then 4
        else 3
        end
    end
end

Typelib.specialize '/corridor_planner/Corridor' do
    def points
        median_curve.vertices.each_slice(median_curve.coordinate_stride).
           map do |x, y, z, _|
               [x, y, z]
           end
    end

    def to_csv(io)
        points.each do |x, y, z|
            io.puts "#{x} #{y} #{z}"
        end
    end
end

Typelib.specialize "/corridor_planner/Plan" do
    CORRIDOR_RECORD_NAMES = { :FRONT_SIDE => 0, :BACK_SIDE => 1 }
    def to_dot(path)
        File.open(path, 'w') do |dot_file|
            dot_file.puts "digraph {"
            dot_file.puts "  rankdir=LR;"
            dot_file.puts "  node [shape=record,height=.1];"
            connections.each do |connection|
                from_side = CORRIDOR_RECORD_NAMES[connection.from_side]
                to_side   = CORRIDOR_RECORD_NAMES[connection.to_side]
                dot_file.puts "   c#{connection.from_idx}:#{from_side} -> c#{connection.to_idx}:#{to_side}"
            end
            corridors.size.times do |i|
                dot_file.puts "   c#{i} [label=\"{<0>|<main> #{i}|<1>}\"];"
            end

            dot_file.puts "};"
        end
    end

    def to_csv(io)
        io.puts "X Y Z"
        corridors.each do |c|
            c.to_csv(io)
        end
    end
end

def traverse(plan, path)
    last_corridor, last_in_side = path.last
    last_out_side =
        if last_in_side == :BACK_SIDE
            :FRONT_SIDE
        else :BACK_SIDE
        end

    next_step = plan.connections.find do |conn|
        conn.from_idx == last_corridor && conn.from_side == last_out_side &&
            !path.include?([conn.to_idx, conn.to_side])
    end

    if !next_step
        raise "something's wrong: path is blocked"
    end
    path << [next_step.to_idx, next_step.to_side]
    if next_step.to_idx == plan.end_corridor
        return
    else
        traverse(plan, path)
    end
end

def output_path(io, plan, path)
    io.puts "X Y Z"
    corridors = plan.corridors.to_a

    path.each do |corridor_idx, side|
        STDERR.puts "entering #{corridor_idx} at #{side}"
        points = corridors[corridor_idx].points
        if side == :BACK_SIDE
            points.reverse
        end

        points.each do |x, y, z|
            io.puts "#{x} #{y} #{z}"
        end
    end
end

