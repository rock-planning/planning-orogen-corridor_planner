require 'typelib'

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

