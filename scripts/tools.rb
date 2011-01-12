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

