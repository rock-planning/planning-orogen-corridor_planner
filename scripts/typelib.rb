Typelib.specialize '/corridors/Corridor_m' do
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

Typelib.specialize "/corridors/Plan_m" do
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

