#!/usr/bin/env ruby

class RandomGaussian
  def initialize(mean = 0.0, sd = 1.0, rng = lambda { Kernel.rand })
    @mean, @sd, @rng = mean, sd, rng
    @compute_next_pair = false
  end

  def rand
    if (@compute_next_pair = !@compute_next_pair)
      # Compute a pair of random values with normal distribution.
      # See http://en.wikipedia.org/wiki/Box-Muller_transform
      theta = 2 * Math::PI * @rng.call
      scale = @sd * Math.sqrt(-2 * Math.log(1 - @rng.call))
      @g1 = @mean + scale * Math.sin(theta)
      @g0 = @mean + scale * Math.cos(theta)
    else
      @g1
    end
  end
end

File.open("sigma_delta_test_data.txt", "w") do |f|
  rg = RandomGaussian.new
  duty = [0.6, 0.2, 0.7]
  cycle = 20
  bits = [ [],[],[] ]
  200.times do
    # "duty cycles"
    3.times do |i|
      n_hi = (cycle * duty[i]).to_i
      n_lo = cycle - n_hi
      n_hi.times { bits[i] += [1] } #f.puts "1" }
      n_lo.times { bits[i] += [0] } #f.puts "0" }
    end
  end
  bits[0].length.times do |i|
    f.puts "#{ bits[0][i] + bits[1][i]*2 + bits[2][i]*4 }"
  end
  
  puts bits.inspect
  #1000.times do
  #  f.puts "#{rg.rand}"
  #end
end
