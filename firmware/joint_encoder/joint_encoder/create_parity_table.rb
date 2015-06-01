#!/usr/bin/env ruby
File.open "parity.c", "w" do |f|
  f.puts '#include "parity.h"'
  f.puts 'const uint8_t g_parity_lookup[32768] = {'
  32768.times do |i|  
    f.puts (i.to_s(2).count('1').odd? ? "  1," : "  0,")+" // #{i}"
  end
  f.puts '};'
end
