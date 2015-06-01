#!/usr/bin/env ruby

prog = <<EOPROG
nop 
r00 = in p0
r01 = in p1
r02 = in p2
r03 = s2f r0
r04 = s2f r1
r05 = s2f r2
r03 = add r3 c0
r03 = mul r3 c1   # r3 = i_a
r04 = add r4 c0
r04 = mul r4 c1   # r4 = i_b
r05 = add r5 c0
r05 = mul r5 c1   # r5 = i_c
################################################################
################### clark transform: 3 phase -> 2 phase
r06 = mul r3 c2   # r6 = 0.6667 * i_a
r07 = mul r4 c3   # r7 = -0.3 * i_b
r08 = mul r5 c3   # r8 = -0.3 * i_c
r06 = add r6 r7
r06 = add r6 r8   # r6 = i_alpha
r07 = mul r4 c4   # r7 = 0.57735 * i_b
r08 = mul r5 c5   # r8 = -0.57735 * i_c
r07 = add r7 r8   # r7 = i_beta
################################################################
################### compute electrical sin/cos 
r08 = in p3       # r8 = port 3 = motor-encoder raw (14 bit)
r09 = in p4       # r9 = port 4 = pole count (8 bit)
r0a = s2f r09     # ra = floating-point version of pole count
r0a = div ce ra   # number of enc ticks per electrical revolution
r0b = f2s ra      # convert #ticks/rev back to integer
r0b = usmod r8 rb # number of ticks we are into current elec rev
r0b = s2f rb      # convert electrical modulo to floating-point
r0b = div rb ra   # electrical angle, in [0,1] ( plus offset )
r0b = mul rb cf   # electrical angle, in radians
r08 = sin rb      # sin(electrical angle)
r09 = add rb c10  # r9 = electrical_angle + pi/2
r09 = sin r9      # cos(electrical angle)
###############################################################
################### park transform: stator frame -> rotor frame
r0c = mul r9 r6   # cos(rotor) * i_alpha
r0d = mul r8 r7   # sin(rotor) * i_beta
r0e = mul r9 r7   # cos(rotor) * i_beta
r0f = mul r8 r6   # sin(rotor) * i_alpha
r0a = sub rc rd   # i_d
r0b = add re rf   # i_q
################################################################
################### control loops
r17 = in pd       # r17 = motor velocity in ticks/sec
r17 = mul r17 c11 # r17 = motor velocity in rad/sec
r18 = in pb       # r18 = motor damping gain, in amps / (rad/sec)
r17 = mul r17 r18 # r17 = damping effort, in amps
r0c = mul ra c6   # rc = error_d = -1 * i_d
r0d = in p5       # rd = target current (port 5)
r0d = add r0d r17 # add damping torque to target current
r0e = sub rd rb   # re = error_q = target - i_q
r0f = mul r0c cc  # rf = error_d * adc cycle time of 51.2 usec
r10 = add r10 rf  # r10 = error_d integrator in SI units
r0f = mul r0e cc  # rf = error_q * adc cycle time of 51.2 usec
r11 = add r11 rf  # r11 = error_q integrator in SI units
r12 = in p6       # r12 = integrator limit (port 6)
r10 = min r10 r12 # upper-bound on error_d integrator
r11 = min r11 r12 # upper-bound on error_i integrator
r12 = mul r12 c6  # r12 = -1 * integrator limit
r10 = max r10 r12 # lower-bound on error_d integrator
r11 = max r11 r12 # lower-bound on error_q integrator
r12 = in p7       # r12 = kp (port 7)
r13 = in p8       # r13 = ki (port 8)
r14 = mul r12 rc  # kp * error_d
r15 = mul r13 r10 # ki * integrator_d
r14 = add r14 r15 # r14 = effort_d
r15 = mul r12 re  # kp * error_q
r16 = mul r13 r11 # ki * integrator_i
r15 = add r15 r16 # r15 = effort_q
r17 = in pc       # r17 = motor resistance in ohms (port 0xc)
r17 = mul r17 r0d # feedforward effort = resistance * target current
r15 = add r15 r17 # add feedforward effort to effort_q
#####################################################################
################### calculate DQ magnitude and rescale to max_effort
r17 = mul r14 r14
r18 = mul r15 r15
r17 = add r17 r18
r17 = sqrt r17    # r17 = sqrt(effort_d^2 + effort_q^2)
r18 = in p9       # r18 = max_effort (port 9)
r17 = max r17 r18 # r17 = max(dq_length, max_effort)
r18 = div r18 r17 # r18 = max_effort / r17
r14 = mul r14 r18 # scale D so that DQ magnitude is at most max_effort
r15 = mul r15 r18 # scale Q so that DQ magnitude is at most max_effort
#####################################################################
################### inverse park transform (rotor coords -> stator coords)
r17 = mul r9 r14   # cos(rotor) * effort_d
r18 = mul r8 r15   # sin(rotor) * effort_q
r17 = add r17 r18  # r17 = v_alpha = v_a
r18 = mul r8 r14   # sin(rotor) * effort_d
r19 = mul r9 r15   # cos(rotor) * effort_q
r18 = sub r19 r18  # r18 = v_beta
#####################################################################
################### inverse clark transform (2 phase -> 3 phase)
r1a = mul r17 c7   # r1a = -0.5 * v_alpha
r1b = mul r18 c8   # r1b = 0.86667 * v_beta
r1c = add r1a r1b  # r1c = v_b
r1d = sub r1a r1b  # r1d = v_c
#####################################################################
################### invert, scale the 3-phase voltages into PWM duty values
r21 = in pa        # r21 =  bus voltage (port 10 = 0xa)
r21 = div c9 r21   # r21 =  pwm scalar = pwm_midpoint / bus_voltage
r1e = mul r17 r21  # r1e =  pwm_midpoint / bus_voltage * v_a
r1e = add r1e c9   # r1e += pwm_midpoint
r1f = mul r1c r21  # r1d =  pwm_midpoint / bus_voltage * v_b
r1f = add r1f c9   # r1d += pwm_midpoint
r20 = mul r1d r21  # r1e =  pwm_midpoint / bus_voltage * v_c
r20 = add r20 c9   # r1e += pwm_midpoint
#####################################################################
################### saturate to 13-bit range
r21 = min r1e cb
r21 = max r21 c1f  # pwm_a = r21 is now in [0, max_duty]
r22 = min r1f cb
r22 = max r22 c1f  # pwm_b = r22 is now in [0, max_duty]
r23 = min r20 cb
r23 = max r23 c1f  # pwm_c = r23 is now in [0, max_duty]
#####################################################################
################### convert to integers
r24 = f2s r21
r25 = f2s r22
r26 = f2s r23
#####################################################################
################### write out
#out p4 r24
#out p5 r25
#out p6 r26
out p4 r26
out p5 r25
out p6 r24
out p0 r0a
out p1 r0b
out p2 r14
out p3 r15
out p7 r03
out p8 r04
out p9 r05
#####################################################################
################### toggle the "done" flag
r27 = add c0 c0
out pa r27
r27 = add c1f c1f
out pa r27
halt
#####################################################################
#####################################################################
#####################################################################
######### entry point: reset integrators
@7c
nop
r10 = add c1f c1f # set error_d integrator to zero
r11 = add c1f c1f # set error_q integrator to zero
halt
EOPROG

mem = [0] * 256

ops = { 'nop' => 0, 'in' => 1, 'out' => 2, 'add' => 3, 'sub' => 4,
        's2f' => 5, 'mul' => 6, 'halt' => 7, 'min' => 8, 'max' => 9,
        'f2s' => 10, 'sqrt' => 11, 'div' => 12,
        'sin' => 13, 'usmod' => 15}

def parse_addr addr
  if addr[0] == 'r' or addr[0] == 'p'
    #puts "addr = #{addr} => #{addr[1..-1].to_i(16)}"
    return addr[1..-1].to_i(16)
  elsif addr[0] == 'c'
    return addr[1..-1].to_i(16) + 0x80
  else
    puts "unknown address format: #{addr}"
    exit
  end
end

puts <<EOHEADER
DEPTH = 256;
WIDTH = 32;
ADDRESS_RADIX = HEX;
DATA_RADIX = HEX;
CONTENT
BEGIN
EOHEADER

pc = 0
prog.split("\n").each do |line|
  line.delete!(',')
  line.lstrip!
  next if line.length == 0 or line[0] == '#'
  line = line.split('#')[0] if line.include?('#')
  if line[0] == '@'
    pc = line[1..-1].to_i(16)
    next
  end
  t = line.split(' ') # tokenize
  next if t.size == 0
  if t.include?('=')
    oreg = parse_addr(t[0])
    iregs = t[3..-1].map{ |s| parse_addr(s) }
    op = t[2]
  else
    iregs = t[1..-1].map{ |s| parse_addr(s) }
    oreg = 0
    op = t[0]
  end
  if not ops.has_key? op 
    puts "unknown opcode: [#{op}]"
    exit
  end
  if iregs.size == 0
    iregs = [0, 0]
  elsif iregs.size == 1
    iregs += [0]
  end
  word = (ops[op] << 24) | (iregs[0] << 16) | (iregs[1] << 8) | oreg;
  puts "#{sprintf("%02x", pc)} : #{sprintf("%08x", word)} ; -- #{line}"
  pc += 1
end

puts "END;"

