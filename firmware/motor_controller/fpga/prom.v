`timescale 1ns/1ns
module prom
(input c,
 input      [ 6:0] addr,
 output reg [31:0] q);

`include "ops.v"

initial q = 32'h0;

always @(posedge c) begin
  case (addr)
    // entry point: current-sense update.
    // bias/scale the input raw current readings
    7'h00: q = { OP_NOP, 8'h00, 8'h00, 8'h00 };
    7'h01: q = { OP_IN , 8'h00, 8'hxx, 8'h00 };
    7'h02: q = { OP_IN , 8'h01, 8'hxx, 8'h01 };
    7'h03: q = { OP_IN , 8'h02, 8'hxx, 8'h02 };
    7'h04: q = { OP_S2F, 8'h00, 8'hxx, 8'h00 }; // raw ADC ticks in float32
    7'h05: q = { OP_S2F, 8'h01, 8'hxx, 8'h01 }; // raw ADC ticks in float32
    7'h06: q = { OP_S2F, 8'h02, 8'hxx, 8'h02 }; // raw ADC ticks in float32
    7'h07: q = { OP_ADD, 8'h00, 8'h20, 8'h03 }; // subtract A current bias
    7'h08: q = { OP_MUL, 8'h03, 8'h21, 8'h03 }; // phase A current in amps
    7'h09: q = { OP_ADD, 8'h01, 8'h20, 8'h04 }; // subtract B current bias
    7'h0a: q = { OP_MUL, 8'h04, 8'h21, 8'h04 }; // phase B current in amps
    7'h0b: q = { OP_ADD, 8'h02, 8'h20, 8'h05 }; // subtract C current bias
    7'h0c: q = { OP_MUL, 8'h05, 8'h21, 8'h05 }; // phase C current in amps
    // clark transform (3 phase -> 2 phase)
    7'h0d: q = { OP_MUL, 8'h03, 8'h22, 8'h06 }; // 0.667 * i_a
    7'h0e: q = { OP_MUL, 8'h04, 8'h23, 8'h07 }; // -0.3 * i_b
    7'h0f: q = { OP_MUL, 8'h05, 8'h23, 8'h08 }; // -0.3 * i_c
    7'h10: q = { OP_ADD, 8'h06, 8'h07, 8'h06 }; // accumulate i_alpha
    7'h11: q = { OP_ADD, 8'h06, 8'h08, 8'h06 }; // i_alpha = r6
    7'h12: q = { OP_MUL, 8'h04, 8'h24, 8'h07 }; // 0.57735 * i_b
    7'h13: q = { OP_MUL, 8'h05, 8'h25, 8'h08 }; // -0.57735 * i_c
    7'h14: q = { OP_ADD, 8'h07, 8'h08, 8'h07 }; // i_beta = r7
    // park transform (stator coordinates -> rotor coordinates)
    7'h15: q = { OP_IN , 8'h03, 8'hxx, 8'h08 }; // r8 = sin(rotor)
    7'h16: q = { OP_IN , 8'h04, 8'hxx, 8'h09 }; // r9 = cos(rotor)
    7'h17: q = { OP_MUL, 8'h09, 8'h06, 8'h0c }; // cos(rotor) * i_alpha
    7'h18: q = { OP_MUL, 8'h08, 8'h07, 8'h0d }; // sin(rotor) * i_beta
    7'h19: q = { OP_MUL, 8'h09, 8'h07, 8'h0e }; // cos(rotor) * i_beta
    7'h1a: q = { OP_MUL, 8'h08, 8'h06, 8'h0f }; // sin(rotor) * i_alpha
    //7'h1b: q = { OP_SUB, 8'h0c, 8'h0d, 8'h0a }; // ra = i_d
    //7'h1b: q = { OP_SUB, 8'h0d, 8'h0c, 8'h0a }; // ra = i_d
    7'h1b: q = { OP_ADD, 8'h3f, 8'h3f, 8'h0a }; // ra = i_d = 0
    7'h1c: q = { OP_ADD, 8'h0e, 8'h0f, 8'h0b }; // rb = i_q

    // PI control loops
    7'h1f: q = { OP_MUL, 8'h0a, 8'h26, 8'h0d }; // rd = error_d = -i_d
    7'h20: q = { OP_IN , 8'h05, 8'hxx, 8'h0c }; // read the target current
    //7'h21: q = { OP_SUB, 8'h0c, 8'h0b, 8'h0e }; // re = error_q = target - i_q
    7'h21: q = { OP_ADD, 8'h0c, 8'h0b, 8'h0e }; // re = error_q = target - i_q

    // TODO: scale the i_d and i_q errors to integrate in amp-seconds
    7'h22: q = { OP_ADD, 8'h0d, 8'h10, 8'h10 }; // integrate i_d error
    7'h23: q = { OP_ADD, 8'h0e, 8'h11, 8'h11 }; // integrate i_q error
    7'h24: q = { OP_IN , 8'h06, 8'hxx, 8'h12 }; // read the integrator limit
    7'h25: q = { OP_MUL, 8'h12, 8'h26, 8'h13 }; // negative limit = -1*limit
    7'h26: q = { OP_MIN, 8'h10, 8'h12, 8'h10 }; // clamp d-integrator high
    7'h27: q = { OP_MAX, 8'h10, 8'h13, 8'h10 }; // clamp d-integrator low
    7'h28: q = { OP_MIN, 8'h11, 8'h12, 8'h11 }; // clamp q-integrator high
    7'h29: q = { OP_MAX, 8'h11, 8'h13, 8'h11 }; // clamp q-integrator low

    7'h2a: q = { OP_IN , 8'h07, 8'hxx, 8'h13 }; // read in Kp
    7'h2b: q = { OP_IN , 8'h08, 8'hxx, 8'h14 }; // read in Ki

    7'h2c: q = { OP_MUL, 8'h13, 8'h0d, 8'h15 }; // Kp * error_d
    7'h2d: q = { OP_MUL, 8'h14, 8'h10, 8'h16 }; // Ki * integrator_d
    7'h2e: q = { OP_ADD, 8'h15, 8'h16, 8'h15 }; // r15 = v_d = effort_d

    7'h2f: q = { OP_MUL, 8'h13, 8'h0e, 8'h16 }; // Kp * error_q
    7'h30: q = { OP_MUL, 8'h14, 8'h11, 8'h17 }; // Ki * integrator_q
    7'h31: q = { OP_ADD, 8'h16, 8'h17, 8'h16 }; // r16 = v_q = effort_q

    // calculate DQ length
    7'h32: q = { OP_MUL , 8'h15, 8'h15, 8'h18 }; // v_d*v_d
    7'h33: q = { OP_MUL , 8'h16, 8'h16, 8'h19 }; // v_q*v_q
    7'h34: q = { OP_ADD , 8'h18, 8'h19, 8'h18 }; // v_d^2 + v_q^2
    7'h35: q = { OP_SQRT, 8'h18, 8'hxx, 8'h18 }; // sqrt(v_d^2 + v_q^2)

    // scale DQ to stay within the max_effort circle 
    7'h36: q = { OP_IN  , 8'h09, 8'hxx, 8'h19 }; // r19 = max effort
    7'h37: q = { OP_MAX , 8'h18, 8'h19, 8'h18 }; // max(max_eff, dq_mag)
    7'h38: q = { OP_DIV , 8'h19, 8'h18, 8'h18 }; // r18 = max_eff/max(max_e, dqmag)
    7'h39: q = { OP_MUL , 8'h15, 8'h18, 8'h15 }; // scale to stay in dq circle
    7'h3a: q = { OP_MUL , 8'h16, 8'h18, 8'h16 }; // scale to stay in dq circle

    // inverse park transform (rotor coordinates -> stator coordinates)
    7'h3d: q = { OP_MUL, 8'h09, 8'h15, 8'h17 }; // cos(rotor) * effort_d
    7'h3e: q = { OP_MUL, 8'h08, 8'h16, 8'h18 }; // sin(rotor) * effort_q
    7'h3f: q = { OP_ADD, 8'h17, 8'h18, 8'h17 }; // r17 = v_alpha = v_a
    7'h3f: q = { OP_ADD, 8'h17, 8'h18, 8'h17 }; // r17 = v_alpha = v_a
    7'h40: q = { OP_MUL, 8'h08, 8'h15, 8'h18 }; // sin(rotor) * effort_d
    7'h41: q = { OP_MUL, 8'h09, 8'h16, 8'h19 }; // cos(rotor) * effort_q
    7'h42: q = { OP_SUB, 8'h19, 8'h18, 8'h18 }; // r18 = v_beta
    // inverse clark transform (2 phase -> 3 phase)
    7'h43: q = { OP_MUL, 8'h17, 8'h27, 8'h1a }; // -0.5 * v_alpha
    7'h44: q = { OP_MUL, 8'h18, 8'h28, 8'h1b }; // 0.86667 * v_beta
    7'h45: q = { OP_ADD, 8'h1a, 8'h1b, 8'h1c }; // r1c = v_b
    7'h46: q = { OP_SUB, 8'h1a, 8'h1b, 8'h1d }; // r1d = v_c

    // scale the signed output voltages into 13-bit positive integers
    // TODO: create scalar from bus-voltage measurement. for now assume 100v
    7'h47: q = { OP_MUL, 8'h17, 8'h2a, 8'h17 }; // v_a = v_a * 4095 / 50volt
    //7'h48: q = { OP_ADD, 8'h17, 8'h29, 8'h17 }; // v_a = v_a + 4095
    7'h48: q = { OP_ADD, 8'h3f, 8'h29, 8'h17 }; // v_a = 4095
    7'h49: q = { OP_MUL, 8'h1c, 8'h2a, 8'h1c }; // v_b = v_a * 4095 / 50volt
    //7'h4a: q = { OP_ADD, 8'h1c, 8'h29, 8'h1c }; // v_b = v_a + 4095
    7'h4a: q = { OP_ADD, 8'h3f, 8'h29, 8'h1c }; // v_b = 4095
    7'h4b: q = { OP_MUL, 8'h1d, 8'h2a, 8'h1d }; // v_c = v_a * 4095 / 50volt
    //7'h4c: q = { OP_ADD, 8'h1d, 8'h29, 8'h1d }; // v_c = v_a + 4095
    7'h4c: q = { OP_ADD, 8'h3f, 8'h29, 8'h1d }; // v_c = 4095
    // saturate to [0, 8191]
    7'h4d: q = { OP_MIN , 8'h17, 8'h2b, 8'h17 }; // pwm_a upper-bound of +8191
    7'h4e: q = { OP_MAX , 8'h17, 8'h3f, 8'h17 }; // pwm_a lower-bound of 0
    7'h4f: q = { OP_MIN , 8'h1c, 8'h2b, 8'h1c }; // pwm_a upper-bound of +8191
    7'h50: q = { OP_MAX , 8'h1c, 8'h3f, 8'h1c }; // pwm_a lower-bound of 0
    7'h51: q = { OP_MIN , 8'h1d, 8'h2b, 8'h1d }; // pwm_a upper-bound of +8191
    7'h52: q = { OP_MAX , 8'h1d, 8'h3f, 8'h1d }; // pwm_a lower-bound of 0
    // convert to 16-bit integers
    7'h53: q = { OP_F2S , 8'h17, 8'hxx, 8'h17 }; // pwm_a as integer
    7'h54: q = { OP_F2S , 8'h1c, 8'hxx, 8'h1c }; // pwm_b as integer
    7'h55: q = { OP_F2S , 8'h1d, 8'hxx, 8'h1d }; // pwm_c as integer
    // write out to pwm registers
    7'h56: q = { OP_OUT , 8'h04, 8'h17, 8'hxx }; // output pwm_a
    7'h57: q = { OP_OUT , 8'h05, 8'h1c, 8'hxx }; // output pwm_b
    7'h58: q = { OP_OUT , 8'h06, 8'h1d, 8'hxx }; // output pwm_c

    //7'h59: q = { OP_OUT , 8'h07, 8'h08, 8'hxx }; // output sin(rotor)
    //7'h5a: q = { OP_OUT , 8'h08, 8'h09, 8'hxx }; // output cos(rotor)
    7'h59: q = { OP_OUT , 8'h07, 8'h0c, 8'hxx }; // output target current
    7'h5a: q = { OP_OUT , 8'h08, 8'h03, 8'hxx }; // output FP current A
    7'h5b: q = { OP_OUT , 8'h09, 8'h0e, 8'hxx }; // output FP current A
    7'h5c: q = { OP_OUT, 8'h00, 8'h0a, 8'hxx }; // write out i_d
    7'h5d: q = { OP_OUT, 8'h01, 8'h0b, 8'hxx }; // write out i_q
    7'h5e: q = { OP_OUT, 8'h02, 8'h15, 8'hxx }; // write out effort_d
    7'h5f: q = { OP_OUT, 8'h03, 8'h16, 8'hxx }; // write out effort_q


    7'h5e: q = { OP_HALT, 8'hxx, 8'hxx, 8'hxx }; // bye

    //////////////////////////////////////////
    //////////////////////////////////////////
    // entry point: reset integrators
    7'h7c: q = { OP_NOP , 8'h00, 8'h00, 8'h00 };
    7'h7d: q = { OP_ADD , 8'h3f, 8'h3f, 8'h10 }; // zero i_d error integrator
    7'h7e: q = { OP_ADD , 8'h3f, 8'h3f, 8'h11 }; // zero i_q error integrator
    7'h7f: q = { OP_HALT, 8'hxx, 8'hxx, 8'hxx }; // bye bye
    default: q = 32'h0;
  endcase
end
endmodule
