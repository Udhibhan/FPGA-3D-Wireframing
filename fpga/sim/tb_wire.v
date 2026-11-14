`timescale 1ns/1ps
// Renders whole frames of the 3D view: wire3d + radar_render, pixels in the same
// order as top.v presents them ((1,0) first, then raster order, then (0,240)).
module tb_wire;
  reg clk = 0; always #5 clk = ~clk;
  reg [7:0] x = 8'd1, y = 8'd0;
  reg [7:0] roll = 0, pitch = 0, yaw = 0, spin = 6;
  reg [1:0] mode = 0; reg [15:0] charge = 0; reg [7:0] secs = 0;
  reg [95:0] line1, line2; reg blink = 0, beam = 0;
  wire w_hit; wire [1:0] w_level; wire [15:0] color;

  wire3d u_w(.clk(clk), .x(x), .y(y), .roll(roll), .pitch(pitch), .yaw(yaw),
             .spin_step(spin), .hit(w_hit), .level(w_level));
  radar_render u_r(.clk(clk), .x(x), .y(y), .mode(mode), .link_ok(1'b1),
    .charge_a((charge * 164) >> 6), .target(1'b1), .heading(8'd20), .secs(secs),
    .line1(line1), .line2(line2), .sweep(8'd70), .blink(blink), .pulse(1'b1), .beam(beam),
    .noise(16'h1234), .wire_on(1'b1), .w_hit(w_hit), .w_level(w_level), .color(color));

  function [95:0] mk(input [95:0] s); integer i;
    begin for (i = 0; i < 12; i = i + 1) mk[8*i +: 8] = s[8*(11-i) +: 8]; end
  endfunction

  integer f, i, j, n;
  // One frame in display order; when f != 0 the colours are written out
  task one_frame(input save); begin
    for (j = 0; j < 240; j = j + 1) for (i = (j == 0 ? 1 : 0); i < 240; i = i + 1) begin
      if (save && j == 0 && i == 1) $fwrite(f, "0000\n");       // pixel 0: always black
      x = i; y = j;
      repeat (11) @(posedge clk); #1;
      if (save) $fwrite(f, "%04x\n", color);
    end
    x = 0; y = 240; repeat (12) @(posedge clk); #1;
    x = 1; y = 0;   repeat (2) @(posedge clk); #1;                  // next frame starts
  end endtask
  task scene(input [8*24-1:0] name); begin
    one_frame(0);                // new angles are read at the next frame start ...
    one_frame(0);                // ... and shown from the frame after that
    f = $fopen(name, "w");
    one_frame(1);                // ... and this frame shows it
    $fclose(f);
  end endtask

  initial begin
    line1 = mk("  STANDBY   "); line2 = mk(" T28.7C H41% ");
    // 1) standby, level, facing forward
    mode = 0; roll = 0; pitch = 0; yaw = 0; spin = 6;
    one_frame(0);                // power-up: first frame start
    scene("w1_standby.hex");
    // 2) standby, tilted and turned (also dumps the corners for the Python check)
    roll = 14; pitch = 245; yaw = 32;
    scene("w2_tilt.hex");
    for (n = 0; n < 25; n = n + 1)
      $display("V %0d %0d %0d %0d", n, u_w.vsx[n], u_w.vsy[n], u_w.vdw[n]);
    $display("TRIG %0d %0d %0d %0d %0d %0d", u_w.sr, u_w.cr, u_w.sp, u_w.cp, u_w.sy, u_w.cy);
    // 3) battle, banking, charge ring
    mode = 1; roll = 235; pitch = 12; yaw = 220; spin = 19; charge = 60;
    line1 = mk("   BATTLE   "); line2 = mk(" HITS 3  60%");
    scene("w3_battle.hex");
    // 4) battle, laser just fired
    beam = 1; charge = 0; roll = 0; pitch = 0; yaw = 16; line2 = mk(" HITS 4   0%");
    scene("w4_fire.hex");
    // 5) attacked: flipped upside down, countdown
    beam = 0; mode = 2; roll = 128; pitch = 0; yaw = 8; secs = 7; blink = 1; spin = 29;
    line1 = mk(" ATTACKED!  "); line2 = mk("  PRESS A   ");
    scene("w5_attacked.hex");
    // 6) attacked, other half of the flash
    blink = 0; roll = 110;
    scene("w6_attacked_b.hex");
    $finish;
  end
endmodule
