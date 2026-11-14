`timescale 1ns/1ps
module tb_render;
  reg clk = 0; always #5 clk = ~clk;
  reg [7:0] x = 0, y = 0;
  reg [1:0] mode; reg link_ok; reg [15:0] charge; reg target; reg [7:0] heading, secs;
  reg [95:0] line1, line2; reg [7:0] sweep; reg blink, pulse, beam; reg [15:0] noise;
  wire [15:0] color;
  radar_render dut(.clk(clk), .x(x), .y(y), .mode(mode), .link_ok(link_ok), .charge_a((charge * 164) >> 6),
    .target(target), .heading(heading), .secs(secs), .line1(line1), .line2(line2), .sweep(sweep),
    .blink(blink), .pulse(pulse), .beam(beam), .noise(noise), .color(color));

  // 12-character string literal -> char 0 in the lowest byte (as the packet sends it)
  function [95:0] mk(input [95:0] s); integer i;
    begin for (i = 0; i < 12; i = i + 1) mk[8*i +: 8] = s[8*(11-i) +: 8]; end
  endfunction

  integer f, i, j;
  task frame(input [8*24-1:0] name); begin
    f = $fopen(name, "w");
    for (j = 0; j < 240; j = j + 1) for (i = 0; i < 240; i = i + 1) begin
      x = i; y = j; noise = $random;
      repeat (7) @(posedge clk);
      $fwrite(f, "%04x\n", color);
    end
    $fclose(f);
  end endtask

  initial begin
    link_ok = 1; blink = 0; pulse = 1; beam = 0; target = 0; heading = 20; secs = 0; charge = 0;
    mode = 0; sweep = 70; line1 = mk("  STANDBY   "); line2 = mk(" T28.7C H41% ");
    frame("f1_standby.hex");
    mode = 1; sweep = 200; charge = 60; target = 1; line1 = mk("   BATTLE   "); line2 = mk(" HITS 3  60%");
    frame("f2_battle.hex");
    beam = 1; charge = 10; line2 = mk(" HITS 4  10%");
    frame("f3_fire.hex");
    beam = 0; mode = 2; secs = 7; blink = 1; sweep = 40; line1 = mk(" ATTACKED!  "); line2 = mk("  PRESS A   ");
    frame("f4_attacked.hex");
    secs = 10; blink = 0;
    frame("f5_attacked10.hex");
    mode = 3; line1 = mk("   SIGNAL   "); line2 = mk("    LOST    ");
    frame("f6_lost.hex");
    link_ok = 0;
    frame("f7_nolink.hex");
    $finish;
  end
endmodule
