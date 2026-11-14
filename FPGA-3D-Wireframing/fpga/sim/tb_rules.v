`timescale 1ns/1ps
// Game rules only (no drawing): the drone's verdict decides every shot
module tb_rules;
  reg clk = 0; always #5 clk = ~clk;
  reg tick = 0, target = 0, shot = 0, shot_hit = 0, go = 0; reg [1:0] mode = 1; reg [7:0] aim = 8'd40; reg [15:0] rnd = 0;
  wire [3:0] on, bg, fl; wire [39:0] wx, wy; wire [31:0] rx, ry; wire bite, kill; wire [7:0] kills, sang;
  zombies z(.clk(clk), .tick(tick), .mode(mode), .target(target), .shot(shot), .shot_hit(shot_hit), .aim(aim), .rnd(rnd),
    .frame_go(go), .z_on(on), .z_big(bg), .z_flash(fl), .z_wx(wx), .z_wy(wy), .z_rx(rx), .z_ry(ry), .bite(bite), .kill(kill),
    .kills(kills), .shot_ang(sang));
  task ms(input integer t); integer q; begin for (q = 0; q < t; q = q + 1) begin tick = 1; @(posedge clk); #1; tick = 0; repeat (2) @(posedge clk); #1; end end endtask
  task fire(input h); begin shot_hit = h; shot = 1; @(posedge clk); #1; shot = 0; repeat (8) @(posedge clk); #1; end endtask
  task show(input [8*28-1:0] what); integer q; begin
    $write("%0s | kills=%0d laser->%0d |", what, kills, sang);
    for (q = 0; q < 4; q = q + 1) if (z.alive[q] || z.dying[q])
      $write(" [%0d: ang=%0d dist=%0d hp=%0d%0s%0s]", q, z.ang[q], z.dist[q], z.hp[q], z.big[q] ? " BIG" : "", z.dying[q] ? " dying" : "");
    $display("");
  end endtask
  initial begin
    rnd = 16'h0105; target = 1; ms(2); target = 0; ms(2);            // IR sees something: small zombie in front (aim 40)
    show("spawn, aim 40");
    fire(0); show("drone says MISSED");
    fire(1); show("drone says HIT");
    ms(300);
    rnd = 16'h000C; aim = 8'd100; target = 1; ms(2); target = 0;     // big one in front of aim 100
    aim = 8'd20; show("big spawned at 100, turned to 20");
    fire(1); show("HIT while aimed away: lock-on");
    fire(1); fire(1); show("2 more HITs: big dies");
    ms(300); fire(1); show("HIT, nothing on screen");
    rnd = 16'h0102; target = 1; ms(2); target = 0; ms(20000); show("20 s of walking");
    mode = 0; ms(2); show("standby clears");
    $finish;
  end
endmodule
