`timescale 1ns/1ps
// Game rules only (no drawing): the drone's verdict decides every shot
module tb_events;
  reg clk = 0; always #5 clk = ~clk;
  reg tick = 0, target = 0, shot = 0, shot_hit = 0, go = 0; reg [1:0] mode = 1; reg [7:0] aim = 8'd40; reg [15:0] rnd = 0;
  wire [3:0] on, bg, fl; wire [39:0] wx, wy; wire [31:0] rx, ry; wire bite, kill; wire [7:0] kills, hits, miss, nears, bites, bigbites; wire [7:0] sang = 0;
  zombies z(.clk(clk), .tick(tick), .mode(mode), .target(target), .shot(shot), .aim(aim), .rnd(rnd),
    .frame_go(go), .z_on(on), .z_big(bg), .z_flash(fl), .z_wx(wx), .z_wy(wy), .z_rx(rx), .z_ry(ry), .bite(bite), .kill(kill),
    .kills(kills), .hits(hits), .misses(miss), .nears(nears), .bites(bites), .bigbites(bigbites));
  task ms(input integer t); integer q; begin for (q = 0; q < t; q = q + 1) begin tick = 1; @(posedge clk); #1; tick = 0; repeat (2) @(posedge clk); #1; end end endtask
  task fire(input h); begin shot_hit = h; shot = 1; @(posedge clk); #1; shot = 0; repeat (8) @(posedge clk); #1; end endtask
  task show(input [8*28-1:0] what); integer q; begin
    $write("%0s | hits=%0d kills=%0d misses=%0d near=%0d bites=%0d big=%0d |", what, hits, kills, miss, nears, bites, bigbites);
    for (q = 0; q < 4; q = q + 1) if (z.alive[q] || z.dying[q])
      $write(" [%0d: ang=%0d dist=%0d hp=%0d%0s%0s]", q, z.ang[q], z.dist[q], z.hp[q], z.big[q] ? " BIG" : "", z.dying[q] ? " dying" : "");
    $display("");
  end endtask
  initial begin
    rnd = 16'h0110; target = 1; ms(2); target = 0; ms(2);            // small zombie
    rnd = 16'h0050; target = 1; ms(4002); target = 0;                 // big zombie (4 s later)
    ms(12000); show("small reached the drone");
    ms(15000); show("big reached the drone");
    $finish;
  end
endmodule
