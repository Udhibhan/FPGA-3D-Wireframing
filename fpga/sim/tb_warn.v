`timescale 1ns/1ps
module tb_warn;
  reg clk = 0; always #5 clk = ~clk;
  reg [7:0] x = 1, y = 0; wire [15:0] color;
  function [95:0] mk(input [95:0] s); integer i; begin for (i = 0; i < 12; i = i + 1) mk[8*i +: 8] = s[8*(11-i) +: 8]; end endfunction
  radar_render u(.clk(clk), .x(x), .y(y), .mode(2'd0), .link_ok(1'b1), .charge_a(9'd0), .target(1'b0), .heading(8'd20),
    .secs(8'd0), .line1(mk("! WARNING ! ")), .line2(mk("    MAG     ")), .sweep(8'd40), .blink(1'b0), .pulse(1'b1), .beam(1'b0),
    .noise(16'h1234), .wire_on(1'b0), .w_hit(1'b0), .w_core(1'b0), .w_level(2'd0), .w_beam(1'b0), .w_zhit(1'b0), .w_zhead(1'b0),
    .w_zbig(1'b0), .w_zflash(1'b0), .rz_on(4'd0), .rz_big(4'd0), .rz_flash(4'd0), .rz_x(32'd0), .rz_y(32'd0), .bite(1'b0),
    .warn(1'b1), .aim(8'd0), .beam_dir(8'd0), .color(color));
  integer f, i, j;
  initial begin
    f = $fopen("w_warn.hex", "w");
    for (j = 0; j < 240; j = j + 1) for (i = 0; i < 240; i = i + 1) begin
      x = i; y = j; repeat (8) @(posedge clk); #1; $fwrite(f, "%04x\n", color);
    end
    $fclose(f); $finish;
  end
endmodule
