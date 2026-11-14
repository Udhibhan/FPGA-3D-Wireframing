`timescale 1ns/1ps
module tb_pyrx;
  reg clk = 0; always #6.667 clk = ~clk;
  reg cs_n = 1, sck = 0, mosi = 0;
  wire valid; wire [7:0] state, charge, flags, heading, secs, hits; wire [95:0] l1, l2;
  spi_status_rx dut(.clk(clk), .sck_in(sck), .mosi_in(mosi), .cs_n_in(cs_n), .valid(valid),
    .state(state), .charge(charge), .flags(flags), .heading(heading), .secs(secs), .hits(hits), .line1(l1), .line2(l2));
  reg [7:0] p [0:127];
  integer k, i, b, j;
  reg [8*12-1:0] s1, s2;
  initial begin
    $readmemh("py_packets.hex", p);
    for (k = 0; k < 4; k = k + 1) begin
      cs_n = 0; #300;
      for (i = 0; i < 32; i = i + 1) for (b = 7; b >= 0; b = b - 1) begin
        mosi = p[32*k + i][b]; #500 sck = 1; #500 sck = 0;       // 1 MHz like code.py
      end
      #300 cs_n = 1; #2000;
      for (j = 0; j < 12; j = j + 1) begin s1[8*(11-j) +: 8] = l1[8*j +: 8]; s2[8*(11-j) +: 8] = l2[8*j +: 8]; end
      $display("FPGA got: state=%0d charge=%0d flags=%0d heading=%0d secs=%0d hits=%0d line1='%s' line2='%s'",
               state, charge, flags, heading, secs, hits, s1, s2);
    end
    $finish;
  end
endmodule
