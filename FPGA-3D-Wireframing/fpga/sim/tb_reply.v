`timescale 1ns/1ps
// SPI reply: send a 32-byte status packet like code.py (1 MHz, mode 0) and read MISO
module tb_reply;
  reg clk = 0; always #6.667 clk = ~clk;
  reg cs_n = 1, sck = 0, mosi = 0; wire miso;
  wire valid; wire [7:0] st, ch, fl, hd, se, ht, r, p, y; wire [95:0] l1, l2;
  spi_status_rx dut(.clk(clk), .sck_in(sck), .mosi_in(mosi), .cs_n_in(cs_n), .valid(valid), .state(st), .charge(ch),
    .flags(fl), .heading(hd), .secs(se), .hits(ht), .line1(l1), .line2(l2), .roll(r), .pitch(p), .yaw(y),
    .reply({8'd7, 8'd5, 8'd2, 8'd3, 8'd1}), .miso(miso));
  reg [7:0] pk [0:31]; reg [7:0] rx [0:31]; integer i, b;
  initial begin
    pk[0] = 8'hA5; for (i = 1; i < 31; i = i + 1) pk[i] = 8'h20; pk[1] = 1; pk[31] = 0;
    for (i = 1; i < 31; i = i + 1) pk[31] = pk[31] ^ pk[i];
    cs_n = 0; #2000;
    for (i = 0; i < 32; i = i + 1) begin
      rx[i] = 0;
      for (b = 7; b >= 0; b = b - 1) begin mosi = pk[i][b]; #500 sck = 1; rx[i][b] = miso; #500 sck = 0; end
    end
    #300 cs_n = 1; #2000;
    $display("status still decoded: state=%0d (expect 1)", st);
    $display("reply bytes: %h %h %h %h %h %h %h (expect 5a 07 05 02 03 01 00)", rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);
    $finish;
  end
endmodule
