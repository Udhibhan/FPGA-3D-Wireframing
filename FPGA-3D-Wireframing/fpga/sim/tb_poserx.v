`timescale 1ns/1ps
// Pose packets made by code.py (py_pose.hex), plus a status packet and a corrupted pose packet
module tb_poserx;
  reg clk = 0; always #6.667 clk = ~clk;
  reg cs_n = 1, sck = 0, mosi = 0;
  wire valid; wire [7:0] state, charge, flags, heading, secs, hits, roll, pitch, yaw; wire [95:0] l1, l2;
  spi_status_rx dut(.clk(clk), .sck_in(sck), .mosi_in(mosi), .cs_n_in(cs_n), .valid(valid),
    .state(state), .charge(charge), .flags(flags), .heading(heading), .secs(secs), .hits(hits),
    .line1(l1), .line2(l2), .roll(roll), .pitch(pitch), .yaw(yaw));
  reg [7:0] p [0:14]; reg [7:0] st [0:31];
  integer k, i, b;
  task send(input integer n, input integer which, input integer base); begin
    cs_n = 0; #300;
    for (i = 0; i < n; i = i + 1) for (b = 7; b >= 0; b = b - 1) begin
      mosi = which ? st[i][b] : p[base + i][b]; #500 sck = 1; #500 sck = 0;
    end
    #300 cs_n = 1; #2000;
  end endtask
  initial begin
    $readmemh("py_pose.hex", p);
    st[0] = 8'hA5; for (i = 1; i < 31; i = i + 1) st[i] = 8'h20; st[1] = 1; st[2] = 60;
    st[31] = 0; for (i = 1; i < 31; i = i + 1) st[31] = st[31] ^ st[i];
    send(5, 0, 0);  $display("pose 1 -> roll=%0d pitch=%0d yaw=%0d (expect 14 246 32)", roll, pitch, yaw);
    send(32, 1, 0); $display("status -> state=%0d charge=%0d, pose kept: roll=%0d (expect 1 60 14)", state, charge, roll);
    p[10] = 8'h81; p[11] = 8'h00; p[12] = 8'h00; p[13] = 8'h00; p[14] = 8'h00;   // bad checksum
    send(5, 0, 10); $display("bad pose -> roll=%0d (expect 14: ignored)", roll);
    send(5, 0, 5);  $display("pose 2 -> roll=%0d pitch=%0d yaw=%0d (expect 129 2 248)", roll, pitch, yaw);
    $finish;
  end
endmodule
