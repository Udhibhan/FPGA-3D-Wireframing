`timescale 1ns/1ps
module tb_rx;
  reg clk = 0; always #6.667 clk = ~clk;
  reg cs_n = 1, sck = 0, mosi = 0;
  wire valid; wire [7:0] state, charge, flags, heading, secs, hits; wire [95:0] l1, l2;
  spi_status_rx dut(.clk(clk), .sck_in(sck), .mosi_in(mosi), .cs_n_in(cs_n), .valid(valid),
    .state(state), .charge(charge), .flags(flags), .heading(heading), .secs(secs), .hits(hits), .line1(l1), .line2(l2));
  integer pulses = 0; always @(posedge clk) if (valid) pulses = pulses + 1;
  reg [7:0] pkt [0:31];
  task send(input integer nbytes, input bad); integer i, b; reg [7:0] c; begin
    c = 0; for (i = 1; i < 31; i = i + 1) c = c ^ pkt[i]; pkt[31] = bad ? ~c : c;
    cs_n = 0; #300;
    for (i = 0; i < nbytes; i = i + 1) for (b = 7; b >= 0; b = b - 1) begin mosi = pkt[i][b]; #500 sck = 1; #500 sck = 0; end
    #300 cs_n = 1; #2000;
  end endtask
  integer i;
  initial begin
    for (i = 0; i < 32; i = i + 1) pkt[i] = 8'h20;
    pkt[0] = 8'hA5; pkt[1] = 2; pkt[2] = 77;
    send(32, 1); $display("bad checksum : pulses=%0d state=%0d (expect 0, 3)", pulses, state);
    send(20, 0); $display("short packet : pulses=%0d state=%0d (expect 0, 3)", pulses, state);
    pkt[0] = 8'h5A; send(32, 0); $display("wrong sync   : pulses=%0d state=%0d (expect 0, 3)", pulses, state);
    pkt[0] = 8'hA5; send(32, 0); $display("good packet  : pulses=%0d state=%0d charge=%0d (expect 1, 2, 77)", pulses, state, charge);
    $finish;
  end
endmodule
