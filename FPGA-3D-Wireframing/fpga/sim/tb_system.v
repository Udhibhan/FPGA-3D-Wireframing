`timescale 1ns/1ps
module tb_system;
  reg cs_n = 1, sck = 0, mosi = 0;
  wire [7:0] ic = {4'b0000, mosi, sck, cs_n, 1'b0};
  wire [7:0] led; wire buzzer, scl, sda, dc, cs, rst;
  top #(.OLED_CLK_HZ(100_000)) dut(.interconnect(ic), .led(led), .buzzer(buzzer),
      .oled_scl(scl), .oled_sda(sda), .oled_dc(dc), .oled_cs(cs), .oled_rst(rst));

  // ---- RP2350 side: send one 32-byte packet over SPI (mode 0, 4 MHz) ----
  reg [7:0] pkt [0:31];
  task send_packet(input bad); integer i, b; reg [7:0] c; begin
    c = 0; for (i = 1; i < 31; i = i + 1) c = c ^ pkt[i];
    pkt[31] = bad ? ~c : c;
    cs_n = 0; #200;
    for (i = 0; i < 32; i = i + 1) for (b = 7; b >= 0; b = b - 1) begin
      mosi = pkt[i][b]; #125 sck = 1; #125 sck = 0;
    end
    #200 cs_n = 1; #500;
  end endtask
  task set_text(input integer base, input [95:0] s); integer i;
    begin for (i = 0; i < 12; i = i + 1) pkt[base + i] = s[8*(11-i) +: 8]; end
  endtask

  // ---- Display model: decode the panel's SPI, keep pixels after RAMWR ----
  reg [7:0] sh; integer nbits = 0, after_ramwr = 0, npix = 0, frames = 0, f;
  reg [15:0] pixword; reg half = 0;
  always @(posedge cs) begin nbits = 0; end
  always @(posedge scl) if (!cs) begin
    sh = {sh[6:0], sda}; nbits = nbits + 1;
    if (nbits == 8) begin
      nbits = 0;
      if (!dc) begin
        after_ramwr = (sh == 8'h2C);
        if (sh == 8'h2C) begin npix = 0; half = 0; end
      end else if (after_ramwr) begin
        if (!half) begin pixword[15:8] = sh; half = 1; end
        else begin
          pixword[7:0] = sh; half = 0;
          if (frames == 1) $fwrite(f, "%04x\n", {pixword[4:0], pixword[10:5], pixword[15:11]}); // undo R/B swap
          npix = npix + 1;
          if (npix == 57600) begin
            frames = frames + 1; after_ramwr = 0;
            $display("frame %0d streamed at %0t ns", frames, $time);
            if (frames == 2) begin $fclose(f); $finish; end
          end
        end
      end
    end
  end

  initial begin
    f = $fopen("system_frame.hex", "w");
    pkt[0] = 8'hA5;
    // 1) a corrupted packet (wrong checksum) that must be ignored: ATTACKED
    pkt[1] = 2; pkt[2] = 0; pkt[3] = 0; pkt[4] = 0; pkt[5] = 9; pkt[6] = 0;
    set_text(7, " IGNORE ME  "); set_text(19, " BAD PACKET ");
    send_packet(1);
    // 2) a good packet: BATTLE, 60 %, zombie in range, heading 90 deg
    pkt[1] = 1; pkt[2] = 60; pkt[3] = 1; pkt[4] = 64; pkt[5] = 0; pkt[6] = 3;
    set_text(7, "   BATTLE   "); set_text(19, " HITS 3  60%");
    send_packet(0);
    $display("packet committed: state=%0d charge=%0d flags=%0d heading=%0d link_ok=%0d",
             dut.st_state, dut.st_charge, dut.st_flags, dut.st_heading, dut.link_ok);
    #80_000_000 $display("timeout"); $finish;
  end
endmodule
