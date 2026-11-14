// spi_status_rx.v - receives the Command Centre's 32-byte status packet.
//
// The RP2350 is the SPI master (mode 0, MSB first, ~1 MHz). It supplies the
// clock, so the FPGA's internal-oscillator tolerance does not matter.
// The new values are committed all at once, and only if the packet is good
// (sync byte 0xA5 and XOR checksum), so the screen never shows half a packet.
//
//   byte 0      0xA5 sync
//   byte 1      state     0 standby, 1 battle, 2 attacked, 3 lost
//   byte 2      laser charge, percent 0..100
//   byte 3      flags     bit0 IR target, bit1 toggles on every laser shot
//   byte 4      compass heading, 0..255 = 0..360 degrees
//   byte 5      rescue seconds left, 0..10
//   byte 6      hits
//   bytes 7-18  text line 1, 12 ASCII characters
//   bytes 19-30 text line 2, 12 ASCII characters
//   byte 31     XOR of bytes 1..30
//
// A second, short packet carries the drone's attitude for the 3D view:
//   byte 0 0xA6 sync, 1 roll, 2 pitch, 3 yaw (0..255 = 0..360 deg), 4 XOR of bytes 1..3
// (The first radar design ignores it: wrong length.)
//
// Reply: while any packet comes in, the FPGA sends 7 bytes back on MISO (SPI mode 0,
// changed on the falling clock edge): 0x5A, then the zombie game's event counters
// hits, kills, misses, approaching, small bites, big bites (each wraps at 255).
module spi_status_rx (
    input  wire        clk,
    input  wire        sck_in,
    input  wire        mosi_in,
    input  wire        cs_n_in,
    output reg         valid = 1'b0,    // one-clock pulse per good packet
    output reg  [7:0]  state = 8'd3,
    output reg  [7:0]  charge = 8'd0,
    output reg  [7:0]  flags = 8'd0,
    output reg  [7:0]  heading = 8'd0,
    output reg  [7:0]  secs = 8'd0,
    output reg  [7:0]  hits = 8'd0,
    output reg  [95:0] line1 = {12{8'h20}},  // char 0 (leftmost) in bits [7:0]
    output reg  [95:0] line2 = {12{8'h20}},
    output reg  [7:0]  roll  = 8'd0,
    output reg  [7:0]  pitch = 8'd0,
    output reg  [7:0]  yaw   = 8'd0,
    input  wire [47:0] reply,           // 6 bytes to send back, first byte in [47:40]
    output wire        miso
);
    // Synchronise the three SPI wires into our clock domain
    reg [2:0] sck_s  = 3'b000;
    reg [2:0] cs_s   = 3'b111;
    reg [1:0] mosi_s = 2'b00;
    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0], sck_in};
        cs_s   <= {cs_s[1:0], cs_n_in};
        mosi_s <= {mosi_s[0], mosi_in};
    end
    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire selected = ~cs_s[1];
    wire cs_rise  = (cs_s[2:1] == 2'b01);
    wire cs_fall  = (cs_s[2:1] == 2'b10);
    wire sck_fall = (sck_s[2:1] == 2'b10);

    // Reply shifter: loaded when CS goes low, next bit after each falling clock edge
    reg [55:0] tx = 56'd0;
    assign miso = tx[55];
    always @(posedge clk)
        if (cs_fall)
            tx <= {8'h5A, reply};
        else if (sck_fall && !cs_s[1])
            tx <= {tx[54:0], 1'b0};

    reg [2:0]   bit_n = 3'd0;
    reg [6:0]   shreg = 7'd0;
    reg [5:0]   count = 6'd0;      // bytes received in this packet
    reg [7:0]   csum  = 8'd0;      // XOR of bytes 1..31 (0 when the packet is good)
    reg [255:0] pkt   = 256'd0;    // byte i ends up in pkt[8*i +: 8]

    wire [7:0] byte_now = {shreg, mosi_s[1]};

    always @(posedge clk) begin
        valid <= 1'b0;
        if (!selected) begin
            // End of packet: commit only a complete, correct packet
            if (cs_rise && count == 6'd32 && pkt[7:0] == 8'hA5 && csum == 8'h00) begin
                state   <= pkt[15:8];
                charge  <= pkt[23:16];
                flags   <= pkt[31:24];
                heading <= pkt[39:32];
                secs    <= pkt[47:40];
                hits    <= pkt[55:48];
                line1   <= pkt[151:56];
                line2   <= pkt[247:152];
                valid   <= 1'b1;
            end
            // 5-byte attitude packet: byte i is now at pkt[8*(27+i) +: 8]
            if (cs_rise && count == 6'd5 && pkt[223:216] == 8'hA6 && csum == 8'h00) begin
                roll  <= pkt[231:224];
                pitch <= pkt[239:232];
                yaw   <= pkt[247:240];
            end
            bit_n <= 3'd0;
            count <= 6'd0;
            csum  <= 8'd0;
        end else if (sck_rise) begin
            shreg <= byte_now[6:0];
            bit_n <= bit_n + 3'd1;
            if (bit_n == 3'd7) begin
                pkt <= {byte_now, pkt[255:8]};
                if (count != 6'd0)
                    csum <= csum ^ byte_now;
                if (count != 6'd63)
                    count <= count + 6'd1;
            end
        end
    end
endmodule
