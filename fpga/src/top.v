// ============================================================================
// EE2028 Command Centre - FPGA radar display (GreyMecha/Army, Lattice ECP5-25K)
//
//   RP2350 --SPI status packet--> spi_status_rx --> radar_render --> GC9A01 display
//          --SPI attitude packet-->     |        \--> wire3d (3D view) --^
//                                       +--> LED effects + buzzer sounds
//
// FPGA buttons (the cross next to the screen, read by the FPGA itself):
//   centre = 3D view on / off    left / right = turn the aim    up = aim offset back to 0
//
// Zombie game (zombies.v): the drone's IR sensor spawns zombies at the edge, they walk in;
// aim with the drone's yaw, the laser kills the nearest one in the aim (big: 3 hits).
// The game's events (hits, kills, misses, approaching, bites) go back to the RP2350 on
// ic_miso while each status packet arrives, and from there to the drone.
//
// The FPGA draws every pixel of the 240x240 round screen in hardware, about
// 40 times per second, with no frame buffer. The RP2350 only sends ~20 small
// status packets per second.
//
// Display driver (oled_gc9a01, oled_init, oled_stream, simple_spi_master,
// gc9a01_init_rom.vh, ecp5_oled_pll) is reused from Hackin7/GreyMechaArmy_Tests.
// ============================================================================
module top #(
    parameter integer OLED_CLK_HZ = 75_000_000,  // smaller only in simulation
    parameter         MIRROR_X    = 0,           // set to 1 if text appears mirrored
    parameter         FLIP_Y      = 0,           // set to 1 if the picture is upside down
    parameter         DEBUG_LEDS  = 0            // 1 = LEDs show internal status (fault finding)
) (
    input  wire [7:1] interconnect,   // from the RP2350: [1] CS, [2] SCK, [3] MOSI
    output wire       ic_miso,        // to the RP2350 (interconnect[0] = GP8): game events
    input  wire [4:0] btn,            // FPGA buttons, pressed = 0: [0] left [1] up [2] centre [3] down [4] right
    output reg  [7:0] led,
    output reg        buzzer,
    output wire       oled_scl,
    output wire       oled_sda,
    output wire       oled_dc,
    output wire       oled_cs,
    output wire       oled_rst
);
    localparam integer CLK_HZ = 75_000_000;
    localparam integer MS_DIV = CLK_HZ / 1000;

    localparam [1:0] STANDBY = 2'd0, BATTLE = 2'd1, ATTACKED = 2'd2, LOST = 2'd3;

    // ---- Clock: internal oscillator /6 -> PLL -> 75 MHz ---------------------
    wire osc_clk, clk, pll_locked;
    defparam OSCI1.DIV = "6";
    OSCG OSCI1 (.OSC(osc_clk));
    ecp5_oled_pll u_pll (.clki(osc_clk), .clko(clk), .locked(pll_locked));

    // Hold everything in reset briefly after power-up / PLL lock
    reg [7:0] reset_counter = 8'd0;
    wire resetn = (&reset_counter) & pll_locked;
    always @(posedge clk)
        if (!pll_locked)
            reset_counter <= 8'd0;
        else if (!(&reset_counter))
            reset_counter <= reset_counter + 8'd1;

    // ---- 1. Status packets from the RP2350 ----------------------------------
    wire        pkt_valid;
    wire [7:0]  st_state, st_charge, st_flags, st_heading, st_secs, st_hits;
    wire [95:0] st_line1, st_line2;
    wire [7:0]  z_kills, z_hits, z_miss, z_near, z_bites, z_bigbites;   // zombie game event counters
    wire [7:0]  st_roll, st_pitch, st_yaw;
    spi_status_rx u_rx (
        .clk(clk), .sck_in(interconnect[2]), .mosi_in(interconnect[3]), .cs_n_in(interconnect[1]),
        .reply({z_hits, z_kills, z_miss, z_near, z_bites, z_bigbites}), .miso(ic_miso),
        .valid(pkt_valid), .state(st_state), .charge(st_charge), .flags(st_flags),
        .heading(st_heading), .secs(st_secs), .hits(st_hits), .line1(st_line1), .line2(st_line2),
        .roll(st_roll), .pitch(st_pitch), .yaw(st_yaw)
    );

    // ---- 2. Timebase: 1 ms tick ----------------------------------------------
    reg [16:0] div  = 17'd0;
    reg        tick = 1'b0;
    reg [15:0] ms   = 16'd0;
    always @(posedge clk) begin
        tick <= (div == MS_DIV - 1);
        div  <= (div == MS_DIV - 1) ? 17'd0 : div + 17'd1;
        if (tick)
            ms <= ms + 16'd1;
    end

    // Link watchdog: no good packet for 1.5 s -> show "NO LINK"
    reg [10:0] since_pkt = 11'd2000;
    always @(posedge clk)
        if (pkt_valid)
            since_pkt <= 11'd0;
        else if (tick && since_pkt != 11'd2000)
            since_pkt <= since_pkt + 11'd1;
    // Registered copies of the status (keeps the logic paths short at 75 MHz)
    reg        link_ok  = 1'b0;
    reg [1:0]  mode     = LOST;
    reg        target   = 1'b0;
    reg        warn     = 1'b0;     // a drone sensor is out of range (standby)
    reg [8:0]  charge_a = 9'd0;   // charge as an angle for the screen, 0..256
    reg [3:0]  lit      = 4'd0;   // charge as a number of LEDs, 0..8
    wire [15:0] charge_p = st_charge * 16'd164;   // percent x 2.56
    wire [15:0] lit_p    = st_charge * 16'd41;    // percent x 8 / 100
    always @(posedge clk) begin
        link_ok  <= (since_pkt < 11'd1500);
        mode     <= link_ok ? st_state[1:0] : LOST;
        target   <= st_flags[0];
        warn     <= st_flags[2];
        charge_a <= charge_p[14:6];
        lit      <= (lit_p[12:9] > 4'd8) ? 4'd8 : lit_p[12:9];
    end
    wire       shot   = st_flags[1];

    // Radar sweep: one turn in ~3 s (standby), ~1.5 s (battle), ~1 s (attacked)
    reg [15:0] sweep_acc = 16'd0;
    always @(posedge clk)
        if (tick)
            sweep_acc <= sweep_acc + ((mode == ATTACKED) ? 16'd66 :
                                      (mode == BATTLE)   ? 16'd44 : 16'd22);

    // 2 Hz blink (250 ms on, 250 ms off)
    reg [7:0] blink_ms = 8'd0;
    reg       blink    = 1'b0;
    always @(posedge clk)
        if (tick) begin
            if (blink_ms == 8'd249) begin
                blink_ms <= 8'd0;
                blink    <= ~blink;
            end else
                blink_ms <= blink_ms + 8'd1;
        end

    // Laser shot: the shot flag toggles once per shot
    reg       shot_prev = 1'b0;
    reg       seen_pkt  = 1'b0;   // ignore the very first packet's flag value
    reg [7:0] flash_ms  = 8'd0;   // LEDs flash
    reg [7:0] pew_ms    = 8'd0;   // "pew" sound and laser beam on screen
    reg [15:0] pew_half = 16'd14400;  // pew pitch: falls a little every millisecond
    reg        shot_pulse = 1'b0;     // one clock per laser shot (for the zombie game)
    always @(posedge clk) begin
        shot_pulse <= 1'b0;
        if (pkt_valid) begin
            seen_pkt  <= 1'b1;
            shot_prev <= shot;
            if (seen_pkt && shot != shot_prev) begin
                shot_pulse <= 1'b1;
                flash_ms <= 8'd120;
                pew_ms   <= 8'd150;
                pew_half <= 16'd14400;                  // starts at 2.6 kHz
            end
        end else if (tick) begin
            if (flash_ms != 8'd0) flash_ms <= flash_ms - 8'd1;
            if (pew_ms   != 8'd0) begin
                pew_ms   <= pew_ms - 8'd1;
                pew_half <= pew_half + 16'd154;         // ... down to 1 kHz after 150 ms
            end
        end
    end

    // ---- FPGA buttons: 3D view on/off, camera orbit ------------------------
    // Sampling every 8 ms ignores contact bounce (it lasts only a few ms).
    reg [4:0] btn_s1 = 5'h1F, btn_s2 = 5'h1F, btn_db = 5'h1F;
    reg [2:0] db_ms    = 3'd0;
    reg       wire_on  = 1'b0;     // 3D view selected
    reg [7:0] orbit    = 8'd0;     // aim offset from left / right (adds to the drone's yaw)
    reg [6:0] chirp_ms = 7'd0;     // short beep when the view changes
    always @(posedge clk) begin
        btn_s1 <= btn;
        btn_s2 <= btn_s1;
        if (tick) begin
            db_ms <= db_ms + 3'd1;
            if (chirp_ms != 7'd0)
                chirp_ms <= chirp_ms - 7'd1;
            if (db_ms == 3'd7) begin
                btn_db <= btn_s2;
                if (btn_db[2] && !btn_s2[2]) begin     // centre just pressed
                    wire_on  <= ~wire_on;
                    chirp_ms <= 7'd80;
                end
                if (!btn_s2[1])                        // up: camera back behind the drone
                    orbit <= 8'd0;
                else if (!btn_s2[0])                   // left held: turn (2 s per turn)
                    orbit <= orbit + 8'd1;
                else if (!btn_s2[4])                   // right held
                    orbit <= orbit - 8'd1;
            end
        end
    end

    // Propellers spin faster as the drone gets busier
    reg [7:0] spin = 8'd0;
    always @(posedge clk)
        spin <= (mode == ATTACKED) ? 8'd29 : (mode == BATTLE) ? 8'd19 :
                (mode == STANDBY)  ? 8'd6  : 8'd0;
    reg [7:0] view_yaw = 8'd0;
    always @(posedge clk)
        view_yaw <= st_yaw + orbit;

    // Random bits for the TV-static screen (16-bit LFSR, one step per clock)
    reg [15:0] lfsr = 16'hACE1;
    always @(posedge clk)
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

    // ---- 3. Screen: render the pixel the display will ask for next ----------
    // The display driver asks for pixels in order (pixel_index = y*240 + x).
    // We compute pixel_index + 1 in advance, so its colour is ready the moment
    // the driver moves on (it allows 32 clocks per pixel; we need 6).
    wire [15:0] pixel_index;
    reg  [15:0] pidx_prev  = 16'd0;
    reg  [7:0]  nx = 8'd1, ny = 8'd0;
    reg  [15:0] pixel_data = 16'h0000;
    wire [15:0] color_next;
    wire        w_hit, w_core, w_beam, w_zhit, w_zhead, w_zbig, w_zflash, w_go;
    wire [1:0]  w_level;
    wire [3:0]  z_on, z_big, z_flash;
    wire [39:0] z_wx, z_wy;
    wire [31:0] z_rx, z_ry;
    wire        z_bite, z_kill;

    always @(posedge clk) begin
        pidx_prev <= pixel_index;
        if (pixel_index == 16'd0) begin
            pixel_data <= 16'h0000;          // pixel 0 is a corner: always black
            nx <= 8'd1;
            ny <= 8'd0;
        end else if (pixel_index != pidx_prev) begin
            pixel_data <= color_next;        // colour of (nx, ny) = the new pixel_index
            if (nx == 8'd239) begin
                nx <= 8'd0;
                ny <= ny + 8'd1;
            end else
                nx <= nx + 8'd1;
        end
    end

    radar_render u_render (
        .clk(clk), .x(MIRROR_X ? 8'd239 - nx : nx), .y(FLIP_Y ? 8'd239 - ny : ny),
        .mode(mode), .link_ok(link_ok),
        .charge_a(charge_a), .target(target), .heading(st_heading), .secs(st_secs),
        .line1(st_line1), .line2(st_line2), .sweep(sweep_acc[15:8]),
        .blink(blink), .pulse(ms[8]), .beam(pew_ms != 8'd0), .noise(lfsr),
        .wire_on(wire_on), .w_hit(w_hit), .w_core(w_core), .w_level(w_level), .w_beam(w_beam),
        .w_zhit(w_zhit), .w_zhead(w_zhead), .w_zbig(w_zbig), .w_zflash(w_zflash),
        .rz_on(z_on), .rz_big(z_big), .rz_flash(z_flash), .rz_x(z_rx), .rz_y(z_ry),
        .bite(z_bite), .aim(view_yaw), .beam_dir(view_yaw), .warn(warn),
        .color(color_next)
    );

    // 3D view: fed the plain raster position (it counts pixels in order)
    wire3d u_wire (
        .clk(clk), .x(nx), .y(ny),
        .roll(st_roll), .pitch(st_pitch), .yaw(view_yaw), .spin_step(spin),
        .beam(pew_ms != 8'd0),
        .z_on(z_on), .z_big(z_big), .z_flash(z_flash), .z_wx(z_wx), .z_wy(z_wy),
        .frame_go(w_go), .hit(w_hit), .core(w_core), .level(w_level), .beam_px(w_beam),
        .zhit(w_zhit), .zhead(w_zhead), .zbig(w_zbig), .zflash(w_zflash)
    );

    // Zombie game: aim = the yaw the drone is drawn with
    zombies u_zombies (
        .clk(clk), .tick(tick), .mode(mode), .target(target), .shot(shot_pulse),
        .aim(view_yaw), .rnd(lfsr), .frame_go(w_go),
        .z_on(z_on), .z_big(z_big), .z_flash(z_flash), .z_wx(z_wx), .z_wy(z_wy),
        .z_rx(z_rx), .z_ry(z_ry), .bite(z_bite), .kill(z_kill), .kills(z_kills),
        .hits(z_hits), .misses(z_miss), .nears(z_near), .bites(z_bites), .bigbites(z_bigbites)
    );

    wire init_done, streaming;
    oled_gc9a01 #(.CLK_HZ(OLED_CLK_HZ), .INIT_SPI_CLK_DIV(8'd4)) u_oled (
        .clk(clk), .resetn(resetn),
        .pixel_index(pixel_index), .pixel_data(pixel_data),
        .oled_scl(oled_scl), .oled_sda(oled_sda), .oled_dc(oled_dc),
        .oled_cs(oled_cs), .oled_rst(oled_rst),
        .init_done(init_done), .streaming(streaming)
    );

    // ---- 4. LEDs --------------------------------------------------------------
    reg [7:0] pwm = 8'd0;
    always @(posedge clk)
        pwm <= pwm + 8'd1;
    wire [7:0]  breathe = ms[11] ? ~ms[10:3] : ms[10:3];   // ~4 s triangle
    wire [8:0]  bar     = (9'd1 << lit) - 9'd1;
    wire [7:0]  sweep_dot = 8'd1 << ms[9:7];

    // Diagnostic: one LED per internal status signal
    reg pkt_toggle = 1'b0;
    always @(posedge clk)
        if (pkt_valid)
            pkt_toggle <= ~pkt_toggle;

    always @(posedge clk) begin
        if (DEBUG_LEDS)
            // 7 PLL locked | 6 out of reset | 5 screen set up | 4 screen data flowing
            // 3 screen reset released | 2 packets arriving | 1 heartbeat | 0 flips per packet
            led <= {pll_locked, resetn, init_done, ~oled_cs, oled_rst, link_ok, ms[9], pkt_toggle};
        else if (flash_ms != 8'd0)
            led <= 8'hFF;
        else if (z_bite)
            led <= ms[6] ? 8'hFF : 8'h00;                             // bitten: fast flash
        else if (z_kill)
            led <= 8'hAA;
        else begin
            case (mode)
                STANDBY:  led <= (pwm < breathe) ? 8'hFF : 8'h00;
                BATTLE:   led <= bar[7:0] ^ (target ? sweep_dot : 8'h00);
                ATTACKED: led <= blink ? 8'hF0 : 8'h0F;
                default:  led <= 8'h00;
            endcase
        end
    end

    // ---- 5. Sound: f = CLK_HZ / (2 * half) ----------------------------------
    wire [9:0] siren_tri = ms[10] ? ~ms[9:0] : ms[9:0];
    reg [15:0] half     = 16'd18750;
    reg        sound_on = 1'b0;
    always @(posedge clk) begin                                       // registered: short paths
        sound_on <= 1'b1;
        half     <= 16'd18750;                                        // 2 kHz
        if (pew_ms != 8'd0)
            half <= pew_half;                                         // "pew" 2.6 -> 1 kHz
        else if (chirp_ms != 7'd0)
            half <= wire_on ? 16'd12500 : 16'd25000;                  // 3D on 3 kHz, off 1.5 kHz
        else if (z_kill)
            half <= 16'd41667;                                        // zombie down: 900 Hz
        else if (z_bite) begin
            half     <= 16'd62500;                                    // bite: 600 Hz growl
            sound_on <= ms[5];
        end
        else if (mode == ATTACKED)
            half <= 16'd31250 + {siren_tri, 5'd0} - siren_tri;        // siren 1.2 <-> 0.6 kHz (x31)
        else if (mode == BATTLE && target && ms[8:0] < 9'd40)
            half <= 16'd18750;                                        // target-lock beep
        else
            sound_on <= 1'b0;
    end

    reg [15:0] tone_cnt = 16'd0;
    always @(posedge clk) begin
        if (!sound_on) begin
            tone_cnt <= 16'd0;
            buzzer   <= 1'b0;
        end else if (tone_cnt >= half) begin
            tone_cnt <= 16'd0;
            buzzer   <= ~buzzer;
        end else
            tone_cnt <= tone_cnt + 16'd1;
    end
endmodule
