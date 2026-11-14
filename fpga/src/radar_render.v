// radar_render.v - computes the colour of ONE pixel of the 240x240 radar screen.
//
// There is no frame buffer: every pixel is recalculated from its (x, y)
// position and the latest drone status while it is being sent to the screen.
// A 6-stage pipeline keeps each step short enough for 75 MHz; the result
// appears 6 clocks after (x, y) changes (the display gives us 32 clocks).
//
//   S1  offsets from the centre, text/digit position lookups
//   S2  squares (distance), atan2 table read (angle)
//   S3  distance squared, full 0..255 angle, which character is here
//   S4  font read
//   S5  shape tests (rings, sweep trail, compass ticks, charge ring, blip...)
//   S6  pick the colour
//
// 3D view (wire_on): the sweep, rings and blip give way to the wireframe drone
// from wire3d.v; text, bezel, charge ring, countdown and laser beam stay.
//
// Angles: 0 = up, clockwise, 256 = a full turn, so they wrap for free in
// 8-bit arithmetic. The atan2 table covers one quadrant (0..64 = 0..90 deg)
// and is mirrored to the other three.
module radar_render (
    input  wire        clk,
    input  wire [7:0]  x,          // 0..239, left to right
    input  wire [7:0]  y,          // 0..239, top to bottom
    input  wire [1:0]  mode,       // 0 standby, 1 battle, 2 attacked, 3 lost
    input  wire        link_ok,    // 0 = no packet from the RP2350 recently
    input  wire [8:0]  charge_a,   // laser charge as an angle, 0..256
    input  wire        target,     // IR sensor sees a zombie
    input  wire [7:0]  heading,    // compass heading, 0..255
    input  wire [7:0]  secs,       // rescue countdown, 0..10
    input  wire [95:0] line1,      // text, char 0 (leftmost) in bits [7:0]
    input  wire [95:0] line2,
    input  wire [7:0]  sweep,      // current radar sweep angle
    input  wire        blink,      // 2 Hz square wave
    input  wire        pulse,      // blip pulse
    input  wire        beam,       // laser beam visible (just fired)
    input  wire [15:0] noise,      // random bits for the "static" screen
    input  wire        wire_on,    // 3D view selected (centre button)
    input  wire        w_hit,      // wire3d: this pixel is on a line of the drone
    input  wire        w_core,     // wire3d: ... on the centre of the line (else soft edge)
    input  wire [1:0]  w_level,    // wire3d: 3 = nearest line .. 0 = farthest
    input  wire        w_beam,     // wire3d: on a laser beam
    input  wire        w_zhit,     // wire3d: inside a zombie column
    input  wire        w_zhead,
    input  wire        w_zbig,
    input  wire        w_zflash,
    input  wire [3:0]  rz_on,      // zombies for the radar: slot in use
    input  wire [3:0]  rz_big,
    input  wire [3:0]  rz_flash,
    input  wire [31:0] rz_x,       // 4 x 8-bit screen position
    input  wire [31:0] rz_y,
    input  wire        bite,       // a zombie reached the drone: bezel flashes red
    input  wire        warn,       // sensor warning: bezel flashes amber in standby
    input  wire [7:0]  aim,
    input  wire [7:0]  beam_dir,   // where the laser goes: the aim, or locked onto the zombie hit        // drone yaw: radar drone icon and laser point this way
    output reg  [15:0] color       // RGB565
);
    localparam [1:0] STANDBY = 2'd0, BATTLE = 2'd1, ATTACKED = 2'd2, LOST = 2'd3;

    function [15:0] rgb(input [4:0] r, input [5:0] g, input [4:0] b);
        rgb = {r, g, b};
    endfunction

    // ---- Lookup tables ------------------------------------------------------
    reg [7:0] angle_rom [0:16383];   // atan2 of one quadrant, addr = {|dy|, |dx|}
    reg [7:0] font_a    [0:511];     // 5x7 font, addr = {char - 0x20, column}, bit 0 = top
    reg [7:0] font_b    [0:511];     // second copy for the big countdown digits
    reg [7:0] map_tx    [0:255];     // x -> text: in, character 0..11, column
    reg [7:0] map_b1x   [0:255];     // x -> one big digit
    reg [7:0] map_b2x   [0:255];     // x -> two big digits ("10")
    reg [7:0] map_ty    [0:255];     // y -> text line 1 / line 2 / big-digit band, row
    reg [7:0] map_by    [0:255];     // y -> big-digit row
    initial begin
        $readmemh("angle_rom.mem",  angle_rom);
        $readmemh("font_rom.mem",   font_a);
        $readmemh("font_rom.mem",   font_b);
        $readmemh("map_text_x.mem", map_tx);
        $readmemh("map_big1_x.mem", map_b1x);
        $readmemh("map_big2_x.mem", map_b2x);
        $readmemh("map_text_y.mem", map_ty);
        $readmemh("map_big_y.mem",  map_by);
    end

    // "NO LINK" shown when the RP2350 stops sending (char 0 is the lowest byte)
    localparam [95:0] NO_LINK_1 = {8'h20, 8'h20, 8'h20, 8'h4B, 8'h4E, 8'h49,
                                   8'h4C, 8'h20, 8'h4F, 8'h4E, 8'h20, 8'h20};
    localparam [95:0] BLANK     = {12{8'h20}};

    // ================= S1: offsets from the centre, position lookups ========
    wire [8:0] dxw = {1'b0, x} - 9'd120;
    wire [8:0] dyw = {1'b0, y} - 9'd120;
    wire [8:0] dbw = {1'b0, y} - 9'd88;       // the target blip sits at (120, 88)

    reg [6:0] s1_ax, s1_ay;
    reg [7:0] s1_by;
    reg       s1_xneg, s1_yneg;
    reg [7:0] s1_x, s1_y;
    reg [7:0] s1_tx, s1_b1x, s1_b2x, s1_ty, s1_by_row;
    always @(posedge clk) begin
        s1_ax     <= dxw[8] ? (~dxw[6:0] + 7'd1) : dxw[6:0];
        s1_ay     <= dyw[8] ? (~dyw[6:0] + 7'd1) : dyw[6:0];
        s1_by     <= dbw[8] ? (~dbw[7:0] + 8'd1) : dbw[7:0];
        s1_xneg   <= dxw[8];
        s1_yneg   <= dyw[8];                  // 1 = above the centre
        s1_x      <= x;
        s1_y      <= y;
        s1_tx     <= map_tx[x];
        s1_b1x    <= map_b1x[x];
        s1_b2x    <= map_b2x[x];
        s1_ty     <= map_ty[y];
        s1_by_row <= map_by[y];
    end

    // ================= S2: squares + angle table ============================
    reg [13:0] s2_sqx, s2_sqy;
    reg [15:0] s2_sqb;
    reg [7:0]  s2_q;
    reg [6:0]  s2_ax;
    reg        s2_xneg, s2_yneg;
    reg [7:0]  s2_x, s2_y;
    reg [7:0]  s2_tx, s2_bx, s2_ty;
    reg [2:0]  s2_brow;
    always @(posedge clk) begin
        s2_sqx  <= s1_ax * s1_ax;
        s2_sqy  <= s1_ay * s1_ay;
        s2_sqb  <= s1_by * s1_by;
        s2_q    <= angle_rom[{s1_ay, s1_ax}];
        s2_ax   <= s1_ax;
        s2_xneg <= s1_xneg;
        s2_yneg <= s1_yneg;
        s2_x    <= s1_x;
        s2_y    <= s1_y;
        s2_tx   <= s1_tx;
        s2_bx   <= (secs >= 8'd10) ? s1_b2x : s1_b1x;
        s2_ty   <= s1_ty;
        s2_brow <= s1_by_row[2:0];
    end

    // ================= S3: distance, full angle, which character ============
    reg [7:0] angle_c;
    always @(*) begin
        case ({s2_xneg, s2_yneg})
            2'b01:   angle_c = s2_q;              // up-right
            2'b00:   angle_c = 8'd128 - s2_q;     // down-right
            2'b10:   angle_c = 8'd128 + s2_q;     // down-left
            default: angle_c = 8'd0 - s2_q;       // up-left
        endcase
    end

    wire        on_l1  = s2_ty[7];
    wire        on_l2  = s2_ty[6];
    wire [95:0] t_line = on_l2 ? (link_ok ? line2 : BLANK) : (link_ok ? line1 : NO_LINK_1);
    wire [7:0]  t_char = t_line[8 * s2_tx[6:3] +: 8];
    wire [7:0]  t_code = (t_char >= 8'h20 && t_char < 8'h60) ? (t_char - 8'h20) : 8'h1F; // else '?'

    wire        two    = (secs >= 8'd10);
    wire [7:0]  ones   = two ? (secs - 8'd10) : secs;
    wire [7:0]  b_code = (two && !s2_bx[6]) ? 8'h11 : (8'h10 + ones);   // '1', or '0' + ones

    reg [14:0] s3_r2;
    reg [15:0] s3_rb2;
    reg [7:0]  s3_a;
    reg [6:0]  s3_ax;
    reg        s3_yneg;
    reg [7:0]  s3_x, s3_y;
    reg [8:0]  s3_fa, s3_fb;
    reg        s3_t_on, s3_b_on;
    reg [2:0]  s3_t_row, s3_b_row;
    always @(posedge clk) begin
        s3_r2    <= s2_sqx + s2_sqy;
        s3_rb2   <= s2_sqx + s2_sqb;
        s3_a     <= angle_c;
        s3_ax    <= s2_ax;
        s3_yneg  <= s2_yneg;
        s3_x     <= s2_x;
        s3_y     <= s2_y;
        s3_fa    <= {t_code[5:0], s2_tx[2:0]};
        s3_fb    <= {b_code[5:0], s2_bx[2:0]};
        s3_t_on  <= s2_tx[7] && (on_l1 || on_l2);
        s3_b_on  <= s2_bx[7] && s2_ty[5];
        s3_t_row <= s2_ty[2:0];
        s3_b_row <= s2_brow;
    end

    // ================= S4: font read ========================================
    reg [7:0]  s4_ga, s4_gb;
    reg        s4_t_on, s4_b_on;
    reg [2:0]  s4_t_row, s4_b_row;
    reg [14:0] s4_r2;
    reg [15:0] s4_rb2;
    reg [7:0]  s4_a;
    reg [6:0]  s4_ax;
    reg        s4_yneg;
    reg [7:0]  s4_x, s4_y;
    // zombie dots: |dx| and |dy| from each zombie (sum compared in S5)
    reg [7:0] zdx [0:3];
    reg [7:0] zdy [0:3];
    integer zi;
    always @(posedge clk)
        for (zi = 0; zi < 4; zi = zi + 1) begin
            zdx[zi] <= (s3_x > rz_x[8*zi +: 8]) ? s3_x - rz_x[8*zi +: 8] : rz_x[8*zi +: 8] - s3_x;
            zdy[zi] <= (s3_y > rz_y[8*zi +: 8]) ? s3_y - rz_y[8*zi +: 8] : rz_y[8*zi +: 8] - s3_y;
        end

    always @(posedge clk) begin
        s4_ga    <= font_a[s3_fa];
        s4_gb    <= font_b[s3_fb];
        s4_t_on  <= s3_t_on;
        s4_b_on  <= s3_b_on;
        s4_t_row <= s3_t_row;
        s4_b_row <= s3_b_row;
        s4_r2    <= s3_r2;
        s4_rb2   <= s3_rb2;
        s4_a     <= s3_a;
        s4_ax    <= s3_ax;
        s4_yneg  <= s3_yneg;
        s4_x     <= s3_x;
        s4_y     <= s3_y;
    end

    // ================= S5: shape tests (each one a yes/no flag) =============
    wire [7:0] d   = sweep - s4_a;               // how far behind the sweep
    // compass turns with the magnetometer heading in STANDBY; in battle the radar is
    // fixed to the drone's start direction (like the 3D view), so the ticks stay put
    wire [7:0] rel = s4_a + ((mode == STANDBY) ? heading : 8'd0);

    reg       f_text, f_big, f_outside, f_bezel, f_ring, f_cross;
    reg       f_trail, f_sweepln, f_tick, f_north;
    reg       f_chg_band, f_chg_on, f_chg_mark;
    reg       f_blip, f_halo, f_beam_core, f_beam_edge, f_text_band;
    reg [5:0] f_glow;
    reg       f_zdot = 1'b0, f_zdbig = 1'b0, f_zdfl = 1'b0;
    // Drone icon in the centre of the battle radar (forward = up when aim = 0).
    // Screen angle of the aim: yaw turns anticlockwise, screen angles go clockwise.
    wire [7:0] da = s4_a + aim;                  // 0 = exactly along the barrel
    wire [7:0] dm = da - 8'd32;                  // arms at 45, 135, 225, 315 degrees
    wire [7:0] db = s4_a + beam_dir;             // 0 = exactly along the laser
    reg       f_ibar = 1'b0, f_iarm = 1'b0, f_ibody = 1'b0, f_abeam = 1'b0, f_abeam_e = 1'b0;
    always @(posedge clk) begin
        f_ibar    <= (da <= 8'd3 || da >= 8'd253) && (s4_r2 < 15'd361) && (s4_r2 >= 15'd16);  // barrel r 4..19
        f_iarm    <= (dm[5:0] <= 6'd3 || dm[5:0] >= 6'd61) && (s4_r2 < 15'd121);             // arms r < 11
        f_ibody   <= (s4_r2 < 15'd20);                                                         // body r < 4.5
        f_abeam   <= beam && (db <= 8'd1 || db >= 8'd255) && (s4_r2 >= 15'd361) && (s4_r2 < 15'd10816);
        f_abeam_e <= beam && (db == 8'd2 || db == 8'd254) && (s4_r2 >= 15'd361) && (s4_r2 < 15'd10816);
    end
    wire [3:0] zd_in;
    genvar zg;
    generate for (zg = 0; zg < 4; zg = zg + 1) begin : zdot
        assign zd_in[zg] = rz_on[zg] &&
            ({1'b0, zdx[zg]} + {1'b0, zdy[zg]} <= (rz_big[zg] ? 9'd6 : 9'd3));
    end endgenerate
    always @(posedge clk) begin
        f_zdot  <= |zd_in;
        f_zdbig <= |(zd_in & rz_big);
        f_zdfl  <= |(zd_in & rz_flash);
    end
    always @(posedge clk) begin
        f_text      <= s4_t_on && s4_ga[s4_t_row];
        f_big       <= s4_b_on && s4_gb[s4_b_row];
        f_outside   <= (s4_r2 >= 15'd14161);                          // r >= 119
        f_bezel     <= (s4_r2 >= 15'd10816);                          // r >= 104
        f_ring      <= (s4_r2 >= 15'd1560 && s4_r2 < 15'd1642) ||     // r = 40
                       (s4_r2 >= 15'd6320 && s4_r2 < 15'd6482);       // r = 80
        f_cross     <= (s4_x == 8'd120 || s4_y == 8'd120) && (s4_r2 < 15'd10816);
        f_trail     <= (d < 8'd64);
        f_glow      <= 6'd63 - d[5:0];                                // bright right behind the sweep
        f_sweepln   <= (d < 8'd2);
        f_tick      <= (rel[3:0] == 4'd0) && (s4_r2 >= 15'd12100);    // r >= 110
        f_north     <= (mode == STANDBY) && (rel <= 8'd2 || rel >= 8'd254) && (s4_r2 >= 15'd11236);  // r >= 106
        f_chg_band  <= (s4_r2 < 15'd12321);                           // r < 111
        f_chg_on    <= ({1'b0, s4_a} < charge_a);
        f_chg_mark  <= (s4_a == 8'd127 || s4_a == 8'd128);            // 50 % = firing level
        f_blip      <= target && (s4_rb2 < (pulse ? 16'd64 : 16'd36));
        f_halo      <= target && pulse && (s4_rb2 >= 16'd121) && (s4_rb2 < 16'd169);
        f_beam_core <= beam && s4_yneg && (s4_r2 < 15'd10816) && (s4_ax <= 7'd1);
        f_beam_edge <= beam && s4_yneg && (s4_r2 < 15'd10816) && (s4_ax == 7'd2 || s4_ax == 7'd3);
        f_text_band <= ((s4_y >= 8'd54 && s4_y < 8'd72) || (s4_y >= 8'd168 && s4_y < 8'd186)) &&
                       (s4_x >= 8'd40 && s4_x < 8'd200);
    end

    // 3D view line colour: blue in standby, red in battle, flashing when attacked
    reg        f_whit = 1'b0, f_wcore = 1'b0, f_wbeam = 1'b0, f_zhit = 1'b0;
    reg [15:0] f_wcol = 16'h0000, f_zcol = 16'h0000;
    always @(posedge clk) begin
        f_whit  <= w_hit;
        f_wcore <= w_core;
        f_wbeam <= w_beam;
        f_zhit  <= w_zhit;
        // zombie columns: green, the big one purple; heads brighter; white when hit
        if (w_zflash)     f_zcol <= rgb(5'd31, 6'd63, 5'd31);
        else if (w_zbig)  f_zcol <= w_zhead ? rgb(5'd31, 6'd36, 5'd31) : rgb(5'd17, 6'd6,  5'd22);
        else              f_zcol <= w_zhead ? rgb(5'd20, 6'd63, 5'd14) : rgb(5'd4,  6'd38, 5'd6);
        if (mode == STANDBY)
            case (w_level)
                2'd3:    f_wcol <= rgb(5'd10, 6'd44, 5'd31);
                2'd2:    f_wcol <= rgb(5'd5,  6'd30, 5'd26);
                2'd1:    f_wcol <= rgb(5'd2,  6'd19, 5'd19);
                default: f_wcol <= rgb(5'd1,  6'd11, 5'd12);
            endcase
        else if (mode == BATTLE)
            case (w_level)
                2'd3:    f_wcol <= rgb(5'd31, 6'd20, 5'd10);
                2'd2:    f_wcol <= rgb(5'd28, 6'd8,  5'd4);
                2'd1:    f_wcol <= rgb(5'd19, 6'd4,  5'd2);
                default: f_wcol <= rgb(5'd12, 6'd2,  5'd1);
            endcase
        else if (blink)
            f_wcol <= (w_level == 2'd3) ? rgb(5'd31, 6'd63, 5'd31) : rgb(5'd31, 6'd40, 5'd24);
        else
            f_wcol <= (w_level == 2'd3) ? rgb(5'd31, 6'd12, 5'd6)  : rgb(5'd24, 6'd4,  5'd2);
    end

    // ================= S6: choose the colour ================================
    reg [15:0] c;
    always @(*) begin
        if (f_outside)
            c = 16'h0000;
        else if (mode == LOST) begin
            // TV static with a dark box behind the text
            if (f_text)            c = rgb(5'd31, 6'd63, 5'd31);
            else if (f_text_band)  c = 16'h0000;
            else                   c = rgb({1'b0, noise[4:1]}, {1'b0, noise[4:0]}, {1'b0, noise[4:1]});
        end
        else if (f_text || (f_big && mode == ATTACKED))
            c = rgb(5'd31, 6'd63, 5'd31);
        else if (mode == BATTLE && !wire_on && f_ibar)           // drone icon: barrel = aim
            c = rgb(5'd31, 6'd63, 5'd31);
        else if (mode == BATTLE && !wire_on && f_ibody)
            c = rgb(5'd31, 6'd44, 5'd0);
        else if (mode == BATTLE && !wire_on && f_iarm)
            c = rgb(5'd18, 6'd40, 5'd31);
        else if (mode == BATTLE && !wire_on && f_abeam)          // laser along the aim
            c = rgb(5'd31, 6'd63, 5'd31);
        else if (mode == BATTLE && !wire_on && f_abeam_e)
            c = rgb(5'd31, 6'd10, 5'd4);
        else if (f_zdot && !wire_on)                     // zombies on the radar
            c = f_zdfl ? rgb(5'd31, 6'd63, 5'd31) : f_zdbig ? rgb(5'd31, 6'd8, 5'd26) : rgb(5'd14, 6'd63, 5'd4);
        else if (f_bezel) begin
            if (bite)
                c = rgb(5'd31, 6'd4, 5'd2);                  // bitten: red bezel
            else if (warn && mode == STANDBY && pulse)
                c = rgb(5'd31, 6'd42, 5'd0);                 // sensor warning: amber flash
            else if (f_north)
                c = rgb(5'd31, 6'd0, 5'd0);
            else if (f_tick)
                c = (mode == ATTACKED) ? rgb(5'd31, 6'd40, 5'd20) : rgb(5'd18, 6'd40, 5'd18);
            else if (mode == BATTLE && f_chg_band && f_chg_mark)
                c = rgb(5'd31, 6'd63, 5'd31);
            else if (mode == BATTLE && f_chg_band)
                c = f_chg_on ? rgb(5'd31, 6'd44, 5'd0) : rgb(5'd5, 6'd10, 5'd4);
            else if (mode == ATTACKED)
                c = blink ? rgb(5'd31, 6'd0, 5'd0) : rgb(5'd10, 6'd0, 5'd0);
            else
                c = rgb(5'd3, 6'd7, 5'd4);
        end
        else if (wire_on) begin
            // 3D view: line colour was chosen one stage earlier (f_wcol).
            // Line centre full colour, soft edge half (anti-aliasing).
            if (f_wbeam)
                c = f_wcore ? rgb(5'd31, 6'd63, 5'd31) : rgb(5'd31, 6'd12, 5'd4);
            else if (f_whit)
                c = f_wcore ? f_wcol : ((f_wcol >> 1) & 16'h7BEF);
            else if (f_zhit)
                c = f_zcol;
            else if (f_ring)
                c = (mode == STANDBY) ? rgb(5'd1, 6'd7, 5'd9) : rgb(5'd8, 6'd3, 5'd1);
            else if (mode == STANDBY)
                c = rgb(5'd0, 6'd2, 5'd4);
            else if (mode == ATTACKED && blink)
                c = rgb(5'd9, 6'd0, 5'd0);
            else
                c = rgb(5'd3, 6'd0, 5'd0);
        end
        else if (mode == ATTACKED) begin
            if (f_sweepln)              c = rgb(5'd31, 6'd40, 5'd20);
            else if (f_ring || f_cross) c = rgb(5'd26, 6'd20, 5'd12);
            else if (f_trail)           c = rgb(blink ? 5'd31 : f_glow[5:1], 6'd0, 5'd0);
            else                        c = blink ? rgb(5'd16, 6'd0, 5'd0) : rgb(5'd5, 6'd0, 5'd0);
        end
        else begin
            // STANDBY (green) and BATTLE (teal) radar
            if (f_sweepln)              c = rgb(5'd16, 6'd63, 5'd16);
            else if (f_ring || f_cross) c = (mode == BATTLE) ? rgb(5'd2, 6'd30, 5'd18) : rgb(5'd2, 6'd30, 5'd4);
            else if (f_trail)           c = (mode == BATTLE) ? rgb(5'd0, f_glow, f_glow[5:1]) : rgb(5'd0, f_glow, f_glow[5:3]);
            else                        c = rgb(5'd0, 6'd3, 5'd1);
        end
    end

    always @(posedge clk)
        color <= c;
endmodule
