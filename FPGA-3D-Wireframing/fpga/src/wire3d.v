// wire3d.v - real-time 3D view: the drone (tilted by its real IMU) and the zombies.
//
// No frame buffer, like the radar: the picture is decided pixel by pixel as the
// screen is refreshed. Two parts:
//
// 1. SETUP ENGINE (once per frame, ~140 clocks, at the start of a frame; rows 0-15
//    are hidden behind the bezel, so the testers pick up the new numbers at row 16)
//    - looks up sin/cos of roll, pitch, yaw and the 4 propeller angles
//    - vertex pipeline: 32 corners of the drone are rotated (roll, pitch, yaw);
//      8 zombie points (feet and heads, fixed to the ground) skip the rotation.
//      Then all are viewed from 30 degrees above with perspective.
//      One point enters the pipeline per clock (14 multipliers).
//    - edge setup: for each of the 32 lines, the numbers its tester needs;
//      for each of the 4 zombies, the screen box of its column.
//
// 2. TESTERS (all in parallel, every pixel)
//    32 line testers: each keeps two running sums for the current pixel,
//    E = how far the pixel is to the side of the line, D = how far along it.
//    One pixel right adds a constant, a new row adds another: adders only.
//    This is the "edge function" trick GPUs use. |E| < half a pixel = line
//    core (full colour); up to one pixel = fringe (half colour): anti-aliasing.
//    4 column testers: is the pixel inside this zombie's column?
//
// Angles: 0..255 = 0..360 degrees. Model units are about one pixel.
// Body axes: X right, Y forward (nose), Z up. Screen: X right, up = Z, into the screen = Y.
module wire3d (
    input  wire        clk,
    input  wire [7:0]  x,          // pixel being drawn; must advance in raster order
    input  wire [7:0]  y,
    input  wire [7:0]  roll,
    input  wire [7:0]  pitch,
    input  wire [7:0]  yaw,
    input  wire [7:0]  spin_step,  // propeller turn per frame
    input  wire        beam,       // laser firing: the barrels become beams
    input  wire [3:0]  z_on,       // zombie slots in use
    input  wire [3:0]  z_big,      // big zombie (boss)
    input  wire [3:0]  z_flash,    // just hit / dying
    input  wire [39:0] z_wx,       // zombie ground positions, 4 x signed 10-bit
    input  wire [39:0] z_wy,
    output reg         frame_go = 1'b0,  // one pulse per frame: setup starts (for zombies.v)
    output reg         hit    = 1'b0,    // pixel on a drone line (core or fringe)
    output reg         core   = 1'b0,    // ... on the centre of a line
    output reg  [1:0]  level  = 2'd0,    // 3 = nearest line (brightest) .. 0 = farthest
    output reg         beam_px = 1'b0,   // pixel on a laser beam
    output reg         zhit   = 1'b0,    // pixel inside a zombie column
    output reg         zhead  = 1'b0,    // ... its head
    output reg         zbig   = 1'b0,    // ... a big zombie
    output reg         zflash = 1'b0     // ... a zombie that was just hit
);
    localparam integer NVD = 32;  // drone corners
    localparam integer NV  = 40;  // + 8 zombie points
    localparam integer NE  = 32;  // drone lines
    localparam integer NZ  = 4;   // zombie columns

    // ---- Lookup tables ----------------------------------------------------
    reg signed [9:0] sin_rom [0:255];   // sin * 256
    reg        [8:0] rec_rom [0:255];   // 256 * D / (D + depth), depth -128..127
    initial begin
        $readmemh("sin_rom.mem", sin_rom);
        $readmemh("rec_rom.mem", rec_rom);
    end

    // ---- Where are we on the screen? ----------------------------------------
    reg [7:0] px = 8'd0, py = 8'd0;
    reg       step = 1'b0, rstart = 1'b0, r16 = 1'b0;
    wire      moved = (x != px) || (y != py);
    always @(posedge clk) begin
        px       <= x;
        py       <= y;
        step     <= moved;
        frame_go <= moved && (y == 8'd0) && (py != 8'd0);   // first pixel of a new frame: set up
        rstart   <= moved && (x == 8'd0);                   // first pixel of a new row
        r16      <= moved && (x == 8'd0) && (y == 8'd16);   // first visible row: testers start
    end

    // =======================================================================
    // 1. SETUP ENGINE
    // =======================================================================
    localparam [1:0] S_IDLE = 2'd0, S_TRIG = 2'd1, S_VERT = 2'd2, S_EDGE = 2'd3;
    reg [1:0] st  = S_IDLE;
    reg [5:0] cnt = 6'd0;
    reg [7:0] a_roll = 8'd0, a_pitch = 8'd0, a_yaw = 8'd0, alpha = 8'd0;
    reg       a_beam = 1'b0;
    reg [3:0] a_zon = 4'd0, a_zbig = 4'd0, a_zfl = 4'd0;
    reg       vin_ok = 1'b0, ein_ok = 1'b0;
    reg [5:0] vin_i = 6'd0, ein_j = 6'd0;

    always @(posedge clk) begin
        vin_ok <= 1'b0;
        ein_ok <= 1'b0;
        case (st)
            S_IDLE:
                if (frame_go) begin             // take the latest angles and zombies
                    st      <= S_TRIG;
                    cnt     <= 6'd0;
                    a_roll  <= roll;
                    a_pitch <= pitch;
                    a_yaw   <= yaw;
                    a_beam  <= beam;
                    a_zon   <= z_on;
                    a_zbig  <= z_big;
                    a_zfl   <= z_flash;
                    alpha   <= alpha + spin_step;
                end
            S_TRIG: begin                       // 14 sin/cos look-ups
                cnt <= cnt + 6'd1;
                if (cnt == 6'd17) begin
                    st  <= S_VERT;
                    cnt <= 6'd0;
                end
            end
            S_VERT: begin                       // 40 points, one per clock
                if (cnt < NV) begin
                    vin_ok <= 1'b1;
                    vin_i  <= cnt;
                end
                cnt <= cnt + 6'd1;
                if (cnt == NV + 16) begin       // wait for the pipeline to empty
                    st  <= S_EDGE;
                    cnt <= 6'd0;
                end
            end
            default: begin                      // S_EDGE: 32 lines + 4 zombies, one per clock
                if (cnt < NE + NZ) begin
                    ein_ok <= 1'b1;
                    ein_j  <= cnt;
                end
                cnt <= cnt + 6'd1;
                if (cnt == NE + NZ + 12)
                    st <= S_IDLE;
            end
        endcase
    end

    // ---- sin / cos look-ups (one per clock, result two clocks later) -------
    reg [7:0] taddr;
    always @(*) begin
        case (cnt[3:0])
            4'd0:    taddr = a_roll;
            4'd1:    taddr = a_roll + 8'd64;          // cos = sin(+90 deg)
            4'd2:    taddr = a_pitch;
            4'd3:    taddr = a_pitch + 8'd64;
            4'd4:    taddr = a_yaw;
            4'd5:    taddr = a_yaw + 8'd64;
            4'd6:    taddr = alpha;                   // propeller 0 turns one way
            4'd7:    taddr = alpha + 8'd64;
            4'd8:    taddr = 8'd32 - alpha;           // propeller 1 the other way
            4'd9:    taddr = 8'd96 - alpha;
            4'd10:   taddr = alpha + 8'd64;           // propeller 2 like 0, offset
            4'd11:   taddr = alpha + 8'd128;
            4'd12:   taddr = 8'd96 - alpha;           // propeller 3 like 1, offset
            default: taddr = 8'd160 - alpha;
        endcase
    end

    reg signed [9:0] trig_q = 10'sd0;
    reg [3:0]        tsel_d = 4'd0;
    reg              tval_d = 1'b0;
    reg signed [9:0] sr = 0, cr = 10'sd256, sp = 0, cp = 10'sd256, sy = 0, cy = 10'sd256;
    reg signed [9:0] rox [0:3];       // propeller tip offset (radius 22) for each rotor
    reg signed [9:0] roy [0:3];
    integer k;
    initial for (k = 0; k < 4; k = k + 1) begin rox[k] = 10'sd22; roy[k] = 10'sd0; end

    // value * 22 / 256 (propeller radius 22), one clock after the table read
    reg signed [9:0] tq_r = 10'sd0;
    reg [3:0]        tsel_r = 4'd0;
    reg              tval_r = 1'b0;
    wire signed [15:0] tq_x = $signed({{6{tq_r[9]}}, tq_r});
    wire signed [15:0] tq22 = (tq_x <<< 4) + (tq_x <<< 3) - (tq_x <<< 1) + 16'sd128;  // x 22, + 0.5
    wire signed [9:0]  tip  = tq22 >>> 8;

    always @(posedge clk) begin
        trig_q <= sin_rom[taddr];
        tsel_d <= cnt[3:0];
        tval_d <= (st == S_TRIG);
        tq_r   <= trig_q;
        tsel_r <= tsel_d;
        tval_r <= tval_d;
        if (tval_r) begin
            case (tsel_r)
                4'd0:  sr <= tq_r;
                4'd1:  cr <= tq_r;
                4'd2:  sp <= tq_r;
                4'd3:  cp <= tq_r;
                4'd4:  sy <= tq_r;
                4'd5:  cy <= tq_r;
                4'd6:  roy[0] <= tip;
                4'd7:  rox[0] <= tip;
                4'd8:  roy[1] <= tip;
                4'd9:  rox[1] <= tip;
                4'd10: roy[2] <= tip;
                4'd11: rox[2] <= tip;
                4'd12: roy[3] <= tip;
                4'd13: rox[3] <= tip;
                default: ;
            endcase
        end
    end

    // ---- Vertex pipeline ----------------------------------------------------
    // The drone: X-frame quadcopter with a gun turret
    //   0-3   motors at (+-41, +-41, 0)          4-7   body top corners (z = +7)
    //   8-11  body bottom corners (z = -7)       12-15 propeller hubs (z = +12)
    //   16-23 propeller tips (2 per rotor)       24-27 turret top corners (z = +16)
    //   28-31 twin barrels (rear, tip) x (left, right); tips reach out when firing
    // The zombies (ground-fixed, not tilted with the drone):
    //   32+2k feet (z = -26), 33+2k head (z = -26 + 32, big +44)
    localparam signed [9:0] A  = 10'sd41, BX = 10'sd13, BY = 10'sd21, BZ = 10'sd7;
    localparam signed [9:0] HZ = 10'sd12;
    localparam signed [9:0] TX = 10'sd8,  TY = 10'sd9,  TZ = 10'sd16;     // turret top
    localparam signed [9:0] GX = 10'sd3,  GR = 10'sd9,  GZ = 10'sd12;     // barrels: x, rear y, z
    localparam signed [9:0] GTIP = 10'sd46, GBEAM = 10'sd170;             // barrel tip / beam end y
    localparam signed [9:0] ZFEET = -10'sd26, ZH = 10'sd32, ZHBIG = 10'sd44;

    function signed [9:0] motor_x(input [1:0] m);
        motor_x = (m == 2'd0 || m == 2'd3) ? A : -A;
    endfunction
    function signed [9:0] motor_y(input [1:0] m);
        motor_y = (m == 2'd0 || m == 2'd1) ? A : -A;
    endfunction

    // V0: point number -> model coordinates (wf = ground point: skip the rotations)
    reg              v0_ok = 1'b0, v0_wf = 1'b0;
    reg [5:0]        v0_i  = 6'd0;
    reg signed [9:0] x0 = 0, y0 = 0, z0 = 0;
    wire [1:0]       tr = vin_i[2:1];             // rotor of a propeller tip (16-23)
    wire [1:0]       zk = vin_i[2:1];             // zombie of a point (32-39)
    wire             zbig_k = a_zbig[zk];
    always @(posedge clk) begin
        v0_ok <= vin_ok;
        v0_i  <= vin_i;
        v0_wf <= (vin_i >= 6'd32);
        if (vin_i < 6'd4) begin                   // motors
            x0 <= motor_x(vin_i[1:0]);
            y0 <= motor_y(vin_i[1:0]);
            z0 <= 10'sd0;
        end else if (vin_i < 6'd12) begin         // body box
            x0 <= (vin_i[1:0] == 2'd0 || vin_i[1:0] == 2'd3) ? BX : -BX;
            y0 <= (vin_i[1:0] == 2'd0 || vin_i[1:0] == 2'd1) ? BY : -BY;
            z0 <= (vin_i < 6'd8) ? BZ : -BZ;
        end else if (vin_i < 6'd16) begin         // hubs
            x0 <= motor_x(vin_i[1:0]);
            y0 <= motor_y(vin_i[1:0]);
            z0 <= HZ;
        end else if (vin_i < 6'd24) begin         // propeller tips: hub +- radius
            x0 <= vin_i[0] ? motor_x(tr) - rox[tr] : motor_x(tr) + rox[tr];
            y0 <= vin_i[0] ? motor_y(tr) - roy[tr] : motor_y(tr) + roy[tr];
            z0 <= HZ;
        end else if (vin_i < 6'd28) begin         // turret top
            x0 <= (vin_i[1:0] == 2'd0 || vin_i[1:0] == 2'd3) ? TX : -TX;
            y0 <= (vin_i[1:0] == 2'd0 || vin_i[1:0] == 2'd1) ? TY : -TY;
            z0 <= TZ;
        end else if (vin_i < 6'd32) begin         // barrels
            x0 <= vin_i[1] ? GX : -GX;
            y0 <= !vin_i[0] ? GR : (a_beam ? GBEAM : GTIP);
            z0 <= GZ;
        end else begin                            // zombie feet / head
            x0 <= pick10(z_wx, zk);
            y0 <= pick10(z_wy, zk);
            z0 <= !vin_i[0] ? ZFEET : (zbig_k ? ZFEET + ZHBIG : ZFEET + ZH);
        end
    end

    // One 10-bit field out of four (a plain selector, not a multiply by 10)
    function signed [9:0] pick10(input [39:0] v, input [1:0] k);
        case (k)
            2'd0:    pick10 = v[9:0];
            2'd1:    pick10 = v[19:10];
            2'd2:    pick10 = v[29:20];
            default: pick10 = v[39:30];
        endcase
    endfunction

    // Rotation operands: ground points use cos = 1 (256), sin = 0
    function signed [9:0] cs(input wf, input signed [9:0] c);
        cs = wf ? 10'sd256 : c;
    endfunction
    function signed [9:0] sn(input wf, input signed [9:0] s);
        sn = wf ? 10'sd0 : s;
    endfunction

    // V1: roll (about X): y' = y cos - z sin, z' = y sin + z cos
    reg              v1_ok = 1'b0, v1_wf = 1'b0;
    reg [5:0]        v1_i  = 6'd0;
    reg signed [9:0] v1_x  = 0;
    reg signed [19:0] m1a = 0, m1b = 0, m1c = 0, m1d = 0;
    always @(posedge clk) begin
        v1_ok <= v0_ok; v1_i <= v0_i; v1_x <= x0; v1_wf <= v0_wf;
        m1a <= y0 * cs(v0_wf, cr); m1b <= z0 * sn(v0_wf, sr);
        m1c <= y0 * sn(v0_wf, sr); m1d <= z0 * cs(v0_wf, cr);
    end
    reg              v2_ok = 1'b0, v2_wf = 1'b0;
    reg [5:0]        v2_i  = 6'd0;
    reg signed [9:0] x1 = 0, y1 = 0, z1 = 0;
    wire signed [19:0] y1w = m1a - m1b;
    wire signed [19:0] z1w = m1c + m1d;
    always @(posedge clk) begin
        v2_ok <= v1_ok; v2_i <= v1_i; v2_wf <= v1_wf;
        x1 <= v1_x; y1 <= (y1w + 20'sd128) >>> 8; z1 <= (z1w + 20'sd128) >>> 8;
    end

    // V2: pitch (about Y): x' = x cos + z sin, z' = -x sin + z cos
    reg              v3_ok = 1'b0, v3_wf = 1'b0;
    reg [5:0]        v3_i  = 6'd0;
    reg signed [9:0] v3_y  = 0;
    reg signed [19:0] m2a = 0, m2b = 0, m2c = 0, m2d = 0;
    always @(posedge clk) begin
        v3_ok <= v2_ok; v3_i <= v2_i; v3_y <= y1; v3_wf <= v2_wf;
        m2a <= x1 * cs(v2_wf, cp); m2b <= z1 * sn(v2_wf, sp);
        m2c <= x1 * sn(v2_wf, sp); m2d <= z1 * cs(v2_wf, cp);
    end
    reg              v4_ok = 1'b0, v4_wf = 1'b0;
    reg [5:0]        v4_i  = 6'd0;
    reg signed [9:0] x2 = 0, y2 = 0, z2 = 0;
    wire signed [19:0] x2w = m2a + m2b;
    wire signed [19:0] z2w = m2d - m2c;
    always @(posedge clk) begin
        v4_ok <= v3_ok; v4_i <= v3_i; v4_wf <= v3_wf;
        x2 <= (x2w + 20'sd128) >>> 8; y2 <= v3_y; z2 <= (z2w + 20'sd128) >>> 8;
    end

    // V3: yaw (about Z): x' = x cos - y sin, y' = x sin + y cos
    reg              v5_ok = 1'b0;
    reg [5:0]        v5_i  = 6'd0;
    reg signed [9:0] v5_z  = 0;
    reg signed [19:0] m3a = 0, m3b = 0, m3c = 0, m3d = 0;
    always @(posedge clk) begin
        v5_ok <= v4_ok; v5_i <= v4_i; v5_z <= z2;
        m3a <= x2 * cs(v4_wf, cy); m3b <= y2 * sn(v4_wf, sy);
        m3c <= x2 * sn(v4_wf, sy); m3d <= y2 * cs(v4_wf, cy);
    end
    reg              v6_ok = 1'b0;
    reg [5:0]        v6_i  = 6'd0;
    reg signed [9:0] x3 = 0, y3 = 0, z3 = 0;
    wire signed [19:0] x3w = m3a - m3b;
    wire signed [19:0] y3w = m3c + m3d;
    always @(posedge clk) begin
        v6_ok <= v5_ok; v6_i <= v5_i;
        x3 <= (x3w + 20'sd128) >>> 8; y3 <= (y3w + 20'sd128) >>> 8; z3 <= v5_z;
    end

    // V4: camera 30 degrees above, behind the drone (cos 30 = 222/256, sin 30 = 1/2)
    //     up = z cos30 + y sin30, depth = y cos30 - z sin30
    // v * 222 + 128 (the + 128 rounds the later / 256; it fills the empty low bits of v << 8)
    function signed [19:0] times222(input signed [9:0] v);
        times222 = (($signed({{10{v[9]}}, v}) <<< 8) | 20'sd128) - ($signed({{10{v[9]}}, v}) <<< 5)
                 - ($signed({{10{v[9]}}, v}) <<< 1);
    endfunction
    reg              v6b_ok = 1'b0;
    reg [5:0]        v6b_i  = 6'd0;
    reg signed [9:0] v6b_x = 0, y3h = 0, z3h = 0;
    reg signed [19:0] z3c = 0, y3c = 0;
    always @(posedge clk) begin
        v6b_ok <= v6_ok; v6b_i <= v6_i; v6b_x <= x3;
        z3c <= times222(z3);
        y3c <= times222(y3);
        y3h <= (y3 + 10'sd1) >>> 1;
        z3h <= (z3 + 10'sd1) >>> 1;
    end
    reg              v7_ok = 1'b0;
    reg [5:0]        v7_i  = 6'd0;
    reg signed [9:0] v7_x = 0, u = 0, w = 0;
    always @(posedge clk) begin
        v7_ok <= v6b_ok; v7_i <= v6b_i; v7_x <= v6b_x;
        u <= (z3c >>> 8) + y3h;
        w <= (y3c >>> 8) - z3h;
    end

    // V5: perspective scale from the depth
    reg              v8_ok = 1'b0;
    reg [5:0]        v8_i  = 6'd0;
    reg signed [9:0] v8_x = 0, v8_u = 0, v8_w = 0;
    reg        [8:0] rec = 9'd256;
    wire       [7:0] w_idx = (w > 10'sd127) ? 8'd127 : (w < -10'sd128) ? 8'd128 : w[7:0];
    always @(posedge clk) begin
        v8_ok <= v7_ok; v8_i <= v7_i; v8_x <= v7_x; v8_u <= u; v8_w <= w;
        rec <= rec_rom[w_idx];
    end

    // Extra register: the table's memory output is slow, keep it away from the multipliers
    reg              v8b_ok = 1'b0;
    reg [5:0]        v8b_i  = 6'd0;
    reg signed [9:0] v8b_x = 0, v8b_u = 0, v8b_w = 0;
    reg        [8:0] rec_r = 9'd256;
    always @(posedge clk) begin
        v8b_ok <= v8_ok; v8b_i <= v8_i; v8b_x <= v8_x; v8b_u <= v8_u; v8b_w <= v8_w;
        rec_r  <= rec;
    end

    // V6: screen position = centre + coordinate * scale
    reg              v9_ok = 1'b0;
    reg [5:0]        v9_i  = 6'd0;
    reg signed [9:0] v9_w  = 0;
    reg signed [19:0] m6a = 0, m6b = 0;
    wire signed [9:0] rec_s = $signed({1'b0, rec_r});
    always @(posedge clk) begin
        v9_ok <= v8b_ok; v9_i <= v8b_i; v9_w <= v8b_w;
        m6a <= v8b_x * rec_s;
        m6b <= v8b_u * rec_s;
    end

    // Point table: screen x, screen y, depth
    reg signed [10:0] vsx [0:63];
    reg signed [10:0] vsy [0:63];
    reg signed [9:0]  vdw [0:63];
    initial for (k = 0; k < 64; k = k + 1) begin vsx[k] = 0; vsy[k] = 0; vdw[k] = 0; end
    wire signed [19:0] sxw = (m6a + 20'sd30848) >>> 8;    // 120 + m6a / 256, rounded
    wire signed [19:0] syw = (20'sd30848 - m6b) >>> 8;    // 120 - m6b / 256, rounded
    always @(posedge clk)
        if (v9_ok) begin
            vsx[v9_i] <= sxw[10:0];
            vsy[v9_i] <= syw[10:0];
            vdw[v9_i] <= v9_w;
        end

    // ---- Edge setup pipeline --------------------------------------------------
    // E0: which two points does line j join? (j 32-35: zombie k's feet and head)
    reg       e0_ok = 1'b0;
    reg [5:0] e0_j = 6'd0, ea = 6'd0, eb = 6'd0;
    always @(posedge clk) begin
        e0_ok <= ein_ok;
        e0_j  <= ein_j;
        case (ein_j)
            6'd0:  begin ea <= 6'd0;  eb <= 6'd2;  end   // arms (diagonals)
            6'd1:  begin ea <= 6'd1;  eb <= 6'd3;  end
            6'd2:  begin ea <= 6'd4;  eb <= 6'd5;  end   // body top
            6'd3:  begin ea <= 6'd5;  eb <= 6'd6;  end
            6'd4:  begin ea <= 6'd6;  eb <= 6'd7;  end
            6'd5:  begin ea <= 6'd7;  eb <= 6'd4;  end
            6'd6:  begin ea <= 6'd8;  eb <= 6'd9;  end   // body bottom
            6'd7:  begin ea <= 6'd9;  eb <= 6'd10; end
            6'd8:  begin ea <= 6'd10; eb <= 6'd11; end
            6'd9:  begin ea <= 6'd11; eb <= 6'd8;  end
            6'd10: begin ea <= 6'd4;  eb <= 6'd8;  end   // body sides
            6'd11: begin ea <= 6'd5;  eb <= 6'd9;  end
            6'd12: begin ea <= 6'd6;  eb <= 6'd10; end
            6'd13: begin ea <= 6'd7;  eb <= 6'd11; end
            6'd14: begin ea <= 6'd0;  eb <= 6'd12; end   // motor posts
            6'd15: begin ea <= 6'd1;  eb <= 6'd13; end
            6'd16: begin ea <= 6'd2;  eb <= 6'd14; end
            6'd17: begin ea <= 6'd3;  eb <= 6'd15; end
            6'd18: begin ea <= 6'd16; eb <= 6'd17; end   // propellers
            6'd19: begin ea <= 6'd18; eb <= 6'd19; end
            6'd20: begin ea <= 6'd20; eb <= 6'd21; end
            6'd21: begin ea <= 6'd22; eb <= 6'd23; end
            6'd22: begin ea <= 6'd24; eb <= 6'd25; end   // turret top
            6'd23: begin ea <= 6'd25; eb <= 6'd26; end
            6'd24: begin ea <= 6'd26; eb <= 6'd27; end
            6'd25: begin ea <= 6'd27; eb <= 6'd24; end
            6'd26: begin ea <= 6'd4;  eb <= 6'd24; end   // turret sides (from the body top)
            6'd27: begin ea <= 6'd5;  eb <= 6'd25; end
            6'd28: begin ea <= 6'd6;  eb <= 6'd26; end
            6'd29: begin ea <= 6'd7;  eb <= 6'd27; end
            6'd30: begin ea <= 6'd28; eb <= 6'd29; end   // barrels (beams when firing)
            6'd31: begin ea <= 6'd30; eb <= 6'd31; end
            6'd32: begin ea <= 6'd32; eb <= 6'd33; end   // zombie columns
            6'd33: begin ea <= 6'd34; eb <= 6'd35; end
            6'd34: begin ea <= 6'd36; eb <= 6'd37; end
            default: begin ea <= 6'd38; eb <= 6'd39; end
        endcase
    end

    // E1: read both points
    reg              e1_ok = 1'b0;
    reg [5:0]        e1_j  = 6'd0;
    reg signed [10:0] ax = 0, ay = 0, bx = 0, by = 0;
    reg signed [9:0]  wa = 0, wb = 0;
    always @(posedge clk) begin
        e1_ok <= e0_ok; e1_j <= e0_j;
        ax <= vsx[ea]; ay <= vsy[ea]; wa <= vdw[ea];
        bx <= vsx[eb]; by <= vsy[eb]; wb <= vdw[eb];
    end

    // E2: line direction
    reg              e2_ok = 1'b0;
    reg [5:0]        e2_j  = 6'd0;
    reg signed [10:0] e2_ax = 0, e2_ay = 0, ldx = 0, ldy = 0;
    reg signed [10:0] wsum = 0;
    always @(posedge clk) begin
        e2_ok <= e1_ok; e2_j <= e1_j;
        e2_ax <= ax; e2_ay <= ay;
        ldx <= bx - ax;
        ldy <= by - ay;
        wsum <= wa + wb;
    end

    // E3: products (6 multipliers)
    reg              e3_ok = 1'b0;
    reg [5:0]        e3_j  = 6'd0;
    reg signed [10:0] e3_dx = 0, e3_dy = 0, e3_ws = 0;
    reg        [10:0] adx = 0, ady = 0;
    reg signed [21:0] p_aydx = 0, p_axdy = 0, p_axdx = 0, p_aydy = 0, p_dxdx = 0, p_dydy = 0;
    always @(posedge clk) begin
        e3_ok <= e2_ok; e3_j <= e2_j;
        e3_dx <= ldx; e3_dy <= ldy; e3_ws <= wsum;
        adx <= ldx[10] ? -ldx : ldx;
        ady <= ldy[10] ? -ldy : ldy;
        p_aydx <= e2_ay * ldx;
        p_axdy <= e2_ax * ldy;
        p_axdx <= e2_ax * ldx;
        p_aydy <= e2_ay * ldy;
        p_dxdx <= ldx * ldx;
        p_dydy <= ldy * ldy;
    end

    // E4: start values at pixel (0, 0), squared length, rough length
    //   E(x,y) = (x-ax)*dy - (y-ay)*dx      D(x,y) = (x-ax)*dx + (y-ay)*dy
    reg              e4_ok = 1'b0;
    reg [5:0]        e4_j  = 6'd0;
    reg signed [10:0] e4_dx = 0, e4_dy = 0, e4_ws = 0;
    reg signed [21:0] ei0 = 0, di0 = 0, l2 = 0;
    reg        [10:0] lmax = 0, lmin = 0;
    always @(posedge clk) begin
        e4_ok <= e3_ok; e4_j <= e3_j;
        e4_dx <= e3_dx; e4_dy <= e3_dy; e4_ws <= e3_ws;
        ei0 <= p_aydx - p_axdy;
        di0 <= -(p_axdx + p_aydy);
        l2  <= p_dxdx + p_dydy;
        lmax <= (adx > ady) ? adx : ady;
        lmin <= (adx > ady) ? ady : adx;
    end

    // E5: length ~ max + 3/8 min. Line half-width 1 pixel (W = length), core half a pixel.
    reg              e5_ok = 1'b0;
    reg [5:0]        e5_j  = 6'd0;
    reg signed [10:0] e5_dx = 0, e5_dy = 0;
    reg signed [21:0] e5_ei = 0, e5_di = 0, e5_l2 = 0;
    reg        [11:0] llen = 0;
    reg        [1:0]  lv = 2'd0;
    always @(posedge clk) begin
        e5_ok <= e4_ok; e5_j <= e4_j;
        e5_dx <= e4_dx; e5_dy <= e4_dy;
        e5_ei <= ei0; e5_di <= di0; e5_l2 <= l2;
        llen  <= {1'b0, lmax} + (lmin >> 2) + (lmin >> 3);
        lv    <= (e4_ws < -11'sd50) ? 2'd3 : (e4_ws < 11'sd0) ? 2'd2 :
                 (e4_ws < 11'sd50)  ? 2'd1 : 2'd0;     // nearer = brighter
    end

    reg              p_ok = 1'b0;
    reg [5:0]        p_j  = 6'd0;
    reg signed [10:0] p_dx = 0, p_dy = 0;
    reg signed [19:0] p_se = 0, p_sd = 0, p_lp = 0;
    reg signed [12:0] p_w = 0, p_nw = 0;
    reg        [1:0]  p_lv = 2'd0;
    always @(posedge clk) begin
        p_ok <= e5_ok; p_j <= e5_j;
        p_dx <= e5_dx; p_dy <= e5_dy;
        p_se <= e5_ei - (e5_dx <<< 4);                // E and D at pixel (0, 16)
        p_sd <= e5_di + (e5_dy <<< 4);
        p_w  <= $signed({1'b0, llen});
        p_nw <= -$signed({1'b0, llen});
        p_lp <= e5_l2 + $signed({10'd0, llen});       // end of the line + half-width
        p_lv <= lv;
    end

    // Zombie column setup, from E2 (a = feet, b = head):
    //   x from feet.x - half-width to + half-width, y from head.y to feet.y,
    //   half-width = height / 16 (big: / 8), head = top quarter
    reg              zs_ok = 1'b0;
    reg [1:0]        zs_k  = 2'd0;
    reg signed [10:0] zs_xl = 0, zs_xr = 0, zs_yt = 0, zs_yb = 0, zs_yh = 0;
    reg        [1:0]  zs_lv = 2'd0;
    wire       [1:0]  e2_k  = e2_j[1:0];
    wire signed [10:0] zh_px = -ldy;                                  // column height in pixels
    wire signed [10:0] zw_px = a_zbig[e2_k] ? ((zh_px >>> 3) + 11'sd1) : ((zh_px >>> 4) + 11'sd1);
    always @(posedge clk) begin
        zs_ok <= e2_ok && (e2_j >= 6'd32);
        zs_k  <= e2_k;
        zs_xl <= e2_ax + (ldx >>> 1) - zw_px;
        zs_xr <= e2_ax + (ldx >>> 1) + zw_px;
        zs_yt <= e2_ay + ldy;
        zs_yb <= e2_ay;
        zs_yh <= e2_ay + ldy + (zh_px >>> 2);
        zs_lv <= (wsum < -11'sd50) ? 2'd3 : (wsum < 11'sd0) ? 2'd2 :
                 (wsum < 11'sd50)  ? 2'd1 : 2'd0;
    end

    // =======================================================================
    // 2. LINE TESTERS - 32 copies, all working on the same pixel at once
    // =======================================================================
    wire [NE-1:0] hits, cores, near3, near2, near1;
    genvar g;
    generate
        for (g = 0; g < NE; g = g + 1) begin : edge_unit
            // This frame's numbers, written by the setup engine during rows 0-15
            reg signed [10:0] dx = 0, dy = 0;
            reg signed [19:0] lp = 0, se = 0, sd = 0;
            reg signed [12:0] wd = 0, nwd = 0;
            reg        [1:0]  lvl = 2'd0;
            wire signed [19:0] wh = {8'd0, wd[12:1]};     // core: half the width
            // Running sums for the current pixel
            reg signed [19:0] E = 0, D = 0, Er = 0, Dr = 0, aE = 0;
            reg               inl = 1'b0, h = 1'b0, c = 1'b0;
            // Own copy of the step signals (keeps each one's fan-out small)
            reg               u_step = 1'b0, u_r16 = 1'b0, u_rs = 1'b0;

            always @(posedge clk) begin
                u_step <= step;
                u_r16  <= r16;
                u_rs   <= rstart;
                if (p_ok && p_j == g) begin
                    dx <= p_dx; dy <= p_dy; lp <= p_lp; lvl <= p_lv;
                    se <= p_se; sd <= p_sd; wd <= p_w; nwd <= p_nw;
                end
                if (u_step) begin
                    if (u_r16) begin                          // row 16: start from the new numbers
                        Er <= se;
                        Dr <= sd;
                        E  <= se;
                        D  <= sd;
                    end else if (u_rs) begin                  // next row: x back to 0
                        Er <= Er - dx;
                        Dr <= Dr + dy;
                        E  <= Er - dx;
                        D  <= Dr + dy;
                    end else begin                            // next pixel to the right
                        E <= E + dy;
                        D <= D + dx;
                    end
                end
                // On the line: close to it sideways (|E|), and between its two ends
                aE  <= E[19] ? -E : E;
                inl <= (D > nwd) && (D < lp);
                h   <= (aE < wd) && inl;
                c   <= (aE < wh) && inl;
            end
            assign hits[g]  = h;
            assign cores[g] = c;
            assign near3[g] = h && (lvl == 2'd3);
            assign near2[g] = h && (lvl == 2'd2);
            assign near1[g] = h && (lvl == 2'd1);
        end
    endgenerate

    // =======================================================================
    // 3. ZOMBIE COLUMN TESTERS - 4 copies
    // =======================================================================
    wire [NZ-1:0] zin, zhd, zbg, zfl;
    wire signed [10:0] sx = {3'b000, px};
    wire signed [10:0] syy = {3'b000, py};
    generate
        for (g = 0; g < NZ; g = g + 1) begin : zombie_unit
            reg signed [10:0] xl = 0, xr = 0, yt = 0, yb = 0, yh = 0;
            reg               zi = 1'b0, zh = 1'b0;
            always @(posedge clk) begin
                if (zs_ok && zs_k == g) begin
                    xl <= zs_xl; xr <= zs_xr; yt <= zs_yt; yb <= zs_yb; yh <= zs_yh;
                end
                zi <= a_zon[g] && (sx >= xl) && (sx <= xr) && (syy >= yt) && (syy <= yb);
                zh <= (syy <= yh);
            end
            assign zin[g] = zi;
            assign zhd[g] = zi && zh;
            assign zbg[g] = zi && a_zbig[g];
            assign zfl[g] = zi && a_zfl[g];
        end
    endgenerate

    // Brightest line wins where lines cross
    always @(posedge clk) begin
        hit     <= |hits;
        core    <= |cores;
        level   <= (|near3) ? 2'd3 : (|near2) ? 2'd2 : (|near1) ? 2'd1 : 2'd0;
        beam_px <= a_beam && (hits[30] || hits[31]);
        zhit    <= |zin;
        zhead   <= |zhd;
        zbig    <= |zbg;
        zflash  <= |zfl;
    end
endmodule
