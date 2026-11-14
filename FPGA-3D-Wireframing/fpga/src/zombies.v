// zombies.v - the zombie game, all in FPGA logic.
//
//  - The drone's IR sensor sees something (flag in the status packet) -> a zombie
//    appears at a random angle on the edge of the screen. While the IR keeps
//    seeing it, another one comes every 4 s. Up to 4 at once; 1 in 4 is a big one.
//  - Zombies appear far out (150 units) and walk towards the drone: small ones
//    take ~11 s, big ones ~25 s.
//    At distance 60 it counts as 'approaching'. One that reaches the drone bites
//    (red flash, growl) and is gone; the drone loses 1 health (big: 2) and at 0 it
//    goes into rescue mode. Hits, kills, misses, approaches and bites are counted
//    and sent back to the drone (see spi_status_rx.v).
//  - Aim = the drone's yaw (turn the drone, or left/right on the FPGA buttons).
//    When the laser fires, the nearest zombie within +-17 degrees of the aim is hit:
//    a small one dies, a big one needs 3 hits.
//  - Only in BATTLE. STANDBY / LOST clear the field; ATTACKED freezes it.
//
// Once per frame it also works out where each zombie is drawn:
//   radar (fixed, forward = up, 2/3 scale): x = 120 - 2/3 d sin(a), y = 120 - 2/3 d cos(a)
//   3D ground position:                     x = -d sin(a), y = d cos(a)
// with one table and one multiplier, 16 products, one per clock.
module zombies (
    input  wire        clk,
    input  wire        tick,          // 1 ms
    input  wire [1:0]  mode,          // 0 standby, 1 battle, 2 attacked, 3 lost
    input  wire        target,        // IR sensor sees something
    input  wire        shot,          // one-clock pulse when the laser fires
    input  wire [7:0]  aim,           // drone yaw (0..255 = 360 degrees)
    input  wire [15:0] rnd,           // random bits
    input  wire        frame_go,      // once per frame: update the drawing positions
    output wire [3:0]  z_on,          // slot in use (walking or dying)
    output wire [3:0]  z_big,
    output wire [3:0]  z_flash,       // just hit or dying: drawn white
    output reg  [39:0] z_wx = 40'd0,  // 3D ground positions, 4 x signed 10-bit
    output reg  [39:0] z_wy = 40'd0,
    output reg  [31:0] z_rx = 32'd0,  // radar positions, 4 x 8-bit
    output reg  [31:0] z_ry = 32'd0,
    output reg         bite = 1'b0,   // a zombie reached the drone (lasts 300 ms)
    output reg         kill = 1'b0,   // a zombie died (lasts 150 ms)
    output reg  [7:0]  kills = 8'd0,
    // event counters for the drone (each counts up, wraps at 255)
    output reg  [7:0]  hits = 8'd0,    // shots that hit a zombie (kill or damage)
    output reg  [7:0]  misses = 8'd0,  // shots with no zombie in the aim
    output reg  [7:0]  nears = 8'd0,   // zombies that came close (distance 60)
    output reg  [7:0]  bites = 8'd0,   // small zombies that reached the drone (-1 health)
    output reg  [7:0]  bigbites = 8'd0 // big zombies that reached the drone (-2 health)
);
    localparam [1:0] STANDBY = 2'd0, BATTLE = 2'd1, ATTACKED = 2'd2, LOST = 2'd3;
    localparam [7:0] SPAWN_D = 8'd150;    // start distance (radar: just inside the bezel)
    localparam [7:0] BITE_D  = 8'd22;     // reaches the drone
    localparam [7:0] NEAR_D  = 8'd60;     // 'approaching' warning at this distance
    localparam [7:0] STEP_MS = 8'd90;     // ms per step: small zombie
    localparam [7:0] STEP_BIG = 8'd200;   //              big zombie
    localparam [7:0] AIM_TOL = 8'd12;     // +-12/256 of a turn = +-17 degrees

    // ---- Slots ---------------------------------------------------------------
    reg [3:0] alive = 4'd0;              // walking
    reg [3:0] dying = 4'd0;              // shown white for a moment, then gone
    reg [3:0] big   = 4'd0;
    reg [1:0] hp   [0:3];
    reg [7:0] ang  [0:3];
    reg [7:0] dist [0:3];
    reg [7:0] walk [0:3];                // ms until the next step inwards
    reg [7:0] fl   [0:3];                // flash time left, ms
    integer i;
    initial for (i = 0; i < 4; i = i + 1) begin
        hp[i] = 2'd0; ang[i] = 8'd0; dist[i] = 8'd0; walk[i] = 8'd0; fl[i] = 8'd0;
    end

    assign z_on  = alive | dying;
    assign z_big = big;
    assign z_flash = {fl[3] != 8'd0, fl[2] != 8'd0, fl[1] != 8'd0, fl[0] != 8'd0};

    // ---- Spawning: IR rising edge, then every 4 s while it stays on ------------
    reg        tgt_prev = 1'b0;
    reg [11:0] respawn_ms = 12'd0;
    wire       spawn_now = target && (!tgt_prev || respawn_ms == 12'd0);
    wire [1:0] free = !z_on[0] ? 2'd0 : !z_on[1] ? 2'd1 : !z_on[2] ? 2'd2 : 2'd3;
    wire       has_free = (z_on != 4'b1111);

    // ---- Shooting: check the 4 slots one per clock, nearest in the aim wins ----
    reg        aiming = 1'b0;
    reg [2:0]  ai = 3'd0;
    reg [1:0]  best = 2'd0;
    reg        found = 1'b0;
    reg [7:0]  best_d = 8'd255;
    wire [1:0] ak = ai[1:0];
    wire [7:0] off = ang[ak] - aim;                          // wraps around
    wire       in_aim = (off <= AIM_TOL) || (off >= 8'd0 - AIM_TOL);

    reg [8:0] bite_ms = 9'd0, kill_ms = 9'd0;

    always @(posedge clk) begin
        bite <= (bite_ms != 9'd0);
        kill <= (kill_ms != 9'd0);

        if (mode == STANDBY || mode == LOST) begin
            alive <= 4'd0; dying <= 4'd0;
            for (i = 0; i < 4; i = i + 1) fl[i] <= 8'd0;
            aiming <= 1'b0;
            tgt_prev <= 1'b0;
        end else begin
            if (tick) begin
                if (bite_ms != 9'd0) bite_ms <= bite_ms - 9'd1;
                if (kill_ms != 9'd0) kill_ms <= kill_ms - 9'd1;
                for (i = 0; i < 4; i = i + 1) begin
                    if (fl[i] != 8'd0) begin
                        fl[i] <= fl[i] - 8'd1;
                        if (fl[i] == 8'd1) dying[i] <= 1'b0;   // flash over: slot free
                    end
                end
                if (mode == BATTLE) begin
                    tgt_prev   <= target;
                    respawn_ms <= (spawn_now || respawn_ms == 12'd0) ? 12'd4000 : respawn_ms - 12'd1;
                    // Walk inwards; bite on arrival
                    for (i = 0; i < 4; i = i + 1)
                        if (alive[i]) begin
                            if (walk[i] != 8'd0)
                                walk[i] <= walk[i] - 8'd1;
                            else begin
                                walk[i] <= big[i] ? STEP_BIG : STEP_MS;
                                if (dist[i] <= BITE_D) begin
                                    alive[i] <= 1'b0;
                                    bite_ms  <= 9'd300;
                                    if (big[i]) bigbites <= bigbites + 8'd1;
                                    else        bites    <= bites + 8'd1;
                                end else begin
                                    dist[i] <= dist[i] - 8'd1;
                                    if (dist[i] == NEAR_D + 8'd1)
                                        nears <= nears + 8'd1;
                                end
                            end
                        end
                    // New zombie
                    if (spawn_now && has_free) begin
                        alive[free] <= 1'b1;
                        big[free]   <= (rnd[9:8] == 2'b00);
                        hp[free]    <= (rnd[9:8] == 2'b00) ? 2'd3 : 2'd1;
                        ang[free]   <= rnd[7:0];
                        dist[free]  <= SPAWN_D;
                        walk[free]  <= STEP_MS;
                    end
                end
            end

            // (after the timers, so a hit is never overwritten by a timer step)
            // Laser fired: start the aim check
            if (shot && mode == BATTLE && !aiming) begin
                aiming <= 1'b1; ai <= 3'd0; found <= 1'b0; best_d <= 8'd255;
            end else if (aiming) begin
                if (ai[2]) begin                              // all 4 checked: apply the hit
                    aiming <= 1'b0;
                    if (!found)
                        misses <= misses + 8'd1;
                    if (found) begin
                        hits <= hits + 8'd1;
                        if (hp[best] <= 2'd1) begin
                            alive[best] <= 1'b0;
                            dying[best] <= 1'b1;
                            fl[best]    <= 8'd250;            // dies in a white flash
                            kills       <= kills + 8'd1;
                            kill_ms     <= 9'd150;
                        end else begin
                            hp[best] <= hp[best] - 2'd1;
                            fl[best] <= 8'd120;               // big one: hit flash
                        end
                    end
                end else begin
                    if (alive[ak] && in_aim && dist[ak] < best_d) begin
                        found <= 1'b1; best <= ak; best_d <= dist[ak];
                    end
                    ai <= ai + 3'd1;
                end
            end

        end
    end

    // ---- Drawing positions, once per frame: 16 products with one multiplier ----
    reg signed [9:0] sin_rom [0:255];
    initial $readmemh("sin_rom.mem", sin_rom);

    reg        run = 1'b0;
    reg [4:0]  s = 5'd0;                  // step: zombie s[3:2], product s[1:0]
    wire [1:0] sk = s[3:2];
    wire [7:0] saddr = s[0] ? ang[sk] + 8'd64 : ang[sk];          // sin(a), then cos(a)
    wire [7:0] dk = dist[sk];
    wire [7:0] drad = (dk >> 1) + (dk >> 3) + (dk >> 5);           // radar scale ~2/3

    // pipeline: table read -> register -> multiply -> store
    reg signed [9:0]  t_q = 0, t_r = 0;
    reg        [4:0]  s1 = 0, s2 = 0, s3 = 0;
    reg               v1 = 0, v2 = 0, v3 = 0;
    reg        [7:0]  d1 = 0, d2 = 0;
    reg signed [18:0] prod = 0;
    reg signed [10:0] pv = 0;                             // d * trig / 256, rounded
    reg        [4:0]  s4 = 0;
    reg               v4 = 0;
    reg        [7:0]  o8r = 0, o8y = 0;
    reg        [9:0]  o10 = 0, o10y = 0;
    reg        [4:0]  s5 = 0;
    reg               v5 = 0;
    always @(posedge clk) begin
        if (frame_go) begin run <= 1'b1; s <= 5'd0; end
        else if (run) begin
            s <= s + 5'd1;
            if (s == 5'd15) run <= 1'b0;
        end
        t_q <= sin_rom[saddr]; s1 <= s; v1 <= run; d1 <= s[1] ? dk : drad;
        t_r <= t_q;            s2 <= s1; v2 <= v1; d2 <= d1;
        prod <= $signed({1'b0, d2}) * t_r; s3 <= s2; v3 <= v2;
        pv   <= (prod + 19'sd128) >>> 8;  s4 <= s3; v4 <= v3;
        // value to store, computed one clock ahead
        o8r <= 8'd120 - pv[7:0];
        o8y <= 8'd120 - pv[7:0];
        o10 <= -pv[9:0];
        o10y <= pv[9:0];
        s5 <= s4; v5 <= v4;
        if (v5)
            case (s5[3:0])          // zombie s5[3:2], value s5[1:0]: plain selectors
                4'd0:  z_rx[7:0]   <= o8r;  4'd1:  z_ry[7:0]   <= o8y;
                4'd2:  z_wx[9:0]   <= o10;  4'd3:  z_wy[9:0]   <= o10y;
                4'd4:  z_rx[15:8]  <= o8r;  4'd5:  z_ry[15:8]  <= o8y;
                4'd6:  z_wx[19:10] <= o10;  4'd7:  z_wy[19:10] <= o10y;
                4'd8:  z_rx[23:16] <= o8r;  4'd9:  z_ry[23:16] <= o8y;
                4'd10: z_wx[29:20] <= o10;  4'd11: z_wy[29:20] <= o10y;
                4'd12: z_rx[31:24] <= o8r;  4'd13: z_ry[31:24] <= o8y;
                4'd14: z_wx[39:30] <= o10;  default: z_wy[39:30] <= o10y;
            endcase
    end
endmodule
