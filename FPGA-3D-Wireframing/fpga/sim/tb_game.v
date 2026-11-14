`timescale 1ns/1ps
// Zombie game + 3D view: wire3d + zombies + radar_render.
// Game time is driven by hand (the 1 ms tick), so minutes of play take no sim time.
module tb_game;
  reg clk = 0; always #5 clk = ~clk;
  reg [7:0] x = 8'd1, y = 8'd0;
  reg [7:0] roll = 0, pitch = 0, yaw = 0;
  reg [1:0] mode = 1; reg target = 0; reg shot = 0; reg tick = 0; reg [15:0] rnd = 0;
  reg wire_on = 1, beam = 0, blink = 0;
  reg [95:0] line1, line2;

  wire go, hit, core, beam_px, zhit, zhead, zbig, zflash; wire [1:0] level;
  wire [3:0] z_on, z_big, z_flash; wire [39:0] z_wx, z_wy; wire [31:0] z_rx, z_ry;
  wire bite, kill; wire [7:0] kills; wire [15:0] color;

  wire3d u_w(.clk(clk), .x(x), .y(y), .roll(roll), .pitch(pitch), .yaw(yaw), .spin_step(8'd19),
    .beam(beam), .z_on(z_on), .z_big(z_big), .z_flash(z_flash), .z_wx(z_wx), .z_wy(z_wy),
    .frame_go(go), .hit(hit), .core(core), .level(level), .beam_px(beam_px),
    .zhit(zhit), .zhead(zhead), .zbig(zbig), .zflash(zflash));
  zombies u_z(.clk(clk), .tick(tick), .mode(mode), .target(target), .shot(shot), .aim(yaw),
    .rnd(rnd), .frame_go(go), .z_on(z_on), .z_big(z_big), .z_flash(z_flash), .z_wx(z_wx), .z_wy(z_wy),
    .z_rx(z_rx), .z_ry(z_ry), .bite(bite), .kill(kill), .kills(kills));
  radar_render u_r(.clk(clk), .x(x), .y(y), .mode(mode), .link_ok(1'b1),
    .charge_a(9'd154), .target(1'b0), .heading(8'd20), .secs(8'd0),
    .line1(line1), .line2(line2), .sweep(8'd200), .blink(blink), .pulse(1'b1), .beam(beam),
    .noise(16'h1234), .wire_on(wire_on), .w_hit(hit), .w_core(core), .w_level(level), .w_beam(beam_px),
    .w_zhit(zhit), .w_zhead(zhead), .w_zbig(zbig), .w_zflash(zflash),
    .rz_on(z_on), .rz_big(z_big), .rz_flash(z_flash), .rz_x(z_rx), .rz_y(z_ry), .bite(bite), .aim(yaw), .beam_dir(yaw), .warn(1'b0),
    .color(color));

  function [95:0] mk(input [95:0] s); integer i;
    begin for (i = 0; i < 12; i = i + 1) mk[8*i +: 8] = s[8*(11-i) +: 8]; end
  endfunction

  integer f, i, j, n;
  task one_frame(input save); begin
    for (j = 0; j < 240; j = j + 1) for (i = (j == 0 ? 1 : 0); i < 240; i = i + 1) begin
      if (save && j == 0 && i == 1) $fwrite(f, "0000\n");
      x = i; y = j;
      repeat (11) @(posedge clk); #1;
      if (save) $fwrite(f, "%04x\n", color);
    end
    x = 0; y = 240; repeat (12) @(posedge clk); #1;
    x = 1; y = 0;   repeat (2) @(posedge clk); #1;
  end endtask
  // two frames to settle (positions at the last row, shown from the next frame), then save
  task scene(input [8*24-1:0] name); begin
    one_frame(0);
    f = $fopen(name, "w"); one_frame(1); $fclose(f);
  end endtask
  task ms(input integer t); integer q; begin
    for (q = 0; q < t; q = q + 1) begin tick = 1; @(posedge clk); #1; tick = 0; repeat (3) @(posedge clk); #1; end
  end endtask
  task spawn(input [7:0] a, input big_one); begin
    rnd = {6'd0, big_one ? 2'b00 : 2'b01, a};
    target = 1; ms(2); target = 0; ms(2);
  end endtask
  task fire; begin
    shot = 1; @(posedge clk); #1; shot = 0; repeat (10) @(posedge clk); #1;
  end endtask
  task status; integer q; begin
    for (q = 0; q < 4; q = q + 1)
      $display("  slot %0d: alive=%0d dying=%0d big=%0d hp=%0d angle=%0d dist=%0d", q,
        u_z.alive[q], u_z.dying[q], u_z.big[q], u_z.hp[q], u_z.ang[q], u_z.dist[q]);
    $display("  kills=%0d bite=%0d", kills, bite);
  end endtask

  initial begin
    line1 = mk("   BATTLE   "); line2 = mk(" HITS 0  60%");
    one_frame(0);
    // Four zombies: ahead (0), left big (64), behind-right (160), right (210)
    spawn(8'd0, 0); ms(300); spawn(8'd64, 1); ms(300); spawn(8'd160, 0); ms(300); spawn(8'd210, 0);
    ms(600);
    $display("after spawning 4:"); status;
    scene("g1_3d.hex");
    wire_on = 0; scene("g2_radar.hex"); wire_on = 1;
    // Shoot straight ahead: zombie 0 dies
    beam = 1; fire; $display("shot at aim 0:"); status;
    beam = 0;
    // Turn to the big one (64) and shoot 3 times
    yaw = 8'd60; fire; ms(200); $display("big one, hit 1:"); status;
    fire; ms(200); fire; $display("big one, hit 3:"); status;
    beam = 1; wire_on = 0; scene("g4_radar_turned.hex"); wire_on = 1; beam = 0;
    // Shooting at nothing changes nothing
    yaw = 8'd100; fire; $display("miss at aim 100:"); status;
    // Let the rest walk in: a bite
    ms(6000); $display("after 6 s more:"); status;
    // Standby clears the field
    mode = 0; ms(2); $display("standby:"); status;
    $finish;
  end
endmodule
