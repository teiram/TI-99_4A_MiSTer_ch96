`timescale 1ns/1ps

// tb_psram_bridge -- TI psram_bridge + real APS6404L behavioural model.
//
// Exercises the host-facing SDRAM-compatible port with the same kinds of
// accesses the TI core generates: word and byte accesses, loader-style
// back-to-back writes, NVRAM save/load addresses, erase-style clearing and
// large-cart addresses. Calibration runs through the DPS phase-step handshake.
//
// Build/run (from TI-99_4A_MiSTer):
//   iverilog -g2012 -o sim/vvp/tb_psram_bridge.vvp \
//        rtl/psram.sv rtl/psram_cal3.sv rtl/psram_dps.sv rtl/psram_bridge.sv \
//        sim/aps6404l_model.sv sim/tb_psram_bridge.sv
//   vvp sim/vvp/tb_psram_bridge.vvp

`ifndef T_NS
  `define T_NS 7.7601423198       // 128.863616 MHz
`endif
`ifndef T_REF_NS
  `define T_REF_NS 23.2804267786  // 42.954539 MHz
`endif
`ifndef PHI0_NS
  `define PHI0_NS 1.16
`endif
`ifndef STEP_NS
  `define STEP_NS 0.1940035580    // VCO/8 for 644.318080 MHz VCO
`endif
`ifndef DSKEW_NS
  `define DSKEW_NS 0.3
`endif
`ifndef DFWD_NS
  `define DFWD_NS 0.4
`endif
`ifndef DBACK_NS
  `define DBACK_NS 0.4
`endif

module pll_dps_model #(
    parameter real  PHI0 = `PHI0_NS,
    parameter real  STEP = `STEP_NS,
    parameter [4:0] CNT  = 5'd2
) (
    input  wire       clk,
    input  wire       scanclk,
    input  wire       phase_en,
    input  wire       updn,
    input  wire [4:0] cntsel,
    output reg        phase_done,
    output wire       clk_cap
);
    real phi;
    reg cap_r;
    initial begin phi = PHI0; phase_done = 1'b1; cap_r = 1'b0; end
    always @(clk) cap_r <= #(phi) clk;
    assign clk_cap = cap_r;

    always @(posedge phase_en) begin
        @(posedge scanclk);
        phase_done = 1'b0;
        if (cntsel == CNT) phi = phi + (updn ? STEP : -STEP);
        #25.0;
        phase_done = 1'b1;
    end
endmodule

module ps_pads #(
    parameter real      DSKEW = `DSKEW_NS,
    parameter real      TACLK = 3.5,
    parameter integer   DEPTH = 1 << 23
) (
    input  wire       sclk_f,
    input  wire       ce_n_f,
    input  wire [3:0] sio_out_f,
    input  wire [3:0] sio_oe_f,
    output wire [3:0] sio_in_b
);
    reg        sclk_d  = 1'b0;
    reg        ce_d    = 1'b1;
    reg [3:0]  sio_fwd = 4'bzzzz;
    tri  [3:0] sio_dev;
    reg [3:0]  sio_back = 4'h0;

    always @(sclk_f)                sclk_d  <= #(`DFWD_NS) sclk_f;
    always @(ce_n_f)                ce_d    <= #(`DFWD_NS + DSKEW) ce_n_f;
    always @(sio_out_f or sio_oe_f) sio_fwd <= #(`DFWD_NS + DSKEW) ((sio_oe_f != 4'h0) ? sio_out_f : 4'bzzzz);
    assign sio_dev = sio_fwd;
    always @(sio_dev)               sio_back <= #(`DBACK_NS) sio_dev;
    assign sio_in_b = sio_back;

    aps6404l_model #(
        .DEPTH(DEPTH), .TACLK_NS(TACLK), .TOH_NS(0.0), .WAIT_EB(5)
    ) u_mem (.sclk(sclk_d), .ce_n(ce_d), .sio(sio_dev));
endmodule

module tb_psram_bridge;
    reg clk     = 1'b0;
    reg clk_ref = 1'b0;
    reg clk_50  = 1'b0;
    always #(`T_NS/2)     clk     = ~clk;
    always #(`T_REF_NS/2) clk_ref = ~clk_ref;
    always #10.0          clk_50  = ~clk_50;

    wire clk_cap;
    reg  init = 1'b1;

    reg  [25:0] addr_r  = 26'd0;
    reg  [15:0] din_r   = 16'd0;
    reg  [1:0]  wtbt_r  = 2'b11;
    reg         rd_r    = 1'b0;
    reg         we_r    = 1'b0;
    wire [15:0] dout;
    wire        ready;
    wire        busy;

    wire pll_pen, pll_upd, pll_phdone;
    wire [4:0] pll_cntsel;
    wire cal_done, cal_fail, cal_busy;
    wire [63:0] cal_status;
    wire cal_ready;

    wire p1_sclk, p1_ce_n;
    wire [3:0] p1_sio_out, p1_sio_oe, p1_sio_in;
    wire p2_sclk, p2_ce_n;
    wire [3:0] p2_sio_out, p2_sio_oe;
    wire [3:0] p2_sio_in = 4'h0;

    integer errors = 0;

`ifdef VCD
    initial begin
        $dumpfile("sim/vcd/tb_psram_bridge.vcd");
        $dumpvars(0, tb_psram_bridge);
    end
`endif

    pll_dps_model #(
        .PHI0(`PHI0_NS), .STEP(`STEP_NS), .CNT(5'd2)
    ) u_pll (
        .clk(clk), .scanclk(clk_50),
        .phase_en(pll_pen), .updn(pll_upd), .cntsel(pll_cntsel),
        .phase_done(pll_phdone), .clk_cap(clk_cap)
    );

    psram_bridge #(
        .S(20),
        .CAL_BASE(23'h700000),
        .PWRUP_CYCLES(2000),
        .RST_CYCLES(400),
        .MID_CYCLES(20),
        .CAL_WORDS(4),
        .CAL_PASSES(1)
    ) dut (
        .init(init),
        .clk(clk),
        .clk_ref(clk_ref),
        .clk_cap(clk_cap),
        .scanclk(clk_50),

        .pll_phase_done(pll_phdone),
        .pll_phase_en(pll_pen),
        .pll_updn(pll_upd),
        .pll_cntsel(pll_cntsel),

        .cal_done(cal_done),
        .cal_fail(cal_fail),
        .cal_busy(cal_busy),
        .cal_status(cal_status),
        .cal_ready(cal_ready),

        .PSRAM_SCLK(p1_sclk),
        .PSRAM_CE_N(p1_ce_n),
        .PSRAM_SIO_OUT(p1_sio_out),
        .PSRAM_SIO_OE(p1_sio_oe),
        .PSRAM_SIO_IN(p1_sio_in),

        .PSRAM2_SCLK(p2_sclk),
        .PSRAM2_CE_N(p2_ce_n),
        .PSRAM2_SIO_OUT(p2_sio_out),
        .PSRAM2_SIO_OE(p2_sio_oe),
        .PSRAM2_SIO_IN(p2_sio_in),

        .addr(addr_r),
        .din(din_r),
        .dout(dout),
        .wtbt(wtbt_r),
        .rd(rd_r),
        .we(we_r),
        .ready(ready),
        .busy(busy)
    );

    ps_pads u_pads1 (
        .sclk_f(p1_sclk), .ce_n_f(p1_ce_n),
        .sio_out_f(p1_sio_out), .sio_oe_f(p1_sio_oe), .sio_in_b(p1_sio_in)
    );

    task automatic wait_ready;
        begin
            repeat (200000) begin
                @(posedge clk_ref);
                if (ready) disable wait_ready;
            end
            $display("FAIL: timeout waiting for ready at %0t", $time);
            errors = errors + 1;
        end
    endtask

    task automatic host_access(
        input  logic         we_i,
        input  logic [1:0]   wtbt_i,
        input  logic [25:0]  addr_i,
        input  logic [15:0]  din_i,
        output logic [15:0]  dout_o
    );
        begin
            wait_ready();
            @(negedge clk_ref);
            rd_r = 1'b0;
            we_r = 1'b0;
            wtbt_r = wtbt_i;
            addr_r = addr_i;
            din_r = din_i;
            @(posedge clk_ref);          // old_rd/old_we see the deasserted level
            @(negedge clk_ref);
            if (we_i) we_r = 1'b1;
            else      rd_r = 1'b1;
            @(posedge clk_ref);          // bridge latches the rising edge
            @(negedge clk_ref);
            rd_r = 1'b0;
            we_r = 1'b0;
            wait_ready();
            dout_o = dout;
        end
    endtask

    task automatic check_word(
        input logic [25:0] addr_i,
        input logic [15:0] expected_i,
        input logic [15:0] din_i,
        input logic [7:0]  tag_i
    );
        logic [15:0] got;
        begin
            host_access(1'b1, 2'b11, addr_i, din_i, got);
            host_access(1'b0, 2'b11, addr_i, 16'h0000, got);
            if (got !== expected_i) begin
                $display("FAIL %0d addr=%h expected=%h got=%h", tag_i, addr_i, expected_i, got);
                errors = errors + 1;
            end else begin
                $display("PASS %0d addr=%h data=%h", tag_i, addr_i, got);
            end
        end
    endtask

    initial begin : main
        logic [15:0] got;

        repeat (40) @(negedge clk_ref);
        init = 1'b0;

        begin
            integer timeout;
            timeout = 0;
            while (!cal_ready && timeout < 200000) begin
                @(posedge clk_ref);
                timeout = timeout + 1;
            end
        end
        if (!cal_ready) begin
            $display("FAIL: calibration did not complete");
            errors = errors + 1;
        end else begin
            $display("calibration done: fail=%b busy=%b status=%h", cal_fail, cal_busy, cal_status);
        end
        if (cal_fail) begin
            $display("FAIL: calibration reported failure");
            errors = errors + 1;
        end
        if (p2_sclk !== 1'b0 || p2_ce_n !== 1'b1 || |p2_sio_oe) begin
            $display("FAIL: PSRAM2 not idle");
            errors = errors + 1;
        end

        // Word access (16-bit CPU / loader).
        check_word(26'h000100, 16'hA55A, 16'hA55A, 8'd1);

        // Byte-lane write then byte-lane read (video byte fetch path).
        host_access(1'b1, 2'b10, 26'h000102, 16'h00CD, got);
        host_access(1'b0, 2'b10, 26'h000102, 16'h0000, got);
        if (got[7:0] !== 8'hCD) begin
            $display("FAIL 2 byte lane expected=cd got=%h", got);
            errors = errors + 1;
        end else $display("PASS 2 byte lane data=%h", got);

        // Loader-style consecutive writes: wait for ready between each one.
        for (int i = 0; i < 8; i++) begin
            host_access(1'b1, 2'b11, 26'h000200 + (i << 1), 16'hB000 + i[15:0], got);
        end
        for (int i = 0; i < 8; i++) begin
            host_access(1'b0, 2'b11, 26'h000200 + (i << 1), 16'h0000, got);
            if (got !== (16'hB000 + i[15:0])) begin
                $display("FAIL 3 loader word %0d got=%h", i, got);
                errors = errors + 1;
            end
        end
        $display("PASS 3 loader stream");

        // NVRAM scratch-pad range (~1xxx in the core's map).
        check_word(26'h001200, 16'h55AA, 16'h55AA, 8'd4);

        // Erase-style zero fill.
        for (int i = 0; i < 8; i++) begin
            host_access(1'b1, 2'b11, 26'h002000 + (i << 1), 16'h0000, got);
        end
        for (int i = 0; i < 8; i++) begin
            host_access(1'b0, 2'b11, 26'h002000 + (i << 1), 16'h0000, got);
            if (got !== 16'h0000) begin
                $display("FAIL 5 erase word %0d got=%h", i, got);
                errors = errors + 1;
            end
        end
        $display("PASS 5 erase fill");

        // Large cart / SAMS window: bits above 23 are discarded by the bridge.
        check_word(26'h7FFFFE, 16'hCAFE, 16'hCAFE, 8'd6);

        // Byte enable 2'b00 selects addr[0]; verify low-byte lane behaviour.
        host_access(1'b1, 2'b00, 26'h000104, 16'h007E, got);
        host_access(1'b0, 2'b00, 26'h000104, 16'h0000, got);
        if (got[7:0] !== 8'h7E) begin
            $display("FAIL 7 byte addr0 got=%h", got);
            errors = errors + 1;
        end else $display("PASS 7 byte addr0 data=%h", got);

        repeat (20) @(posedge clk_ref);
        $display("RESULT: %0d errors", errors);
        $finish;
    end
endmodule
