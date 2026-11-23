`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Testbench   : tb_stft_istft_axi
// Description : System-level test of vrm_stft_axi and vrm_istft_axi (v2), modelled
//               on the intended block design (two independent AXI DMA paths,
//               host mask = identity in this test).
//
//   T1  register map: ID (v2), defaults, shadow/commit, invalid FFT_N, W1C,
//       watchdog clamp, unmapped read, HOP/BATCH defaults, invalid HOP/BATCH
//       (atomic geometry commit)
//   T2  streaming N=2048: audio -> STFT -> (identity) -> iSTFT -> audio,
//       random tvalid gaps and random tready on all four stream ports,
//       concurrent AXI-Lite polling of both IPs,
//       tlast position checks, counter checks; CSV for bit-exact check
//   T3  iSTFT watchdog: partial frame -> trip -> flush -> recovery
//       (first hop after recovery must equal the first hop of T2)
//   T4  STFT watchdog: output never accepted -> trip -> flush
//   T5  iSTFT input boundary saturation -> SAT_CNT / STATUS[2]
//   T6  reconfiguration to N=256 via commit, streaming, CSV
//   T7  N=2048, hop N/4, batch 1 (sine)
//   T8  N=2048, hop N/4, batch 4 (noise): packet tlast every 4 frames / hops
//   T9  N=1024, hop N/8, batch 3 (sine)
//   T10 iSTFT batch 2 with tlast on every frame: STATUS[3] set, data intact
//
// Packet rule (same as the host / DMA): every stream run uses a whole number
// of packets, NF = multiple of BATCH, and exactly (NF-1)*H + N input samples,
// so the last beat of every run carries tlast (no frame is left without
// tlast, which would hang an AXI DMA S2MM transfer).
//
// Stream checkers (DUT outputs): payload stable while stalled,
// tdata/tlast zero while tvalid = 0. Masters in this TB assert
// tvalid independently of tready.
//
// Plusargs : +NS_MAIN=<n> (default 12000) +NS_ALT=<n> (default 6000)
// Files    : stim_sine.mem, stim_noise.mem ({R,L} per line, $readmemh),
//            window_2048_q17.mem, twiddle_2048_q17.mem
// Outputs  : axi_spec_<tag>.csv, axi_out_<tag>.csv for tags n2048, n256,
//            n2048_h4, n2048_h4_b4, n1024_h8_b3
// ============================================================================

module tb_stft_istft_axi;

    parameter string WIN_FILE  = "window_2048_q17.mem";
    parameter string TWID_FILE = "twiddle_2048_q17.mem";
    parameter string STIM_MAIN = "stim_sine.mem";
    parameter string STIM_ALT  = "stim_noise.mem";

    localparam int AW       = 6;
    localparam int MAX_N    = 2048;
    localparam int SPEC_MAX = 1 << 17;
    localparam int STIM_MAX = 1 << 16;

    localparam logic [AW-1:0] A_CTRL = 6'h00, A_STATUS = 6'h04, A_FFTN = 6'h08, A_FFTN_ACT = 6'h0C,
                              A_WDOG = 6'h10, A_WDOG_ACT = 6'h14, A_FRAME = 6'h18, A_SAT = 6'h1C,
                              A_WDCNT = 6'h20, A_BUSYC = 6'h24, A_ID = 6'h28, A_HOP = 6'h2C,
                              A_HOP_ACT = 6'h30, A_BATCH = 6'h34, A_BATCH_ACT = 6'h38, A_NONE = 6'h3C;
    localparam logic [31:0] DEF_WDOG = 32'h0800_0000;

    // ------------------------------------------------------------------ clock / reset
    logic clk = 1'b0, aresetn = 1'b0;
    always #5 clk = ~clk;                      // 100 MHz
    longint cycle = 0;
    always @(posedge clk) cycle <= cycle + 1;

    // ------------------------------------------------------------------ AXI-Lite wires
    logic [AW-1:0] a_awaddr, a_araddr, b_awaddr, b_araddr;
    logic a_awvalid, a_awready, a_wvalid, a_wready, a_bvalid, a_bready, a_arvalid, a_arready, a_rvalid, a_rready;
    logic b_awvalid, b_awready, b_wvalid, b_wready, b_bvalid, b_bready, b_arvalid, b_arready, b_rvalid, b_rready;
    logic [31:0] a_wdata, a_rdata, b_wdata, b_rdata;
    logic [3:0]  a_wstrb, b_wstrb;
    logic [1:0]  a_bresp, a_rresp, b_bresp, b_rresp;

    // ------------------------------------------------------------------ stream wires
    logic [31:0] a_s_tdata = '0;  logic a_s_tvalid = 1'b0; logic a_s_tready;
    logic [63:0] a_m_tdata;       logic a_m_tvalid, a_m_tlast; logic a_m_tready = 1'b0;
    logic [63:0] b_s_tdata = '0;  logic b_s_tvalid = 1'b0, b_s_tlast = 1'b0; logic b_s_tready;
    logic [31:0] b_m_tdata;       logic b_m_tvalid, b_m_tlast; logic b_m_tready = 1'b0;

    // ------------------------------------------------------------------ DUTs
    vrm_stft_axi #(.WIN_FILE(WIN_FILE), .TWID_FILE(TWID_FILE)) u_stft (
        .aclk(clk), .aresetn(aresetn),
        .s_axi_awaddr(a_awaddr), .s_axi_awvalid(a_awvalid), .s_axi_awready(a_awready),
        .s_axi_wdata(a_wdata), .s_axi_wstrb(a_wstrb), .s_axi_wvalid(a_wvalid), .s_axi_wready(a_wready),
        .s_axi_bresp(a_bresp), .s_axi_bvalid(a_bvalid), .s_axi_bready(a_bready),
        .s_axi_araddr(a_araddr), .s_axi_arvalid(a_arvalid), .s_axi_arready(a_arready),
        .s_axi_rdata(a_rdata), .s_axi_rresp(a_rresp), .s_axi_rvalid(a_rvalid), .s_axi_rready(a_rready),
        .s_axis_tdata(a_s_tdata), .s_axis_tlast(1'b0), .s_axis_tvalid(a_s_tvalid), .s_axis_tready(a_s_tready),
        .m_axis_tdata(a_m_tdata), .m_axis_tlast(a_m_tlast), .m_axis_tvalid(a_m_tvalid), .m_axis_tready(a_m_tready)
    );

    vrm_istft_axi #(.WIN_FILE(WIN_FILE), .TWID_FILE(TWID_FILE)) u_istft (
        .aclk(clk), .aresetn(aresetn),
        .s_axi_awaddr(b_awaddr), .s_axi_awvalid(b_awvalid), .s_axi_awready(b_awready),
        .s_axi_wdata(b_wdata), .s_axi_wstrb(b_wstrb), .s_axi_wvalid(b_wvalid), .s_axi_wready(b_wready),
        .s_axi_bresp(b_bresp), .s_axi_bvalid(b_bvalid), .s_axi_bready(b_bready),
        .s_axi_araddr(b_araddr), .s_axi_arvalid(b_arvalid), .s_axi_arready(b_arready),
        .s_axi_rdata(b_rdata), .s_axi_rresp(b_rresp), .s_axi_rvalid(b_rvalid), .s_axi_rready(b_rready),
        .s_axis_tdata(b_s_tdata), .s_axis_tlast(b_s_tlast), .s_axis_tvalid(b_s_tvalid), .s_axis_tready(b_s_tready),
        .m_axis_tdata(b_m_tdata), .m_axis_tlast(b_m_tlast), .m_axis_tvalid(b_m_tvalid), .m_axis_tready(b_m_tready)
    );

    tb_axil_master #(.AW(AW)) bfm_a (
        .clk(clk), .awaddr(a_awaddr), .awvalid(a_awvalid), .awready(a_awready),
        .wdata(a_wdata), .wstrb(a_wstrb), .wvalid(a_wvalid), .wready(a_wready),
        .bresp(a_bresp), .bvalid(a_bvalid), .bready(a_bready),
        .araddr(a_araddr), .arvalid(a_arvalid), .arready(a_arready),
        .rdata(a_rdata), .rresp(a_rresp), .rvalid(a_rvalid), .rready(a_rready)
    );

    tb_axil_master #(.AW(AW)) bfm_b (
        .clk(clk), .awaddr(b_awaddr), .awvalid(b_awvalid), .awready(b_awready),
        .wdata(b_wdata), .wstrb(b_wstrb), .wvalid(b_wvalid), .wready(b_wready),
        .bresp(b_bresp), .bvalid(b_bvalid), .bready(b_bready),
        .araddr(b_araddr), .arvalid(b_arvalid), .arready(b_arready),
        .rdata(b_rdata), .rresp(b_rresp), .rvalid(b_rvalid), .rready(b_rready)
    );

    // ------------------------------------------------------------------ storage / bookkeeping
    logic [31:0] stim_main [0:STIM_MAX-1];
    logic [31:0] stim_alt  [0:STIM_MAX-1];
    logic [63:0] spec_mem  [0:SPEC_MAX-1];
    logic [31:0] hop0_mem  [0:MAX_N/2-1];     // first iSTFT hop of T2 (reference for T3)
    int spec_cnt = 0, out_cnt = 0;
    int errors = 0, proto_err = 0, polls = 0;
    int gap_pct = 20, rdy_pct = 70;
    bit allow_abort = 1'b0;
    bit tlast_every = 1'b0;
    bit last_a_tlast = 1'b0, last_b_tlast = 1'b0;   // tlast of the last captured beat                    // T10: host sends tlast on every frame
    int ns_main, ns_alt;
    int drv_a_idx = 0, drv_b_idx = 0;
    longint timeout_cyc;

    task automatic check(input bit cond, input string msg);
        if (!cond) begin
            errors++;
            $display("[TB] CHECK FAILED @%0d: %s", cycle, msg);
        end
    endtask

    task automatic expect_reg(input bit is_a, input logic [AW-1:0] addr,
                              input logic [31:0] mask, input logic [31:0] exp, input string name);
        logic [31:0] v;
        if (is_a) bfm_a.read(addr, v); else bfm_b.read(addr, v);
        check((v & mask) == (exp & mask),
              $sformatf("%s %s = 0x%08h, expected 0x%08h (mask 0x%08h)", is_a ? "STFT" : "iSTFT", name, v, exp, mask));
    endtask

    // ------------------------------------------------------------------ stream protocol checkers
    logic [64:0] am_hold; logic am_stall = 1'b0;
    logic [32:0] bm_hold; logic bm_stall = 1'b0;
    always @(posedge clk) begin
        if (am_stall && !allow_abort && (!a_m_tvalid || {a_m_tlast, a_m_tdata} != am_hold)) proto_err++;
        if (bm_stall && !allow_abort && (!b_m_tvalid || {b_m_tlast, b_m_tdata} != bm_hold)) proto_err++;
        if (!a_m_tvalid && (a_m_tdata != '0 || a_m_tlast)) proto_err++;
        if (!b_m_tvalid && (b_m_tdata != '0 || b_m_tlast)) proto_err++;
        am_stall <= a_m_tvalid && !a_m_tready;  am_hold <= {a_m_tlast, a_m_tdata};
        bm_stall <= b_m_tvalid && !b_m_tready;  bm_hold <= {b_m_tlast, b_m_tdata};
    end

    // ------------------------------------------------------------------ stream helpers
    // Masters keep tvalid high across back-to-back beats and drop it only for
    // an idle gap (end_audio / end_spec close the burst).
    task automatic send_audio(input int idx, input bit alt);
        if ($urandom_range(0, 99) < gap_pct) begin
            a_s_tvalid <= 1'b0;
            repeat ($urandom_range(1, 4)) @(posedge clk);
        end
        a_s_tdata  <= alt ? stim_alt[idx] : stim_main[idx];
        a_s_tvalid <= 1'b1;
        do @(posedge clk); while (!a_s_tready);
    endtask

    task automatic end_audio();
        a_s_tvalid <= 1'b0;
        a_s_tdata  <= '0;
    endtask

    task automatic send_spec(input logic [63:0] w, input bit last);
        if ($urandom_range(0, 99) < gap_pct) begin
            b_s_tvalid <= 1'b0;
            repeat ($urandom_range(1, 4)) @(posedge clk);
        end
        b_s_tdata  <= w;
        b_s_tlast  <= last;
        b_s_tvalid <= 1'b1;
        do @(posedge clk); while (!b_s_tready);
    endtask

    task automatic end_spec();
        b_s_tvalid <= 1'b0;
        b_s_tlast  <= 1'b0;
        b_s_tdata  <= '0;
    endtask

    // collect n audio words from the iSTFT, always ready.
    // fo > 0: write CSV; fo == 0: compare against hop0_mem (T2 first hop); fo < 0: count only
    task automatic collect_hop(input int n, input int fo);
        int k = 0, l, r;
        while (k < n) begin
            b_m_tready <= 1'b1;
            @(posedge clk);
            if (b_m_tvalid && b_m_tready) begin
                l = {{16{b_m_tdata[15]}}, b_m_tdata[15:0]};
                r = {{16{b_m_tdata[31]}}, b_m_tdata[31:16]};
                if (fo > 0) $fdisplay(fo, "%0d,%0d,%0d", k, l, r);
                check(b_m_tlast == (k == n - 1), $sformatf("iSTFT tlast at hop sample %0d", k));
                if (fo == 0 && n == MAX_N / 2) check(b_m_tdata == hop0_mem[k],
                    $sformatf("recovery hop sample %0d: 0x%08h vs 0x%08h", k, b_m_tdata, hop0_mem[k]));
                k++;
            end
        end
        b_m_tready <= 1'b0;
    endtask

    // ------------------------------------------------------------------ full streaming test
    task automatic run_stream(input int N, input int HL, input int B, input int NS, input bit alt,
                              input string tag);
        int H = N >> HL;
        int NF = ((NS - N) / H + 1) / B * B;     // whole packets only
        int NSU = (NF - 1) * H + N;              // input samples for exactly NF frames
        int nspec = NF * N;
        int nout = NF * H;
        int fs, fo;
        logic [31:0] v;
        fs = $fopen({"axi_spec_", tag, ".csv"}, "w"); $fdisplay(fs, "frame,k,re,im");
        fo = $fopen({"axi_out_", tag, ".csv"}, "w");  $fdisplay(fo, "idx,L,R");
        spec_cnt = 0; out_cnt = 0;
        $display("[TB] stream %s: N=%0d H=%0d batch=%0d NS=%0d frames=%0d packets=%0d",
                 tag, N, H, B, NSU, NF, NF / B);
        check(NF > 0 && NF % B == 0, "whole number of packets");
        fork
            begin : drv_a
                for (int i = 0; i < NSU; i++) begin send_audio(i, alt); drv_a_idx = i + 1; end
                end_audio();
            end
            begin : cap_a
                int re, im;
                while (spec_cnt < nspec) begin
                    a_m_tready <= ($urandom_range(0, 99) < rdy_pct);
                    @(posedge clk);
                    if (a_m_tvalid && a_m_tready) begin
                        spec_mem[spec_cnt] = a_m_tdata;
                        last_a_tlast = a_m_tlast;
                        re = a_m_tdata[31:0];
                        im = a_m_tdata[63:32];
                        $fdisplay(fs, "%0d,%0d,%0d,%0d", spec_cnt / N, spec_cnt % N, re, im);
                        check(a_m_tlast == (((spec_cnt % N) == N - 1) && ((spec_cnt / N) % B == B - 1)),
                              $sformatf("STFT tlast at word %0d", spec_cnt));
                        spec_cnt++;
                    end
                end
                a_m_tready <= 1'b0;
            end
            begin : drv_b
                for (int j = 0; j < nspec; j++) begin
                    if (spec_cnt <= j) begin
                        end_spec();
                        while (spec_cnt <= j) @(posedge clk);
                    end
                    send_spec(spec_mem[j], ((j % N) == N - 1) && (tlast_every || ((j / N) % B == B - 1)));
                    drv_b_idx = j + 1;
                end
                end_spec();
            end
            begin : cap_b
                int l, r;
                while (out_cnt < nout) begin
                    b_m_tready <= ($urandom_range(0, 99) < rdy_pct);
                    @(posedge clk);
                    if (b_m_tvalid && b_m_tready) begin
                        l = {{16{b_m_tdata[15]}}, b_m_tdata[15:0]};
                        r = {{16{b_m_tdata[31]}}, b_m_tdata[31:16]};
                        $fdisplay(fo, "%0d,%0d,%0d", out_cnt, l, r);
                        last_b_tlast = b_m_tlast;
                        if (N == MAX_N && HL == 1 && out_cnt < H) hop0_mem[out_cnt] = b_m_tdata;
                        check(b_m_tlast == (((out_cnt % H) == H - 1) && ((out_cnt / H) % B == B - 1)),
                              $sformatf("iSTFT tlast at sample %0d", out_cnt));
                        out_cnt++;
                    end
                end
                b_m_tready <= 1'b0;
            end
            begin : poll
                while (out_cnt < nout) begin
                    repeat (3000) @(posedge clk);
                    bfm_a.read(A_STATUS, v);
                    bfm_b.read(A_STATUS, v);
                    polls++;
                end
            end
        join
        $fclose(fs); $fclose(fo);
        check(last_a_tlast, "STFT: last beat of the run carries tlast");
        check(last_b_tlast, "iSTFT: last beat of the run carries tlast");
        expect_reg(1, A_FRAME,  32'hFFFF_FFFF, NF, "FRAME_CNT");
        expect_reg(0, A_FRAME,  32'hFFFF_FFFF, NF, "FRAME_CNT");
        expect_reg(1, A_STATUS, 32'h0000_003E, 32'h2, "STATUS (running, no flags)");
        if (tlast_every) expect_reg(0, A_STATUS, 32'h0000_003E, 32'hA, "STATUS (running, tlast error)");
        else             expect_reg(0, A_STATUS, 32'h0000_003E, 32'h2, "STATUS (running, no flags)");
        expect_reg(1, A_SAT, 32'hFFFF_FFFF, 0, "SAT_CNT");
        expect_reg(0, A_SAT, 32'hFFFF_FFFF, 0, "SAT_CNT");
        bfm_a.read(A_BUSYC, v);
        $display("[TB] stream %s done @%0d: spec words=%0d audio=%0d, STFT transform = %0d cycles",
                 tag, cycle, spec_cnt, out_cnt, v);
    endtask

    // configure one IP: FFT_N, HOP, BATCH, WDOG, then CTRL = 7 (enable + soft reset + commit)
    task automatic configure(input bit is_a, input int n, input logic [31:0] wdog,
                             input int hl = 1, input int b = 1);
        if (is_a) begin
            bfm_a.write(A_FFTN, n); bfm_a.write(A_HOP, hl); bfm_a.write(A_BATCH, b);
            bfm_a.write(A_WDOG, wdog); bfm_a.write(A_CTRL, 32'd7);
        end else begin
            bfm_b.write(A_FFTN, n); bfm_b.write(A_HOP, hl); bfm_b.write(A_BATCH, b);
            bfm_b.write(A_WDOG, wdog); bfm_b.write(A_CTRL, 32'd7);
        end
        repeat (20) @(posedge clk);
    endtask

    task automatic configure_both(input int n, input int hl, input int b);
        configure(1, n, DEF_WDOG, hl, b);
        configure(0, n, DEF_WDOG, hl, b);
        expect_reg(1, A_HOP_ACT,   32'hFFFF_FFFF, hl, "HOP_ACT");
        expect_reg(0, A_HOP_ACT,   32'hFFFF_FFFF, hl, "HOP_ACT");
        expect_reg(1, A_BATCH_ACT, 32'hFFFF_FFFF, b,  "BATCH_ACT");
        expect_reg(0, A_BATCH_ACT, 32'hFFFF_FFFF, b,  "BATCH_ACT");
    endtask

    // ------------------------------------------------------------------ main
    initial begin
        logic [31:0] v;
        int fr;
        if (!$value$plusargs("NS_MAIN=%d", ns_main)) ns_main = 12000;
        if (!$value$plusargs("NS_ALT=%d",  ns_alt))  ns_alt  = 6000;
        $readmemh(STIM_MAIN, stim_main, 0, ns_main - 1);
        $readmemh(STIM_ALT,  stim_alt,  0, ns_alt - 1);
        repeat (10) @(posedge clk);
        aresetn = 1'b1;
        repeat (5) @(posedge clk);

        // ---------------------------------------------------------------- T1
        $display("[TB] T1 register map");
        expect_reg(1, A_ID, 32'hFFFF_FFFF, 32'h5354_4602, "ID");
        expect_reg(0, A_ID, 32'hFFFF_FFFF, 32'h4953_4602, "ID");
        expect_reg(1, A_CTRL, 32'hFFFF_FFFF, 0, "CTRL after reset");
        expect_reg(1, A_STATUS, 32'h0000_003F, 0, "STATUS after reset");
        expect_reg(1, A_FFTN_ACT, 32'hFFFF_FFFF, 2048, "FFT_N_ACT default");
        expect_reg(1, A_WDOG_ACT, 32'hFFFF_FFFF, DEF_WDOG, "WDOG_ACT default");
        check(a_s_tready == 1'b0 && b_s_tready == 1'b0, "tready low while disabled");
        bfm_a.write(A_FFTN, 32'd1000);
        expect_reg(1, A_FFTN, 32'hFFFF_FFFF, 1000, "FFT_N shadow readback");
        expect_reg(1, A_FFTN_ACT, 32'hFFFF_FFFF, 2048, "FFT_N_ACT before commit");
        bfm_a.write(A_CTRL, 32'd5);
        repeat (20) @(posedge clk);
        expect_reg(1, A_FFTN_ACT, 32'hFFFF_FFFF, 2048, "FFT_N_ACT after invalid commit");
        expect_reg(1, A_STATUS, 32'h0000_0022, 32'h22, "STATUS running + invalid N");
        bfm_a.write(A_STATUS, 32'h20);
        expect_reg(1, A_STATUS, 32'h0000_0020, 0, "STATUS invalid N after W1C");
        bfm_a.write(A_WDOG, 32'd100); bfm_a.write(A_CTRL, 32'd5);
        expect_reg(1, A_WDOG_ACT, 32'hFFFF_FFFF, 4096, "WDOG clamp");
        bfm_a.write(A_WDOG, 32'd0); bfm_a.write(A_CTRL, 32'd5);
        expect_reg(1, A_WDOG_ACT, 32'hFFFF_FFFF, 0, "WDOG disable");
        expect_reg(1, A_NONE, 32'hFFFF_FFFF, 0, "unmapped read");
        // v2 geometry registers
        for (int ip = 0; ip < 2; ip++) begin
            expect_reg(ip, A_HOP_ACT,   32'hFFFF_FFFF, 1, "HOP_ACT default");
            expect_reg(ip, A_BATCH_ACT, 32'hFFFF_FFFF, 1, "BATCH_ACT default");
            expect_reg(ip, A_HOP,       32'hFFFF_FFFF, 1, "HOP shadow default");
            expect_reg(ip, A_BATCH,     32'hFFFF_FFFF, 1, "BATCH shadow default");
        end
        bfm_a.write(A_FFTN, 32'd1024); bfm_a.write(A_HOP, 32'd0); bfm_a.write(A_BATCH, 32'd4);
        bfm_a.write(A_CTRL, 32'd5);
        repeat (20) @(posedge clk);
        expect_reg(1, A_STATUS, 32'h0000_0020, 32'h20, "STATUS invalid HOP");
        expect_reg(1, A_FFTN_ACT, 32'hFFFF_FFFF, 2048, "FFT_N_ACT kept (atomic geometry)");
        expect_reg(1, A_BATCH_ACT, 32'hFFFF_FFFF, 1, "BATCH_ACT kept (atomic geometry)");
        bfm_a.write(A_STATUS, 32'h20);
        bfm_a.write(A_HOP, 32'd4);
        expect_reg(1, A_HOP, 32'hFFFF_FFFF, 0, "HOP out of range reads 0");
        bfm_a.write(A_HOP, 32'd3); bfm_a.write(A_BATCH, 32'd17); bfm_a.write(A_CTRL, 32'd5);
        repeat (20) @(posedge clk);
        expect_reg(1, A_STATUS, 32'h0000_0020, 32'h20, "STATUS invalid BATCH");
        expect_reg(1, A_HOP_ACT, 32'hFFFF_FFFF, 1, "HOP_ACT kept");
        bfm_a.write(A_STATUS, 32'h20);
        bfm_a.write(A_BATCH, 32'd16); bfm_a.write(A_CTRL, 32'd5);
        repeat (20) @(posedge clk);
        expect_reg(1, A_STATUS, 32'h0000_0020, 0, "STATUS valid geometry");
        expect_reg(1, A_FFTN_ACT,  32'hFFFF_FFFF, 1024, "FFT_N_ACT applied");
        expect_reg(1, A_HOP_ACT,   32'hFFFF_FFFF, 3, "HOP_ACT applied");
        expect_reg(1, A_BATCH_ACT, 32'hFFFF_FFFF, 16, "BATCH_ACT applied");
        bfm_a.write(A_HOP, 32'd1); bfm_a.write(A_BATCH, 32'd1);
        bfm_a.write(A_CTRL, 32'd0);
        repeat (5) @(posedge clk);
        expect_reg(1, A_STATUS, 32'h0000_0002, 0, "not running after CTRL=0");

        // ---------------------------------------------------------------- T2
        $display("[TB] T2 streaming N=2048");
        configure(1, 2048, DEF_WDOG);
        configure(0, 2048, DEF_WDOG);
        run_stream(2048, 1, 1, ns_main, 1'b0, "n2048");

        // ---------------------------------------------------------------- T3
        $display("[TB] T3 iSTFT watchdog on a partial frame");
        configure(0, 2048, 32'd20000);
        for (int j = 0; j < 100; j++) send_spec(spec_mem[j], 1'b0);
        end_spec();
        repeat (25000) @(posedge clk);
        expect_reg(0, A_STATUS, 32'h0000_0012, 32'h12, "STATUS watchdog + running");
        expect_reg(0, A_WDCNT, 32'hFFFF_FFFF, 1, "WDOG_CNT");
        bfm_b.write(A_STATUS, 32'h10);
        expect_reg(0, A_STATUS, 32'h0000_0010, 0, "STATUS watchdog after W1C");
        fork
            begin for (int j = 0; j < MAX_N; j++) send_spec(spec_mem[j], j == MAX_N - 1); end_spec(); end
            collect_hop(MAX_N / 2, 0);
        join
        $display("[TB] T3 recovery hop compared against first T2 hop");

        // ---------------------------------------------------------------- T4
        $display("[TB] T4 STFT watchdog on a stalled output");
        configure(1, 2048, 32'd20000);
        a_m_tready <= 1'b0;
        for (int i = 0; i < MAX_N; i++) send_audio(i, 1'b0);
        end_audio();
        fr = 0;
        while (!a_m_tvalid && fr < 2_000_000) begin @(posedge clk); fr++; end
        check(a_m_tvalid, "STFT frame produced before stall test");
        allow_abort = 1'b1;
        repeat (25000) @(posedge clk);
        check(!a_m_tvalid, "STFT output flushed by watchdog");
        allow_abort = 1'b0;
        expect_reg(1, A_STATUS, 32'h0000_0012, 32'h12, "STATUS watchdog + running");
        expect_reg(1, A_WDCNT, 32'hFFFF_FFFF, 1, "WDOG_CNT");

        // ---------------------------------------------------------------- T5
        $display("[TB] T5 iSTFT input saturation");
        configure(0, 2048, DEF_WDOG);
        fork
            begin
                for (int j = 0; j < MAX_N; j++)
                    send_spec((j == 1) ? 64'h0000_0000_7FFF_0000 : spec_mem[j], j == MAX_N - 1);
                end_spec();
            end
            collect_hop(MAX_N / 2, -1);  // count + tlast check only
        join
        expect_reg(0, A_STATUS, 32'h0000_0004, 32'h4, "STATUS saturation");
        bfm_b.read(A_SAT, v);
        check(v >= 1, $sformatf("SAT_CNT = %0d, expected >= 1", v));

        // ---------------------------------------------------------------- T6
        $display("[TB] T6 reconfiguration to N=256");
        configure(1, 256, DEF_WDOG);
        configure(0, 256, DEF_WDOG);
        expect_reg(1, A_FFTN_ACT, 32'hFFFF_FFFF, 256, "FFT_N_ACT");
        expect_reg(0, A_FFTN_ACT, 32'hFFFF_FFFF, 256, "FFT_N_ACT");
        run_stream(256, 1, 1, ns_alt, 1'b1, "n256");

        // ---------------------------------------------------------------- T7
        $display("[TB] T7 N=2048 hop N/4 batch 1");
        configure_both(2048, 2, 1);
        run_stream(2048, 2, 1, ns_main, 1'b0, "n2048_h4");

        // ---------------------------------------------------------------- T8
        $display("[TB] T8 N=2048 hop N/4 batch 4");
        configure_both(2048, 2, 4);
        run_stream(2048, 2, 4, ns_alt, 1'b1, "n2048_h4_b4");

        // ---------------------------------------------------------------- T9
        $display("[TB] T9 N=1024 hop N/8 batch 3");
        configure_both(1024, 3, 3);
        run_stream(1024, 3, 3, ns_alt, 1'b0, "n1024_h8_b3");

        // ---------------------------------------------------------------- T10
        $display("[TB] T10 iSTFT batch 2, host tlast on every frame");
        configure_both(256, 2, 2);
        tlast_every = 1'b1;
        run_stream(256, 2, 2, 2048, 1'b1, "n256_h4_b2_terr");
        tlast_every = 1'b0;

        // ---------------------------------------------------------------- summary
        $display("[TB] polls during streaming = %0d, AXI-Lite response errors = %0d",
                 polls, bfm_a.resp_errors + bfm_b.resp_errors);
        $display("[TB] checks failed = %0d, stream protocol errors = %0d", errors, proto_err);
        if (errors == 0 && proto_err == 0 && bfm_a.resp_errors == 0 && bfm_b.resp_errors == 0)
            $display("[TB] RESULT: PASS");
        else
            $display("[TB] RESULT: FAIL");
        $finish;
    end

    // global timeout
    initial begin
        if (!$value$plusargs("TIMEOUT=%d", timeout_cyc)) timeout_cyc = 200_000_000;
        repeat (timeout_cyc) @(posedge clk);
        $display("[TB] TIMEOUT @%0d", cycle);
        $display("[TB] drv_a=%0d spec_cnt=%0d drv_b=%0d out_cnt=%0d", drv_a_idx, spec_cnt, drv_b_idx, out_cnt);
        $display("[TB] a_s v/r=%0b%0b a_m v/r=%0b%0b b_s v/r=%0b%0b b_m v/r=%0b%0b",
                 a_s_tvalid, a_s_tready, a_m_tvalid, a_m_tready, b_s_tvalid, b_s_tready, b_m_tvalid, b_m_tready);
        $display("[TB] stft: rstn=%0b fft.state=%0d fft.in_cnt=%0d fft.busy=%0b framer.pending=%0b framer.cnt=%0d framer.first=%0b win.idx=%0d",
                 u_stft.core_rstn, u_stft.u_stft.u_fft.state, u_stft.u_stft.u_fft.in_cnt, u_stft.u_stft.u_fft.fft_busy,
                 u_stft.u_stft.u_framer.pending, u_stft.u_stft.u_framer.cnt_since, u_stft.u_stft.u_framer.first,
                 u_stft.u_stft.u_win.sample_idx);
        $display("[TB] istft: rstn=%0b fft.state=%0d fft.in_cnt=%0d core.state=%0d ola.idx=%0d n=%0d",
                 u_istft.core_rstn, u_istft.u_istft.u_ifft.state, u_istft.u_istft.u_ifft.in_cnt,
                 u_istft.u_istft.u_ifft.u_fft.state, u_istft.u_istft.u_ola.idx, u_istft.cfg_fft_n);
        $display("[TB] RESULT: FAIL");
        $finish;
    end

endmodule
