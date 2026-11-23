`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_stft_axil_ctrl
// Description : AXI4-Lite control/status block shared by vrm_stft_axi and
//               vrm_istft_axi (v2: hop and batch registers). Shadow registers + single-write atomic commit
//               (commit = CTRL bit 2, value 5 = enable + commit),
//               core flush/reset sequencing and a progress watchdog
//.
//
// AXI4-Lite slave :
//   - AW and W channels are accepted independently (any order, same cycle
//     allowed); the register write is performed once both are held.
//   - One outstanding write and one outstanding read.
//   - Read data is registered on the AR handshake (registers only).
//   - Reads have no side effects; state changes only on writes.
//   - Unmapped reads return 0 (zero-gating, ). BRESP/RRESP = OKAY.
//
// Register map (byte offsets, 32-bit):
//   0x00 CTRL        RW  [0] enable
//                        [1] soft reset (write 1: flush core, clear counters
//                            and sticky flags; self-clearing)
//                        [2] commit     (write 1: copy shadow FFT_N / HOP /
//                            BATCH / WDOG to active, flush core; self-clearing)
//                        write 5 = enable + commit (standard bring-up)
//   0x04 STATUS      R/W1C [0] busy (transform running)        RO
//                          [1] running (core out of reset)      RO
//                          [2] saturation seen                  W1C
//                          [3] tlast position error seen        W1C
//                          [4] watchdog trip seen               W1C
//                          [5] invalid config on last commit    W1C
//                              (FFT_N, HOP or BATCH out of range)
//   0x08 FFT_N       RW  shadow: 256 / 512 / 1024 / 2048
//   0x0C FFT_N_ACT   RO  active FFT size
//   0x10 WDOG        RW  shadow watchdog limit in clock cycles
//                        (0 = disabled, 1..WDOG_MIN-1 clamped to WDOG_MIN)
//   0x14 WDOG_ACT    RO  active watchdog limit
//   0x18 FRAME_CNT   RO  frames completed at the output port
//   0x1C SAT_CNT     RO  saturation events
//   0x20 WDOG_CNT    RO  watchdog trips
//   0x24 BUSY_CYC    RO  duration of the last transform in clock cycles
//   0x28 ID          RO  IP_ID
//   0x2C HOP         RW  shadow hop: log2(N/H), 1 = N/2 (default), 2 = N/4, 3 = N/8
//   0x30 HOP_ACT     RO  active hop code
//   0x34 BATCH       RW  shadow frames per stream packet (tlast), 1 (default)..MAX_BATCH
//   0x38 BATCH_ACT   RO  active batch size
//
// Commit: FFT_N, HOP and BATCH form the stream geometry and are applied
// together only if all three are legal (else STATUS[5] is set and the active
// geometry is kept). WDOG is applied on every commit (clamped). With the
// defaults (HOP = 1, BATCH = 1) the IP behaves exactly like v1.
//
// Watchdog: counts consecutive cycles in which work is pending (a stalled
// handshake on either stream port, or a partially received frame) and no
// progress is made (no handshake on either port, core not busy). On reaching
// the active limit the core is flushed (tready drops, internal state reset)
// and the STATUS[4] flag / WDOG_CNT are updated.
// ============================================================================

module vrm_stft_axil_ctrl #(
    parameter integer C_S_AXI_ADDR_WIDTH = 6,
    parameter [31:0]  IP_ID              = 32'h5354_4601,
    parameter [15:0]  DEF_FFT_N          = 16'd2048,
    parameter [31:0]  DEF_WDOG           = 32'h0800_0000,   // 134 M cycles (1.34 s at 100 MHz)
    parameter [31:0]  WDOG_MIN           = 32'd4096,
    parameter [3:0]   FLUSH_CYCLES       = 4'd8,
    parameter [1:0]   DEF_HOP_LOG2       = 2'd1,
    parameter [4:0]   DEF_BATCH          = 5'd1,
    parameter [4:0]   MAX_BATCH          = 5'd16
) (
    input  wire                          aclk,
    input  wire                          aresetn,

    // AXI4-Lite slave
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    input  wire                          s_axi_awvalid,
    output wire                          s_axi_awready,
    input  wire [31:0]                   s_axi_wdata,
    input  wire [3:0]                    s_axi_wstrb,
    input  wire                          s_axi_wvalid,
    output wire                          s_axi_wready,
    output wire [1:0]                    s_axi_bresp,
    output wire                          s_axi_bvalid,
    input  wire                          s_axi_bready,
    input  wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    input  wire                          s_axi_arvalid,
    output wire                          s_axi_arready,
    output wire [31:0]                   s_axi_rdata,
    output wire [1:0]                    s_axi_rresp,
    output wire                          s_axi_rvalid,
    input  wire                          s_axi_rready,

    // core control
    output reg                           core_rstn,     // active-low core reset (registered)
    output reg  [15:0]                   cfg_fft_n,     // active FFT size
    output reg  [1:0]                    cfg_hop_log2,  // active hop code (H = N >> code)
    output reg  [4:0]                    cfg_batch,     // active frames per packet

    // core status / events
    input  wire                          core_busy,
    input  wire                          evt_frame,     // pulse per completed output frame / hop
    input  wire                          evt_sat,       // pulse per saturation event
    input  wire                          evt_tlast_err, // pulse per tlast mismatch
    input  wire                          wd_pending,    // work pending this cycle
    input  wire                          wd_progress    // forward progress this cycle
);

    // ------------------------------------------------------------------------
    // Register offsets (word index = byte offset >> 2)
    // ------------------------------------------------------------------------
    localparam [3:0] R_CTRL = 4'h0, R_STATUS = 4'h1, R_FFT_N = 4'h2, R_FFT_N_ACT = 4'h3,
                     R_WDOG = 4'h4, R_WDOG_ACT = 4'h5, R_FRAME = 4'h6, R_SAT = 4'h7,
                     R_WDCNT = 4'h8, R_BUSYC = 4'h9, R_ID = 4'hA, R_HOP = 4'hB,
                     R_HOP_ACT = 4'hC, R_BATCH = 4'hD, R_BATCH_ACT = 4'hE;

    // ------------------------------------------------------------------------
    // AXI4-Lite write channel: independent AW / W capture
    // ------------------------------------------------------------------------
    reg        aw_hold, w_hold, bvalid_r;
    reg [3:0]  aw_idx;
    reg [31:0] w_data;
    reg [3:0]  w_strb;

    assign s_axi_awready = !aw_hold && !bvalid_r;
    assign s_axi_wready  = !w_hold  && !bvalid_r;
    assign s_axi_bvalid  = bvalid_r;
    assign s_axi_bresp   = 2'b00;

    wire wr_go = aw_hold && w_hold;            // perform the register write this cycle

    always @(posedge aclk) begin
        if (!aresetn) begin
            aw_hold <= 1'b0; w_hold <= 1'b0; bvalid_r <= 1'b0;
            aw_idx <= 4'd0; w_data <= 32'd0; w_strb <= 4'd0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin
                aw_hold <= 1'b1;
                aw_idx  <= s_axi_awaddr[5:2];
            end
            if (s_axi_wvalid && s_axi_wready) begin
                w_hold <= 1'b1;
                w_data <= s_axi_wdata;
                w_strb <= s_axi_wstrb;
            end
            if (wr_go) begin
                aw_hold  <= 1'b0;
                w_hold   <= 1'b0;
                bvalid_r <= 1'b1;
            end else if (bvalid_r && s_axi_bready) begin
                bvalid_r <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Register file, commit, flush, watchdog, counters (single writer)
    // ------------------------------------------------------------------------
    reg        ctrl_en;
    reg [15:0] fft_n_sh;
    reg [1:0]  hop_sh;
    reg [4:0]  batch_sh;
    reg [31:0] wdog_sh, wdog_act;
    reg        st_sat, st_terr, st_wdog, st_cfg;
    reg [31:0] cnt_frame, cnt_sat, cnt_wdog, busy_cyc, busy_acc;
    reg        busy_q;
    reg [3:0]  flush_cnt;
    reg [31:0] wd_cnt;

    // byte-masked merge for RW shadow registers
    function [31:0] merge;
        input [31:0] old_v;
        input [31:0] new_v;
        input [3:0]  strb;
        begin
            merge[7:0]   = strb[0] ? new_v[7:0]   : old_v[7:0];
            merge[15:8]  = strb[1] ? new_v[15:8]  : old_v[15:8];
            merge[23:16] = strb[2] ? new_v[23:16] : old_v[23:16];
            merge[31:24] = strb[3] ? new_v[31:24] : old_v[31:24];
        end
    endfunction

    wire wr_ctrl   = wr_go && (aw_idx == R_CTRL)   && w_strb[0];
    wire wr_status = wr_go && (aw_idx == R_STATUS) && w_strb[0];
    wire do_soft   = wr_ctrl && w_data[1];
    wire do_commit = wr_ctrl && w_data[2];

    wire n_legal   = (fft_n_sh == 16'd256)  || (fft_n_sh == 16'd512) ||
                     (fft_n_sh == 16'd1024) || (fft_n_sh == 16'd2048);
    wire hop_legal = (hop_sh != 2'd0);                          // 1..3 (H >= 32 for N >= 256)
    wire bat_legal = (batch_sh != 5'd0) && (batch_sh <= MAX_BATCH);
    wire geo_legal = n_legal && hop_legal && bat_legal;
    wire [31:0] wdog_clamped = (wdog_sh == 32'd0)    ? 32'd0 :
                               (wdog_sh <  WDOG_MIN) ? WDOG_MIN : wdog_sh;

    wire wd_armed = core_rstn && (wdog_act != 32'd0) && wd_pending && !wd_progress;
    wire wd_trip  = wd_armed && (wd_cnt >= wdog_act - 32'd1);
    wire flush_rq = do_soft || do_commit || wd_trip;

    always @(posedge aclk) begin
        if (!aresetn) begin
            ctrl_en   <= 1'b0;
            fft_n_sh  <= DEF_FFT_N;  cfg_fft_n <= DEF_FFT_N;
            hop_sh    <= DEF_HOP_LOG2; cfg_hop_log2 <= DEF_HOP_LOG2;
            batch_sh  <= DEF_BATCH;    cfg_batch    <= DEF_BATCH;
            wdog_sh   <= DEF_WDOG;   wdog_act  <= DEF_WDOG;
            st_sat    <= 1'b0; st_terr <= 1'b0; st_wdog <= 1'b0; st_cfg <= 1'b0;
            cnt_frame <= 32'd0; cnt_sat <= 32'd0; cnt_wdog <= 32'd0;
            busy_cyc  <= 32'd0; busy_acc <= 32'd0; busy_q <= 1'b0;
            flush_cnt <= 4'd0; wd_cnt <= 32'd0;
            core_rstn <= 1'b0;
        end else begin
            // ---------------- register writes
            if (wr_ctrl) ctrl_en <= w_data[0];
            if (wr_go && aw_idx == R_FFT_N) fft_n_sh <= merge({16'd0, fft_n_sh}, w_data, w_strb) & 32'h0000_FFFF;
            if (wr_go && aw_idx == R_WDOG)  wdog_sh  <= merge(wdog_sh, w_data, w_strb);
            if (wr_go && aw_idx == R_HOP   && w_strb[0]) hop_sh   <= (|w_data[31:2]) ? 2'd0 : w_data[1:0];   // out of range -> 0 (illegal)
            if (wr_go && aw_idx == R_BATCH && w_strb[0]) batch_sh <= (|w_data[31:5]) ? 5'd0 : w_data[4:0];  // > 31 -> 0 (illegal)

            // ---------------- commit: shadow -> active
            if (do_commit) begin
                if (geo_legal) begin
                    cfg_fft_n    <= fft_n_sh;
                    cfg_hop_log2 <= hop_sh;
                    cfg_batch    <= batch_sh;
                end
                wdog_act <= wdog_clamped;
            end

            // ---------------- sticky flags (event set has priority over W1C)
            st_sat  <= evt_sat       | (st_sat  & ~(wr_status & w_data[2]) & ~do_soft);
            st_terr <= evt_tlast_err | (st_terr & ~(wr_status & w_data[3]) & ~do_soft);
            st_wdog <= wd_trip       | (st_wdog & ~(wr_status & w_data[4]) & ~do_soft);
            st_cfg  <= (do_commit & ~geo_legal) | (st_cfg & ~(wr_status & w_data[5]) & ~do_soft);

            // ---------------- counters
            if (do_soft) begin
                cnt_frame <= 32'd0; cnt_sat <= 32'd0; cnt_wdog <= 32'd0;
            end else begin
                if (evt_frame) cnt_frame <= cnt_frame + 32'd1;
                if (evt_sat)   cnt_sat   <= cnt_sat   + 32'd1;
                if (wd_trip)   cnt_wdog  <= cnt_wdog  + 32'd1;
            end

            // ---------------- transform duration
            busy_q <= core_busy;
            if (core_busy)                 busy_acc <= busy_acc + 32'd1;
            else                           busy_acc <= 32'd0;
            if (busy_q && !core_busy)      busy_cyc <= busy_acc;

            // ---------------- watchdog
            if (wd_armed && !wd_trip) wd_cnt <= wd_cnt + 32'd1;
            else                      wd_cnt <= 32'd0;

            // ---------------- flush / core reset sequencing
            if (flush_rq)               flush_cnt <= FLUSH_CYCLES;
            else if (flush_cnt != 4'd0) flush_cnt <= flush_cnt - 4'd1;
            core_rstn <= ctrl_en && (flush_cnt == 4'd0) && !flush_rq;
        end
    end

    // ------------------------------------------------------------------------
    // AXI4-Lite read channel
    // ------------------------------------------------------------------------
    reg        rvalid_r;
    reg [31:0] rdata_r;
    reg [31:0] rd_mux;

    assign s_axi_arready = !rvalid_r;
    assign s_axi_rvalid  = rvalid_r;
    assign s_axi_rdata   = rdata_r;
    assign s_axi_rresp   = 2'b00;

    always @(*) begin
        case (s_axi_araddr[5:2])
            R_CTRL:      rd_mux = {31'd0, ctrl_en};
            R_STATUS:    rd_mux = {26'd0, st_cfg, st_wdog, st_terr, st_sat, core_rstn, core_busy};
            R_FFT_N:     rd_mux = {16'd0, fft_n_sh};
            R_FFT_N_ACT: rd_mux = {16'd0, cfg_fft_n};
            R_WDOG:      rd_mux = wdog_sh;
            R_WDOG_ACT:  rd_mux = wdog_act;
            R_FRAME:     rd_mux = cnt_frame;
            R_SAT:       rd_mux = cnt_sat;
            R_WDCNT:     rd_mux = cnt_wdog;
            R_BUSYC:     rd_mux = busy_cyc;
            R_ID:        rd_mux = IP_ID;
            R_HOP:       rd_mux = {30'd0, hop_sh};
            R_HOP_ACT:   rd_mux = {30'd0, cfg_hop_log2};
            R_BATCH:     rd_mux = {27'd0, batch_sh};
            R_BATCH_ACT: rd_mux = {27'd0, cfg_batch};
            default:     rd_mux = 32'd0;
        endcase
    end

    always @(posedge aclk) begin
        if (!aresetn) begin
            rvalid_r <= 1'b0;
            rdata_r  <= 32'd0;
        end else begin
            if (s_axi_arvalid && !rvalid_r) begin
                rvalid_r <= 1'b1;
                rdata_r  <= rd_mux;
            end else if (rvalid_r && s_axi_rready) begin
                rvalid_r <= 1'b0;
                rdata_r  <= 32'd0;
            end
        end
    end

endmodule
