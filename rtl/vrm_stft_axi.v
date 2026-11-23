`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_stft_axi
// Description : SoC wrapper for vrm_stft (stereo STFT, periodic sqrt-Hann),
//               v2: hop H = N/2, N/4, N/8 (HOP register) and B frames per
//               stream packet (BATCH register). AXI4-Lite control (vrm_stft_axil_ctrl) +
//               AXI4-Stream audio in + AXI4-Stream spectrum out.
//
// s_axis (audio, from AXI DMA MM2S, 32-bit)
//   tdata = {R[15:0], L[15:0]} Q1.15. s_axis_tlast is
//   ignored: framing is counted from FFT_N, so the host may send
//   audio in chunks of any length.
//
// m_axis (spectrum, to AXI DMA S2MM, 64-bit)
//   One packet of B*N beats per B frames (BATCH = B, default 1), natural
//   bin order, tlast only on bin N-1 of every B-th frame.
//   tdata = {im[31:0], re[31:0]}, each a sign-extended 24-bit value, so the
//   host sees an int32 array [re0, im0, re1, im1,...] (little endian).
//   Z[k] = Z_L[k] + j*Z_R[k] packing of the two real channels:
//     Z_int[k] = 128 * DFT{w * (xL + j*xR)}[k] / N        (x in int16 units)
//     X_L[k] = (Z[k] + conj(Z[N-k])) / 2,  X_R[k] = (Z[k] - conj(Z[N-k])) / (2j)
//
// Frame timing : the first frame is emitted after N input samples (host
//                primes N - H samples, then sends H samples per frame), then
//                one frame every H samples. FRAME_CNT counts frames, not
//                packets.
// DMA note     : one frame is N*8 bytes (16384 bytes for N = 2048), which is
//                above the 16383-byte limit of a 14-bit AXI DMA buffer-length
//                register. Configure the DMA buffer length
//                width >= 15 bits (e.g. 26); a batch is B*N*8 bytes
//                (256 KiB for N = 2048, B = 16).
// Reset/flush  : while not running (disabled, flushing after commit / soft
//                reset / watchdog) s_axis_tready = 0 and m_axis_tvalid = 0.
//                A watchdog flush aborts an output packet in progress.
// ============================================================================

module vrm_stft_axi #(
    parameter integer C_S_AXI_ADDR_WIDTH = 6,
    parameter integer MAX_N     = 2048,
    parameter integer DATA_W    = 24,
    parameter integer TWID_W    = 18,
    parameter integer COEF_W    = 18,
    parameter WIN_FILE          = "window_2048_q17.mem",
    parameter TWID_FILE         = "twiddle_2048_q17.mem"
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

    // AXI4-Stream slave: audio
    input  wire [31:0]                   s_axis_tdata,
    input  wire                          s_axis_tlast,   // ignored (see header)
    input  wire                          s_axis_tvalid,
    output wire                          s_axis_tready,

    // AXI4-Stream master: spectrum
    output wire [63:0]                   m_axis_tdata,
    output wire                          m_axis_tlast,
    output wire                          m_axis_tvalid,
    input  wire                          m_axis_tready
);

    wire        core_rstn;
    wire [15:0] cfg_fft_n;
    wire [1:0]  cfg_hop_log2;
    wire [4:0]  cfg_batch;
    wire        running = core_rstn;

    // ------------------------------------------------------------------------
    // Core
    // ------------------------------------------------------------------------
    wire                c_s_tready;
    wire [2*DATA_W-1:0] c_m_tdata;
    wire                c_m_tlast, c_m_tvalid;
    wire [31:0]         c_frames;
    wire                c_busy, c_sat, c_terr;

    vrm_stft #(
        .MAX_N(MAX_N), .DATA_W(DATA_W), .TWID_W(TWID_W), .COEF_W(COEF_W),
        .WIN_FILE(WIN_FILE), .TWID_FILE(TWID_FILE)
    ) u_stft (
        .clk(aclk), .rstn(core_rstn), .cfg_fft_n(cfg_fft_n), .cfg_hop_log2(cfg_hop_log2),
        .s_axis_tdata(s_axis_tdata), .s_axis_tvalid(s_axis_tvalid && running),
        .s_axis_tready(c_s_tready),
        .m_axis_tdata(c_m_tdata), .m_axis_tlast(c_m_tlast),
        .m_axis_tvalid(c_m_tvalid), .m_axis_tready(m_axis_tready && running),
        .frame_count(c_frames), .busy(c_busy), .sat_event(c_sat), .tlast_err(c_terr)
    );

    // ------------------------------------------------------------------------
    // Stream boundary
    // ------------------------------------------------------------------------
    wire signed [DATA_W-1:0] z_re = c_m_tdata[2*DATA_W-1:DATA_W];
    wire signed [DATA_W-1:0] z_im = c_m_tdata[DATA_W-1:0];

    assign s_axis_tready = c_s_tready && running;
    assign m_axis_tvalid = c_m_tvalid && running;
    reg  [4:0] bat_cnt;                      // frames sent in the current packet
    wire       pkt_end = (bat_cnt == cfg_batch - 5'd1);
    assign m_axis_tlast  = m_axis_tvalid && c_m_tlast && pkt_end;
    assign m_axis_tdata  = m_axis_tvalid ? {{(32-DATA_W){z_im[DATA_W-1]}}, z_im,
                                            {(32-DATA_W){z_re[DATA_W-1]}}, z_re} : 64'd0;

    wire s_fire = s_axis_tvalid && s_axis_tready;
    wire m_fire = m_axis_tvalid && m_axis_tready;
    wire m_frame_end = m_fire && c_m_tlast;

    always @(posedge aclk) begin
        if (!core_rstn)       bat_cnt <= 5'd0;
        else if (m_frame_end) bat_cnt <= pkt_end ? 5'd0 : bat_cnt + 5'd1;
    end

    // ------------------------------------------------------------------------
    // Control / status
    // ------------------------------------------------------------------------
    vrm_stft_axil_ctrl #(
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH), .IP_ID(32'h5354_4602)   // "ST" F v2
    ) u_ctrl (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wvalid(s_axi_wvalid),
        .s_axi_wready(s_axi_wready), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready), .s_axi_araddr(s_axi_araddr), .s_axi_arvalid(s_axi_arvalid),
        .s_axi_arready(s_axi_arready), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
        .core_rstn(core_rstn), .cfg_fft_n(cfg_fft_n),
        .cfg_hop_log2(cfg_hop_log2), .cfg_batch(cfg_batch),
        .core_busy(c_busy),
        .evt_frame(m_frame_end),
        .evt_sat(c_sat),
        .evt_tlast_err(c_terr),
        .wd_pending((s_axis_tvalid && !s_axis_tready && running) || (m_axis_tvalid && !m_axis_tready)),
        .wd_progress(s_fire || m_fire || c_busy)
    );

endmodule
