`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_istft_axi
// Description : SoC wrapper for vrm_istft (stereo inverse STFT, periodic
//               sqrt-Hann synthesis window, K-frame overlap-add), v2: hop
//               H = N/2, N/4, N/8 (HOP register) and B frames per stream
//               packet (BATCH register).
//               AXI4-Lite control (vrm_stft_axil_ctrl) + AXI4-Stream spectrum
//               in + AXI4-Stream audio out.
//
// s_axis (spectrum, from AXI DMA MM2S, 64-bit)
//   N beats per frame, natural bin order. tdata = {im[31:0], re[31:0]}
//   (int32 pairs, same format and scale as the vrm_stft_axi output).
//   Each component is saturated to 24 bit at this boundary; saturations are
//   counted in SAT_CNT. Frame length is counted from FFT_N;
//   per-frame tlast for the core is generated here. s_axis_tlast is only
//   checked (STATUS[3]): expected on bin N-1 of every B-th frame (one DMA
//   packet of B*N beats); other packet sizes still work but set the flag. For a real output the spectrum must keep the
//   two-real packing: Y[k] = Y_L[k] + j*Y_R[k] with Y_L, Y_R conjugate-
//   symmetric (for a per-bin real mask a, b:
//   Y[k] = (a+b)/2 * Z[k] + (a-b)/2 * conj(Z[N-k])).
//
// m_axis (audio, to AXI DMA S2MM, 32-bit)
//   H samples per input frame, tdata = {R[15:0], L[15:0]} Q1.15,
//   tlast on the last sample of every B-th hop (B*H samples per packet).
//   The first K-1 hops after a flush (K = N/H) are the fade-in of the first
//   frames. FRAME_CNT counts hops (frames), not packets.
//
// Reset/flush  : while not running s_axis_tready = 0 and m_axis_tvalid = 0.
//                A watchdog flush aborts an output packet in progress.
// ============================================================================

module vrm_istft_axi #(
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

    // AXI4-Stream slave: spectrum
    input  wire [63:0]                   s_axis_tdata,
    input  wire                          s_axis_tlast,
    input  wire                          s_axis_tvalid,
    output wire                          s_axis_tready,

    // AXI4-Stream master: audio
    output wire [31:0]                   m_axis_tdata,
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
    // Input boundary: int32 -> DATA_W with saturation
    // ------------------------------------------------------------------------
    localparam signed [31:0] DMAX = (32'sd1 <<< (DATA_W - 1)) - 32'sd1;
    localparam signed [31:0] DMIN = -(32'sd1 <<< (DATA_W - 1));

    wire signed [31:0] in_re = s_axis_tdata[31:0];
    wire signed [31:0] in_im = s_axis_tdata[63:32];
    wire re_hi = (in_re > DMAX), re_lo = (in_re < DMIN);
    wire im_hi = (in_im > DMAX), im_lo = (in_im < DMIN);
    wire [DATA_W-1:0] q_re = re_hi ? DMAX[DATA_W-1:0] : re_lo ? DMIN[DATA_W-1:0] : in_re[DATA_W-1:0];
    wire [DATA_W-1:0] q_im = im_hi ? DMAX[DATA_W-1:0] : im_lo ? DMIN[DATA_W-1:0] : in_im[DATA_W-1:0];

    // ------------------------------------------------------------------------
    // Input framing: per-frame tlast for the core, packet tlast check
    // ------------------------------------------------------------------------
    localparam BW = $clog2(MAX_N);
    reg  [BW-1:0] in_bin;                    // bin index within the frame
    reg  [4:0]    in_frm;                    // frame index within the packet
    wire [BW-1:0] n_last   = cfg_fft_n[BW:0] - 1'b1;
    wire          in_flast = (in_bin == n_last);
    wire          in_plast = in_flast && (in_frm == cfg_batch - 5'd1);

    // ------------------------------------------------------------------------
    // Core
    // ------------------------------------------------------------------------
    wire        c_s_tready;
    wire [31:0] c_m_tdata;
    wire        c_m_tlast, c_m_tvalid;
    wire        c_busy, c_sat, c_terr, c_partial;

    vrm_istft #(
        .MAX_N(MAX_N), .DATA_W(DATA_W), .TWID_W(TWID_W), .COEF_W(COEF_W),
        .WIN_FILE(WIN_FILE), .TWID_FILE(TWID_FILE)
    ) u_istft (
        .clk(aclk), .rstn(core_rstn), .cfg_fft_n(cfg_fft_n), .cfg_hop_log2(cfg_hop_log2),
        .s_axis_tdata({q_re, q_im}), .s_axis_tlast(in_flast),
        .s_axis_tvalid(s_axis_tvalid && running), .s_axis_tready(c_s_tready),
        .m_axis_tdata(c_m_tdata), .m_axis_tlast(c_m_tlast),
        .m_axis_tvalid(c_m_tvalid), .m_axis_tready(m_axis_tready && running),
        .busy(c_busy), .sat_event(c_sat), .tlast_err(c_terr), .frame_partial(c_partial)
    );

    assign s_axis_tready = c_s_tready && running;
    assign m_axis_tvalid = c_m_tvalid && running;
    reg  [4:0] out_hop;                      // hops sent in the current packet
    wire       pkt_end = (out_hop == cfg_batch - 5'd1);
    assign m_axis_tlast  = m_axis_tvalid && c_m_tlast && pkt_end;
    assign m_axis_tdata  = m_axis_tvalid ? c_m_tdata : 32'd0;

    wire s_fire = s_axis_tvalid && s_axis_tready;
    wire m_fire = m_axis_tvalid && m_axis_tready;
    wire in_sat = s_fire && (re_hi || re_lo || im_hi || im_lo);
    wire m_hop_end = m_fire && c_m_tlast;
    reg  in_terr;

    always @(posedge aclk) begin
        if (!core_rstn) begin
            in_bin <= {BW{1'b0}}; in_frm <= 5'd0; out_hop <= 5'd0; in_terr <= 1'b0;
        end else begin
            in_terr <= s_fire && (s_axis_tlast != in_plast);
            if (s_fire) begin
                if (in_flast) begin
                    in_bin <= {BW{1'b0}};
                    in_frm <= in_plast ? 5'd0 : in_frm + 5'd1;
                end else begin
                    in_bin <= in_bin + 1'b1;
                end
            end
            if (m_hop_end) out_hop <= pkt_end ? 5'd0 : out_hop + 5'd1;
        end
    end

    // ------------------------------------------------------------------------
    // Control / status
    // ------------------------------------------------------------------------
    vrm_stft_axil_ctrl #(
        .C_S_AXI_ADDR_WIDTH(C_S_AXI_ADDR_WIDTH), .IP_ID(32'h4953_4602)   // "IS" F v2
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
        .evt_frame(m_hop_end),
        .evt_sat(c_sat || in_sat),
        .evt_tlast_err(c_terr || in_terr),
        .wd_pending((s_axis_tvalid && !s_axis_tready && running) ||
                    (m_axis_tvalid && !m_axis_tready) || (c_partial && running)),
        .wd_progress(s_fire || m_fire || c_busy)
    );

endmodule
