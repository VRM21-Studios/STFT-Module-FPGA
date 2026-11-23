`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_istft
// Description : Stereo inverse STFT, hop H = N >> cfg_hop_log2 (N/2, N/4,
//               N/8; v2), periodic sqrt-Hann synthesis window, K-frame
//               overlap-add (K = N/H) with COLA normalisation.
//
//   s_axis (spectrum {re, im}, N bins per frame, natural order, tlast on N-1)
//     -> vrm_fft_stream_core  : unscaled IFFT -> y = IDFT(Y)*N  (= w_a*x for Y = Z)
//     -> lane mapping         : re = L, im = R (valid when Y is conjugate-
//                               symmetric per channel, i.e. Y = Y_L + j*Y_R)
//     -> vrm_windowing_core   : y * w_s (DATA_W -> DATA_W)
//     -> vrm_overlap_add      : out = sum_j frame_{f-j}[jH.. jH+H-1] / 2^(hop_log2-1)
//     -> m_axis (audio {R,L} Q1.15, H samples per frame)
//
// Gain : STFT -> iSTFT with an identity mask is x * sum_j w^2[n+jH] / (N/2H) = x.
// ============================================================================

module vrm_istft #(
    parameter MAX_N     = 2048,
    parameter DATA_W    = 24,
    parameter TWID_W    = 18,
    parameter COEF_W    = 18,
    parameter WIN_FILE  = "window_2048_q17.mem",
    parameter TWID_FILE = "twiddle_2048_q17.mem"
) (
    input  wire                clk,
    input  wire                rstn,

    input  wire [15:0]         cfg_fft_n,      // 256/512/1024/2048, change only in reset
    input  wire [1:0]          cfg_hop_log2,   // 1/2/3 -> H = N/2, N/4, N/8, change only in reset

    input  wire [2*DATA_W-1:0] s_axis_tdata,   // {re, im}
    input  wire                s_axis_tlast,
    input  wire                s_axis_tvalid,
    output wire                s_axis_tready,

    output wire [31:0]         m_axis_tdata,   // {R, L}
    output wire                m_axis_tlast,
    output wire                m_axis_tvalid,
    input  wire                m_axis_tready,

    output wire                busy,
    output wire                sat_event,
    output wire                tlast_err,
    output wire                frame_partial   // IFFT input frame started but incomplete
);

    localparam integer OUT_SHIFT = (DATA_W - 2) - 15;

    wire [2*DATA_W-1:0] y_tdata;
    wire                y_tlast, y_tvalid, y_tready;
    wire [2*DATA_W-1:0] ws_tdata;
    wire                ws_tlast, ws_tvalid, ws_tready;
    wire                fft_sat, win_sat, ola_sat, fft_terr, ola_terr;

    vrm_fft_stream_core #(
        .MAX_N(MAX_N), .DATA_W(DATA_W), .TWID_W(TWID_W),
        .ROUND(1), .IFFT_SCALE(0), .TWID_STRIDE(1), .TWID_FILE(TWID_FILE)
    ) u_ifft (
        .clk(clk), .rstn(rstn), .cfg_mode(1'b1), .cfg_fft_n(cfg_fft_n),
        .s_axis_tdata(s_axis_tdata), .s_axis_tlast(s_axis_tlast),
        .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
        .m_axis_tdata(y_tdata), .m_axis_tlast(y_tlast),
        .m_axis_tvalid(y_tvalid), .m_axis_tready(y_tready),
        .busy(busy), .frame_done(), .sat_event(fft_sat), .tlast_err(fft_terr),
        .frame_partial(frame_partial)
    );

    // {re, im} -> {lane1 = R = im, lane0 = L = re}
    wire [2*DATA_W-1:0] y_lanes = {y_tdata[DATA_W-1:0], y_tdata[2*DATA_W-1:DATA_W]};

    vrm_windowing_core #(
        .MAX_PTS(MAX_N), .IN_W(DATA_W), .OUT_W(DATA_W), .COEF_W(COEF_W),
        .SHIFT(COEF_W - 1), .ROUND(1), .ROM_FILE(WIN_FILE)
    ) u_win (
        .aclk(clk), .aresetn(rstn), .cfg_fft_n(cfg_fft_n),
        .s_axis_tdata(y_lanes), .s_axis_tlast(y_tlast),
        .s_axis_tvalid(y_tvalid), .s_axis_tready(y_tready),
        .m_axis_tdata(ws_tdata), .m_axis_tlast(ws_tlast),
        .m_axis_tvalid(ws_tvalid), .m_axis_tready(ws_tready),
        .sat_event(win_sat)
    );

    vrm_overlap_add #(
        .MAX_N(MAX_N), .IN_W(DATA_W), .OUT_W(16), .OUT_SHIFT(OUT_SHIFT)
    ) u_ola (
        .clk(clk), .rstn(rstn), .cfg_fft_n(cfg_fft_n), .cfg_hop_log2(cfg_hop_log2),
        .s_axis_tdata(ws_tdata), .s_axis_tlast(ws_tlast),
        .s_axis_tvalid(ws_tvalid), .s_axis_tready(ws_tready),
        .m_axis_tdata(m_axis_tdata), .m_axis_tlast(m_axis_tlast),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready),
        .sat_event(ola_sat), .tlast_err(ola_terr)
    );

    assign sat_event = fft_sat | win_sat | ola_sat;
    assign tlast_err = fft_terr | ola_terr;

endmodule
