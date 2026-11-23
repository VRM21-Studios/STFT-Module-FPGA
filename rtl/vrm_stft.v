`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_stft
// Description : Stereo STFT analysis, hop H = N >> cfg_hop_log2 (N/2, N/4,
//               N/8; v2), periodic sqrt-Hann window.
//
//   s_axis (audio {R,L} Q1.15)
//     -> vrm_stft_framer      : overlapping frames x[f*H.. f*H+N-1]
//     -> vrm_windowing_core   : x * w_a, Q1.15 -> DATA_W bits at scale 2^(DATA_W-2)
//     -> lane mapping         : z = L + jR   (two real signals in one complex FFT)
//     -> vrm_fft_stream_core  : forward FFT, /2 per stage -> Z = DFT(z)/N
//     -> m_axis (spectrum {re, im}, N bins per frame, natural order, tlast on bin N-1)
//
//   Per-channel spectra (if needed by a mask stage):
//     X_L[k] = (Z[k] + conj(Z[N-k])) / 2,   X_R[k] = (Z[k] - conj(Z[N-k])) / (2j)
//
// Numeric scale : full-scale audio (1.0) = 2^(DATA_W-2) inside the datapath
//                 (one guard bit), Z = DFT{w*x}/N at that scale.
// ============================================================================

module vrm_stft #(
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

    input  wire [31:0]         s_axis_tdata,   // {R, L}
    input  wire                s_axis_tvalid,
    output wire                s_axis_tready,

    output wire [2*DATA_W-1:0] m_axis_tdata,   // {re, im}
    output wire                m_axis_tlast,
    output wire                m_axis_tvalid,
    input  wire                m_axis_tready,

    output wire [31:0]         frame_count,
    output wire                busy,
    output wire                sat_event,
    output wire                tlast_err
);

    // window shift: Q1.15 * Q1.(COEF_W-1) -> scale 2^(DATA_W-2)
    localparam integer WIN_SHIFT = 15 + (COEF_W - 1) - (DATA_W - 2);

    wire [31:0]         fr_tdata;
    wire                fr_tlast, fr_tvalid, fr_tready;
    wire [2*DATA_W-1:0] wn_tdata;
    wire                wn_tlast, wn_tvalid, wn_tready;
    wire                win_sat, fft_sat;

    vrm_stft_framer #(.MAX_N(MAX_N), .DATA_W(32)) u_framer (
        .clk(clk), .rstn(rstn), .cfg_fft_n(cfg_fft_n), .cfg_hop_log2(cfg_hop_log2),
        .s_axis_tdata(s_axis_tdata), .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
        .m_axis_tdata(fr_tdata), .m_axis_tlast(fr_tlast),
        .m_axis_tvalid(fr_tvalid), .m_axis_tready(fr_tready),
        .frame_count(frame_count)
    );

    vrm_windowing_core #(
        .MAX_PTS(MAX_N), .IN_W(16), .OUT_W(DATA_W), .COEF_W(COEF_W),
        .SHIFT(WIN_SHIFT), .ROUND(1), .ROM_FILE(WIN_FILE)
    ) u_win (
        .aclk(clk), .aresetn(rstn), .cfg_fft_n(cfg_fft_n),
        .s_axis_tdata(fr_tdata), .s_axis_tlast(fr_tlast),
        .s_axis_tvalid(fr_tvalid), .s_axis_tready(fr_tready),
        .m_axis_tdata(wn_tdata), .m_axis_tlast(wn_tlast),
        .m_axis_tvalid(wn_tvalid), .m_axis_tready(wn_tready),
        .sat_event(win_sat)
    );

    // lane0 = L -> re, lane1 = R -> im
    wire [2*DATA_W-1:0] z_tdata = {wn_tdata[DATA_W-1:0], wn_tdata[2*DATA_W-1:DATA_W]};

    vrm_fft_stream_core #(
        .MAX_N(MAX_N), .DATA_W(DATA_W), .TWID_W(TWID_W),
        .ROUND(1), .IFFT_SCALE(0), .TWID_STRIDE(1), .TWID_FILE(TWID_FILE)
    ) u_fft (
        .clk(clk), .rstn(rstn), .cfg_mode(1'b0), .cfg_fft_n(cfg_fft_n),
        .s_axis_tdata(z_tdata), .s_axis_tlast(wn_tlast),
        .s_axis_tvalid(wn_tvalid), .s_axis_tready(wn_tready),
        .m_axis_tdata(m_axis_tdata), .m_axis_tlast(m_axis_tlast),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready),
        .busy(busy), .frame_done(), .sat_event(fft_sat), .tlast_err(tlast_err),
        .frame_partial()
    );

    assign sat_event = win_sat | fft_sat;

endmodule
