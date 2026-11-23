`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_windowing_core
// Description : Two-lane AXI4-Stream window multiplier.
//               out_lane = sat_OUT_W( round( in_lane * w[n], SHIFT ) )
//               - Window ROM always holds MAX_PTS entries (periodic window);
//                 for N = MAX_PTS/s the ROM is read with stride s.
//               - Sample index resets on tlast or when it reaches N-1.
//               - Coefficients are unsigned Q1.(COEF_W-1).
//               - Output is registered through vrm_axis_skid: s_axis_tready is
//                 a register output and m_axis_tdata is zero while tvalid = 0.
//
// Data format : s_axis_tdata = {lane1[IN_W-1:0],  lane0[IN_W-1:0]}
//               m_axis_tdata = {lane1[OUT_W-1:0], lane0[OUT_W-1:0]}
//               (audio convention: lane0 = L, lane1 = R)
//
// Typical use :
//   STFT analysis : IN_W=16, OUT_W=24, COEF_W=18, SHIFT=10, ROUND=1
//                   (Q1.15 audio -> 24-bit word at scale 2^22, 1 guard bit)
//   iSTFT synth.  : IN_W=24, OUT_W=24, COEF_W=18, SHIFT=17, ROUND=1
//   Original core : IN_W=16, OUT_W=16, COEF_W=16, SHIFT=15, ROUND=0
// ============================================================================

module vrm_windowing_core #(
    parameter integer MAX_PTS = 2048,
    parameter integer IN_W    = 16,
    parameter integer OUT_W   = 16,
    parameter integer COEF_W  = 16,
    parameter integer SHIFT   = 15,
    parameter integer ROUND   = 0,
    parameter ROM_FILE = "window_2048.mem"
)(
    input  wire aclk,
    input  wire aresetn,

    input  wire [15:0] cfg_fft_n,             // 256 / 512 / 1024 / 2048 (static while streaming)

    input  wire [2*IN_W-1:0]  s_axis_tdata,
    input  wire               s_axis_tlast,
    input  wire               s_axis_tvalid,
    output wire               s_axis_tready,

    output wire [2*OUT_W-1:0] m_axis_tdata,
    output wire               m_axis_tlast,
    output wire               m_axis_tvalid,
    input  wire               m_axis_tready,

    output reg                sat_event        // 1-cycle pulse: an output sample saturated
);

    localparam ADDR_W = $clog2(MAX_PTS);
    localparam PW     = IN_W + COEF_W + 1;
    localparam integer RND = (ROUND != 0 && SHIFT > 0) ? (1 << (SHIFT - 1)) : 0;

    // ------------------------------------------------------------------------
    // 1. Window ROM (block RAM, synchronous read)
    // ------------------------------------------------------------------------
    (* rom_style = "block" *)
    reg [COEF_W-1:0] window_rom [0:MAX_PTS-1];

    initial begin
        $readmemh(ROM_FILE, window_rom);
    end

    // ROM stride for N < MAX_PTS
    reg [3:0] step_sh;
    always @(*) begin
        case (cfg_fft_n)
            16'd256:  step_sh = ADDR_W - 8;
            16'd512:  step_sh = ADDR_W - 9;
            16'd1024: step_sh = ADDR_W - 10;
            default:  step_sh = ADDR_W - 11;
        endcase
    end

    // ------------------------------------------------------------------------
    // 2. Pipeline control (advance only when the output skid can accept)
    // ------------------------------------------------------------------------
    wire sk_ready;
    wire en     = sk_ready;
    wire s_fire = s_axis_tvalid && en;
    assign s_axis_tready = en;

    reg [ADDR_W-1:0] sample_idx;
    wire [ADDR_W-1:0] n_last   = cfg_fft_n[ADDR_W:0] - 1'b1;
    wire [ADDR_W-1:0] rom_addr = sample_idx << step_sh;

    always @(posedge aclk) begin
        if (!aresetn) begin
            sample_idx <= 0;
        end else if (s_fire) begin
            if (s_axis_tlast || sample_idx == n_last) sample_idx <= 0;
            else                                      sample_idx <= sample_idx + 1'b1;
        end
    end

    // ------------------------------------------------------------------------
    // 3. Stage 1: ROM fetch and data alignment
    // ------------------------------------------------------------------------
    reg signed [IN_W-1:0] s1_l, s1_r;
    reg [COEF_W-1:0]      s1_coeff;
    reg                   s1_valid, s1_last;

    always @(posedge aclk) begin
        if (!aresetn) begin
            s1_l <= 0; s1_r <= 0; s1_coeff <= 0; s1_valid <= 0; s1_last <= 0;
        end else if (en) begin
            s1_valid <= s_axis_tvalid;
            s1_last  <= s_axis_tlast;
            if (s_axis_tvalid) begin
                s1_l     <= s_axis_tdata[IN_W-1:0];
                s1_r     <= s_axis_tdata[2*IN_W-1:IN_W];
                s1_coeff <= window_rom[rom_addr];
            end
        end
    end

    // ------------------------------------------------------------------------
    // 4. Stage 2: multiply, round, saturate
    // ------------------------------------------------------------------------
    function [OUT_W-1:0] sat_o;
        input signed [PW-1:0] v;
        begin
            if (v > $signed({{(PW-OUT_W){1'b0}}, 1'b0, {(OUT_W-1){1'b1}}}))
                sat_o = {1'b0, {(OUT_W-1){1'b1}}};
            else if (v < $signed({{(PW-OUT_W){1'b1}}, 1'b1, {(OUT_W-1){1'b0}}}))
                sat_o = {1'b1, {(OUT_W-1){1'b0}}};
            else
                sat_o = v[OUT_W-1:0];
        end
    endfunction

    (* use_dsp = "yes" *) wire signed [PW-1:0] mult_l = s1_l * $signed({1'b0, s1_coeff});
    (* use_dsp = "yes" *) wire signed [PW-1:0] mult_r = s1_r * $signed({1'b0, s1_coeff});
    wire signed [PW-1:0] shr_l  = (mult_l + RND) >>> SHIFT;
    wire signed [PW-1:0] shr_r  = (mult_r + RND) >>> SHIFT;
    wire signed [OUT_W-1:0] q_l = sat_o(shr_l);
    wire signed [OUT_W-1:0] q_r = sat_o(shr_r);

    reg signed [OUT_W-1:0] s2_l, s2_r;
    reg                    s2_valid, s2_last;

    always @(posedge aclk) begin
        if (!aresetn) begin
            s2_l <= 0; s2_r <= 0; s2_valid <= 0; s2_last <= 0; sat_event <= 0;
        end else begin
            sat_event <= 1'b0;
            if (en) begin
                s2_valid  <= s1_valid;
                s2_last   <= s1_last;
                s2_l      <= q_l;
                s2_r      <= q_r;
                sat_event <= s1_valid && ((q_l != shr_l) || (q_r != shr_r));
            end
        end
    end

    // ------------------------------------------------------------------------
    // 5. Output register slice
    // ------------------------------------------------------------------------
    vrm_axis_skid #(.DATA_W(2*OUT_W)) u_out_skid (
        .clk(aclk), .rstn(aresetn),
        .s_axis_tdata({s2_r, s2_l}), .s_axis_tlast(s2_last),
        .s_axis_tvalid(s2_valid && en), .s_axis_tready(sk_ready),
        .m_axis_tdata(m_axis_tdata), .m_axis_tlast(m_axis_tlast),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready)
    );

endmodule
