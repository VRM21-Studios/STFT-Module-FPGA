`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_overlap_add (v2)
// Description : iSTFT overlap-add for hop H = N >> cfg_hop_log2
//               (cfg_hop_log2 = 1, 2, 3 -> H = N/2, N/4, N/8, i.e. K = N/H =
//               2, 4, 8 overlapping frames).
//               Input : N windowed time-domain samples per frame (2 lanes).
//               Output: H samples per frame:
//                   out[f*H + i] = sum_{j=0..K-1} frame_{f-j}[i + j*H],  i = 0..H-1
//
//   Accumulator : one N-word circular buffer (SDP RAM, ACC_W bits per lane).
//                 Frame f, sample n (n = 0..N-1), address a = (ptr + n) mod N:
//                   sum = acc[a] + y[n]
//                   n <  H : sum is complete -> output, acc[a] <= 0
//                   n >= H : acc[a] <= sum
//                 After the frame, ptr <= ptr + H. Every address read by frame
//                 f was written by frame f-1 (sum or 0), so only frame 0 needs
//                 its read data masked to zero (no RAM clear after reset).
//                 Within a frame all addresses differ; the closest read-after-
//                 write distance between frames is N-H-1 beats, so the
//                 read-modify-write needs no forwarding.
//   Scaling     : periodic sqrt-Hann analysis and synthesis windows:
//                 sum_j w^2[n + jH] = N / (2H) = 2^(cfg_hop_log2 - 1).
//                 Output = round-half-up(sum >>> (OUT_SHIFT + cfg_hop_log2 - 1)),
//                 saturated to OUT_W (gain 1 for every hop).
//   Pipeline    : S0 accept + RAM read, S1 add + write-back, S2 output
//                 register, then vrm_axis_skid. One global enable
//                 (en = !s2_v || skid ready); s_axis_tready = en depends on
//                 registers only. tdata zero while tvalid = 0, tlast on the
//                 last of the H output beats of each frame.
//   Fade-in     : after a reset the first K-1 output hops contain fewer than
//                 K frames (fade-in of the first frames).
//
// Data format : s_axis_tdata = {lane1[IN_W-1:0],  lane0[IN_W-1:0]}
//               m_axis_tdata = {lane1[OUT_W-1:0], lane0[OUT_W-1:0]}
// ============================================================================

module vrm_overlap_add #(
    parameter MAX_N        = 2048,
    parameter IN_W         = 24,
    parameter OUT_W        = 16,
    parameter OUT_SHIFT    = 7,
    parameter MAX_HOP_LOG2 = 3
) (
    input  wire               clk,
    input  wire               rstn,

    input  wire [15:0]        cfg_fft_n,      // static while streaming
    input  wire [1:0]         cfg_hop_log2,   // 1..MAX_HOP_LOG2, static while streaming

    input  wire [2*IN_W-1:0]  s_axis_tdata,
    input  wire               s_axis_tlast,
    input  wire               s_axis_tvalid,
    output wire               s_axis_tready,

    output wire [2*OUT_W-1:0] m_axis_tdata,
    output wire               m_axis_tlast,
    output wire               m_axis_tvalid,
    input  wire               m_axis_tready,

    output reg                sat_event,      // 1-cycle pulse: an output sample saturated
    output reg                tlast_err       // 1-cycle pulse: tlast position != N-1
);

    localparam ADDR_W = $clog2(MAX_N);
    localparam ACC_W  = IN_W + MAX_HOP_LOG2;  // up to 2^MAX_HOP_LOG2 frames

    wire [ADDR_W:0]   n_full = cfg_fft_n[ADDR_W:0];
    wire [ADDR_W-1:0] n_mask = n_full - 1'b1;            // N - 1
    wire [ADDR_W:0]   h_full = n_full >> cfg_hop_log2;
    wire [ADDR_W-1:0] h_val  = h_full[ADDR_W-1:0];

    // ------------------------------------------------------------------------
    // Global enable
    // ------------------------------------------------------------------------
    wire sk_ready;
    reg  s2_v;
    wire en = !s2_v || sk_ready;
    assign s_axis_tready = en;
    wire s_fire = s_axis_tvalid && en;

    // ------------------------------------------------------------------------
    // S0: index, address, RAM read
    // ------------------------------------------------------------------------
    reg  [ADDR_W-1:0] idx;
    reg  [ADDR_W-1:0] ptr;
    reg               first_frame;

    wire [ADDR_W-1:0] rd_addr = (ptr + idx) & n_mask;
    wire [2*ACC_W-1:0] acc_rd;

    reg               s1_v, s1_out, s1_olast, s1_zero;
    reg  [ADDR_W-1:0] s1_addr;
    reg  [2*IN_W-1:0] s1_d;

    // ------------------------------------------------------------------------
    // S1: add, write back
    // ------------------------------------------------------------------------
    wire signed [IN_W-1:0]  cur_l  = s1_d[IN_W-1:0];
    wire signed [IN_W-1:0]  cur_r  = s1_d[2*IN_W-1:IN_W];
    wire signed [ACC_W-1:0] prev_l = s1_zero ? {ACC_W{1'b0}} : acc_rd[ACC_W-1:0];
    wire signed [ACC_W-1:0] prev_r = s1_zero ? {ACC_W{1'b0}} : acc_rd[2*ACC_W-1:ACC_W];
    wire signed [ACC_W-1:0] sum_l  = prev_l + {{(ACC_W-IN_W){cur_l[IN_W-1]}}, cur_l};
    wire signed [ACC_W-1:0] sum_r  = prev_r + {{(ACC_W-IN_W){cur_r[IN_W-1]}}, cur_r};

    wire               acc_we = s1_v && en;
    wire [2*ACC_W-1:0] acc_wd = s1_out ? {2*ACC_W{1'b0}} : {sum_r, sum_l};

    vrm_ram_core #(.DATA_WIDTH(2*ACC_W), .ADDR_WIDTH(ADDR_W), .RAM_STYLE("block")) u_acc (
        .clk(clk), .rstn(rstn),
        .we(acc_we), .wr_addr(s1_addr), .wr_data(acc_wd),
        .re(s_fire), .rd_addr(rd_addr), .rd_data(acc_rd)
    );

    reg signed [ACC_W-1:0] s2_l, s2_r;
    reg                    s2_last;

    always @(posedge clk) begin
        if (!rstn) begin
            idx <= 0; ptr <= 0; first_frame <= 1'b1;
            s1_v <= 1'b0; s1_out <= 1'b0; s1_olast <= 1'b0; s1_zero <= 1'b0;
            s1_addr <= 0; s1_d <= 0;
            s2_v <= 1'b0; s2_l <= 0; s2_r <= 0; s2_last <= 1'b0;
            tlast_err <= 1'b0;
        end else begin
            tlast_err <= 1'b0;
            if (en) begin
                // S0 -> S1
                s1_v <= s_fire;
                if (s_fire) begin
                    s1_d      <= s_axis_tdata;
                    s1_addr   <= rd_addr;
                    s1_out    <= (idx < h_val);
                    s1_olast  <= (idx == h_val - 1'b1);
                    s1_zero   <= first_frame;
                    tlast_err <= s_axis_tlast ^ (idx == n_mask);
                    if (idx == n_mask) begin
                        idx         <= 0;
                        ptr         <= (ptr + h_val) & n_mask;
                        first_frame <= 1'b0;
                    end else begin
                        idx <= idx + 1'b1;
                    end
                end
                // S1 -> S2 (complete samples only)
                s2_v <= s1_v && s1_out;
                if (s1_v && s1_out) begin
                    s2_l    <= sum_l;
                    s2_r    <= sum_r;
                    s2_last <= s1_olast;
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // S2: round-half-up, hop-dependent shift, saturate
    // ------------------------------------------------------------------------
    localparam integer SH1 = OUT_SHIFT;        // K = 2
    localparam integer SH2 = OUT_SHIFT + 1;    // K = 4
    localparam integer SH3 = OUT_SHIFT + 2;    // K = 8

    function signed [ACC_W:0] rnd_shr;
        input signed [ACC_W-1:0] v;
        input [1:0]              hl;
        reg   signed [ACC_W:0]   e;
        begin
            e = {v[ACC_W-1], v};
            case (hl)
                2'd2:    rnd_shr = (e + (1 << (SH2 - 1))) >>> SH2;
                2'd3:    rnd_shr = (e + (1 << (SH3 - 1))) >>> SH3;
                default: rnd_shr = (e + (1 << (SH1 - 1))) >>> SH1;
            endcase
        end
    endfunction

    function [OUT_W-1:0] sat_o;
        input signed [ACC_W:0] v;
        begin
            if (v > $signed({{(ACC_W+2-OUT_W){1'b0}}, {(OUT_W-1){1'b1}}}))
                sat_o = {1'b0, {(OUT_W-1){1'b1}}};
            else if (v < $signed({{(ACC_W+2-OUT_W){1'b1}}, {(OUT_W-1){1'b0}}}))
                sat_o = {1'b1, {(OUT_W-1){1'b0}}};
            else
                sat_o = v[OUT_W-1:0];
        end
    endfunction

    wire signed [ACC_W:0]   shr_l = rnd_shr(s2_l, cfg_hop_log2);
    wire signed [ACC_W:0]   shr_r = rnd_shr(s2_r, cfg_hop_log2);
    wire signed [OUT_W-1:0] q_l   = sat_o(shr_l);
    wire signed [OUT_W-1:0] q_r   = sat_o(shr_r);

    wire push = s2_v && sk_ready;

    always @(posedge clk) begin
        if (!rstn) sat_event <= 1'b0;
        else       sat_event <= push && (($signed(q_l) != shr_l) || ($signed(q_r) != shr_r));
    end

    vrm_axis_skid #(.DATA_W(2*OUT_W)) u_out_skid (
        .clk(clk), .rstn(rstn),
        .s_axis_tdata(s2_v ? {q_r, q_l} : {2*OUT_W{1'b0}}), .s_axis_tlast(s2_v && s2_last),
        .s_axis_tvalid(s2_v), .s_axis_tready(sk_ready),
        .m_axis_tdata(m_axis_tdata), .m_axis_tlast(m_axis_tlast),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready)
    );

endmodule
