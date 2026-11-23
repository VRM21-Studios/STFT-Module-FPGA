`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_stft_framer
// Description : STFT analysis framer (sliding frame buffer).
//               Continuous stereo audio in, overlapping frames out:
//               frame f = x[f*H.. f*H + N - 1], hop H = N >> cfg_hop_log2
//               (cfg_hop_log2 = 1, 2, 3 -> H = N/2, N/4, N/8; v2).
//               - Ring buffer of 2*MAX_N words (one BRAM-friendly SDP RAM).
//               - The first frame is emitted after N samples, then one frame
//                 every H samples.
//               - Host view: after a reset, N - H samples prime the ring,
//                 then every H samples produce one frame.
//               - If the previous frame has not been fully read out when the
//                 next hop boundary is reached, s_axis_tready is deasserted
//                 (back-pressure instead of overwriting). Writes during a
//                 readout can never reach the frame being read because at
//                 most H-1 < N new samples are accepted before it completes.
//               - s_axis_tready depends on registers only.
//               - Output goes through vrm_axis_skid (registered tready towards
//                 this block, tdata zero while tvalid = 0), tlast at beat N-1.
//
// Data format : {R[15:0], L[15:0]} (passed through unchanged)
// ============================================================================

module vrm_stft_framer #(
    parameter MAX_N  = 2048,
    parameter DATA_W = 32
) (
    input  wire              clk,
    input  wire              rstn,

    input  wire [15:0]       cfg_fft_n,      // static while streaming (change only in reset)
    input  wire [1:0]        cfg_hop_log2,   // 1..3, static while streaming

    input  wire [DATA_W-1:0] s_axis_tdata,
    input  wire              s_axis_tvalid,
    output wire              s_axis_tready,

    output wire [DATA_W-1:0] m_axis_tdata,
    output wire              m_axis_tlast,
    output wire              m_axis_tvalid,
    input  wire              m_axis_tready,

    output reg  [31:0]       frame_count     // frames fully issued
);

    localparam AW = $clog2(2*MAX_N);         // ring address width
    localparam CW = $clog2(MAX_N) + 1;       // counter width (0..N)

    wire [CW-1:0] n_val = cfg_fft_n[CW-1:0];
    wire [CW-1:0] h_val = n_val >> cfg_hop_log2;

    // ------------------------------------------------------------------------
    // Write side
    // ------------------------------------------------------------------------
    reg [AW-1:0] wp;                         // next write address
    reg [CW-1:0] cnt_since;                  // samples since the last trigger
    reg          first;                      // waiting for the first full frame
    reg          pending;                    // a frame is waiting to be / being read
    reg [AW-1:0] frame_base;                 // ring address of sample 0 of that frame

    wire [CW-1:0] target  = first ? n_val : h_val;
    wire          at_trig = (cnt_since == target - 1'b1);
    assign s_axis_tready = !(pending && at_trig);
    wire          s_fire  = s_axis_tvalid && s_axis_tready;

    // ------------------------------------------------------------------------
    // Read side: one RAM-output stage + skid buffer
    // ------------------------------------------------------------------------
    wire          sk_ready;
    reg           rd_v, rd_last;
    reg  [CW-1:0] rd_cnt;                    // beats issued in the current frame
    wire          adv      = !rd_v || sk_ready;
    wire          issue    = adv && pending;
    wire          issue_ls = issue && (rd_cnt == n_val - 1'b1);
    wire [AW-1:0] rd_addr  = frame_base + rd_cnt[AW-1:0];
    wire [DATA_W-1:0] rd_data;

    vrm_ram_core #(.DATA_WIDTH(DATA_W), .ADDR_WIDTH(AW), .RAM_STYLE("block")) u_ring (
        .clk(clk), .rstn(rstn),
        .we(s_fire), .wr_addr(wp), .wr_data(s_axis_tdata),
        .re(issue), .rd_addr(rd_addr), .rd_data(rd_data)
    );

    always @(posedge clk) begin
        if (!rstn) begin
            wp <= 0; cnt_since <= 0; first <= 1'b1;
            pending <= 1'b0; frame_base <= 0;
            rd_v <= 1'b0; rd_last <= 1'b0; rd_cnt <= 0;
            frame_count <= 0;
        end else begin
            // write side
            if (s_fire) begin
                wp <= wp + 1'b1;
                if (at_trig) begin
                    cnt_since  <= 0;
                    first      <= 1'b0;
                    pending    <= 1'b1;
                    frame_base <= wp + 1'b1 - n_val;   // frame ends with this sample
                end else begin
                    cnt_since <= cnt_since + 1'b1;
                end
            end

            // read side
            if (adv) begin
                rd_v    <= issue;
                rd_last <= issue_ls;
            end
            if (issue) begin
                if (issue_ls) begin
                    rd_cnt      <= 0;
                    pending     <= 1'b0;               // trigger cannot fire while pending
                    frame_count <= frame_count + 1'b1;
                end else begin
                    rd_cnt <= rd_cnt + 1'b1;
                end
            end
        end
    end

    vrm_axis_skid #(.DATA_W(DATA_W)) u_out_skid (
        .clk(clk), .rstn(rstn),
        .s_axis_tdata(rd_data), .s_axis_tlast(rd_last),
        .s_axis_tvalid(rd_v), .s_axis_tready(sk_ready),
        .m_axis_tdata(m_axis_tdata), .m_axis_tlast(m_axis_tlast),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tready(m_axis_tready)
    );

endmodule
