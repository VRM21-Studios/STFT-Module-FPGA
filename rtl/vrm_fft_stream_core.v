`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_fft_stream_core
// Description : Frame-based AXI4-Stream wrapper around fft_stage_iterative.
//               LOAD (N beats in) -> RUN (in-place FFT/IFFT) -> OUTPUT (N beats
//               out, natural order) -> LOAD...
//               - Twiddle table is a ROM initialised from TWID_FILE
//                 (W_MAX_N, Q1.(TWID_W-1), word = {Wr, Wi}).
//               - cfg_mode / cfg_fft_n are sampled at the first beat of a frame
//                 and must stay static while a frame is in flight.
//               - Input frame length is counted (N beats); s_axis_tlast is
//                 checked and a mismatch is reported on tlast_err.
//               - m_axis_tdata is zero while m_axis_tvalid = 0.
//
// Data format : {re[DATA_W-1:0], im[DATA_W-1:0]}
// ============================================================================

module vrm_fft_stream_core #(
    parameter MAX_N       = 2048,
    parameter DATA_W      = 24,
    parameter TWID_W      = 18,
    parameter ROUND       = 1,
    parameter IFFT_SCALE  = 0,
    parameter TWID_STRIDE = 1,
    parameter TWID_FILE   = "twiddle_2048_q17.mem"
) (
    input  wire                clk,
    input  wire                rstn,

    input  wire                cfg_mode,       // 0 = FFT, 1 = IFFT
    input  wire [15:0]         cfg_fft_n,

    input  wire [2*DATA_W-1:0] s_axis_tdata,
    input  wire                s_axis_tlast,
    input  wire                s_axis_tvalid,
    output wire                s_axis_tready,

    output wire [2*DATA_W-1:0] m_axis_tdata,
    output wire                m_axis_tlast,
    output wire                m_axis_tvalid,
    input  wire                m_axis_tready,

    output wire                busy,           // transform running
    output reg                 frame_done,     // 1-cycle pulse after the last output beat
    output wire                sat_event,      // 1-cycle pulse from the butterfly
    output reg                 tlast_err,      // 1-cycle pulse: tlast position != N-1
    output wire                frame_partial   // input frame started but not complete (watchdog)
);

    localparam ADDR_DATA = $clog2(MAX_N);
    localparam ADDR_TWID = $clog2(MAX_N/2);

    localparam S_LOAD = 2'd0, S_RUN = 2'd1, S_PREFETCH = 2'd2, S_OUTPUT = 2'd3;
    reg [1:0] state;

    reg        cur_mode;
    reg [15:0] cur_n;
    reg [ADDR_DATA-1:0] in_cnt, out_cnt, rd_ptr;
    reg fft_start;

    // While waiting for the first beat the configuration is taken live
    wire [15:0]          n_sel  = (state == S_LOAD && in_cnt == 0) ? cfg_fft_n : cur_n;
    wire [ADDR_DATA-1:0] n_last = n_sel[ADDR_DATA:0] - 1'b1;

    assign s_axis_tready = (state == S_LOAD);
    wire s_fire = s_axis_tvalid && (state == S_LOAD);
    wire m_fire = m_axis_tvalid && m_axis_tready;

    // ------------------------------------------------------------------------
    // RAMs and FFT engine
    // ------------------------------------------------------------------------
    wire                 ram_we, ram_re;
    wire [ADDR_DATA-1:0] ram_wr_addr, ram_rd_addr;
    wire [2*DATA_W-1:0]  ram_wr_data, ram_rd_data;

    wire                 fft_ram_we;
    wire [ADDR_DATA-1:0] fft_ram_addr;
    wire [2*DATA_W-1:0]  fft_ram_wdata;
    wire                 fft_twid_re;
    wire [ADDR_TWID-1:0] fft_twid_addr;
    wire [2*TWID_W-1:0]  twid_rd_data;
    wire                 fft_done, fft_busy;

    vrm_ram_core #(.DATA_WIDTH(2*DATA_W), .ADDR_WIDTH(ADDR_DATA), .RAM_STYLE("block")) u_data_ram (
        .clk(clk), .rstn(rstn),
        .we(ram_we), .wr_addr(ram_wr_addr), .wr_data(ram_wr_data),
        .re(ram_re), .rd_addr(ram_rd_addr), .rd_data(ram_rd_data)
    );

    vrm_ram_core #(.DATA_WIDTH(2*TWID_W), .ADDR_WIDTH(ADDR_TWID), .RAM_STYLE("block"),
                   .INIT_FILE(TWID_FILE)) u_twid_rom (
        .clk(clk), .rstn(rstn),
        .we(1'b0), .wr_addr({ADDR_TWID{1'b0}}), .wr_data({(2*TWID_W){1'b0}}),
        .re(fft_twid_re), .rd_addr(fft_twid_addr), .rd_data(twid_rd_data)
    );

    fft_stage_iterative #(
        .MAX_N(MAX_N), .DATA_W(DATA_W), .TWID_W(TWID_W),
        .ROUND(ROUND), .IFFT_SCALE(IFFT_SCALE), .TWID_STRIDE(TWID_STRIDE)
    ) u_fft (
        .clk(clk), .rstn(rstn),
        .start(fft_start), .mode(cur_mode), .fft_n(cur_n),
        .ram_data_we(fft_ram_we), .ram_data_addr(fft_ram_addr),
        .ram_data_wdata(fft_ram_wdata), .ram_data_rdata(ram_rd_data),
        .ram_twid_re(fft_twid_re), .ram_twid_addr(fft_twid_addr),
        .ram_twid_rdata(twid_rd_data),
        .done(fft_done), .busy(fft_busy), .sat_event(sat_event)
    );

    assign busy = fft_busy;
    assign frame_partial = (state == S_LOAD) && (in_cnt != {ADDR_DATA{1'b0}});

    // ------------------------------------------------------------------------
    // Frame FSM
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rstn) begin
            state <= S_LOAD;
            cur_mode <= 1'b0; cur_n <= 16'd2048;
            in_cnt <= 0; out_cnt <= 0; rd_ptr <= 0;
            fft_start <= 1'b0; frame_done <= 1'b0; tlast_err <= 1'b0;
        end else begin
            fft_start  <= 1'b0;
            frame_done <= 1'b0;
            tlast_err  <= 1'b0;
            case (state)
                S_LOAD: begin
                    if (in_cnt == 0) begin
                        cur_mode <= cfg_mode;
                        cur_n    <= cfg_fft_n;
                    end
                    if (s_fire) begin
                        tlast_err <= s_axis_tlast ^ (in_cnt == n_last);
                        if (in_cnt == n_last) begin
                            in_cnt    <= 0;
                            fft_start <= 1'b1;
                            state     <= S_RUN;
                        end else begin
                            in_cnt <= in_cnt + 1'b1;
                        end
                    end
                end

                S_RUN: begin
                    if (fft_done) begin
                        out_cnt <= 0;
                        rd_ptr  <= 0;
                        state   <= S_PREFETCH;
                    end
                end

                S_PREFETCH: begin
                    rd_ptr <= 1;
                    state  <= S_OUTPUT;
                end

                S_OUTPUT: begin
                    if (m_fire) begin
                        rd_ptr <= rd_ptr + 1'b1;
                        if (out_cnt == n_last) begin
                            frame_done <= 1'b1;
                            state      <= S_LOAD;
                        end else begin
                            out_cnt <= out_cnt + 1'b1;
                        end
                    end
                end
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Memory multiplexing
    // ------------------------------------------------------------------------
    assign ram_we      = (state == S_LOAD) ? s_fire :
                         (state == S_RUN)  ? fft_ram_we : 1'b0;
    assign ram_wr_addr = (state == S_LOAD) ? in_cnt : fft_ram_addr;
    assign ram_wr_data = (state == S_LOAD) ? s_axis_tdata : fft_ram_wdata;

    assign ram_re      = (state == S_RUN && !fft_ram_we) ||
                         (state == S_PREFETCH) ||
                         (state == S_OUTPUT && m_axis_tready);
    assign ram_rd_addr = (state == S_RUN) ? fft_ram_addr : rd_ptr;

    // ------------------------------------------------------------------------
    // AXI4-Stream master
    // ------------------------------------------------------------------------
    assign m_axis_tvalid = (state == S_OUTPUT);
    assign m_axis_tdata  = (state == S_OUTPUT) ? ram_rd_data : {(2*DATA_W){1'b0}};
    assign m_axis_tlast  = (state == S_OUTPUT) && (out_cnt == n_last);

endmodule
