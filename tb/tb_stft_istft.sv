`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Testbench   : tb_stft_istft
// Description : vrm_stft -> (identity mask) -> vrm_istft loopback (v2: +HOP).
//               - Stereo stimulus from a $readmemh file ({R,L} per line).
//               - Random tvalid gaps on the audio input, random throttling on
//                 the spectrum link, random tready on the audio output.
//               - CSV outputs:
//                   +OUT=<file>   idx,L,R            (iSTFT audio output)
//                   +SPEC=<file>  frame,k,re,im      (first SPEC_FRAMES frames)
//                   +PERF=<file>  frame,stft_cycle,istft_cycle (frame end stamps)
//               - Ends after nfr*H output samples, nfr = (NSAMP-N)/H + 1,
//                 H = N >> HOP (HOP = 1, 2, 3 -> N/2, N/4, N/8).
// Plusargs    : +STIM=<hex> +NSAMP=<n> +N=<fft size> +HOP=<1|2|3> +OUT=.. +SPEC=.. +PERF=..
//               +SPEC_FRAMES=<n> +STRESS=<0|1>
// ============================================================================

module tb_stft_istft;

    parameter int    MAX_N     = 2048;
    parameter string WIN_FILE  = "window_2048_q17.mem";
    parameter string TWID_FILE = "twiddle_2048_q17.mem";
    localparam int   DATA_W    = 24;
    localparam int   MAX_STIM  = 1 << 18;

    // ------------------------------------------------------------------ clock
    logic clk = 1'b0;
    logic rstn = 1'b0;
    always #5 clk = ~clk;                    // 100 MHz

    // ------------------------------------------------------------------ config
    string stim_file, out_file, spec_file, perf_file;
    int    nsamp, n_fft, hop, hop_log2, nfr, n_out_exp, spec_frames, stress;
    logic [31:0] stim [0:MAX_STIM-1];
    logic [15:0] cfg_fft_n;
    logic [1:0]  cfg_hop_log2;

    // ------------------------------------------------------------------ DUT wiring
    logic [31:0]         a_tdata;
    logic                a_tvalid, a_tready;
    logic [2*DATA_W-1:0] z_tdata;
    logic                z_tlast, z_tvalid, z_tready;
    logic                y_tvalid, y_tready;
    logic [31:0]         o_tdata;
    logic                o_tlast, o_tvalid, o_tready;
    logic [31:0]         frame_count;
    logic                stft_busy, istft_busy, stft_sat, istft_sat, stft_terr, istft_terr;

    logic in_gate, link_gate, out_gate;

    vrm_stft #(.MAX_N(MAX_N), .WIN_FILE(WIN_FILE), .TWID_FILE(TWID_FILE)) u_stft (
        .clk(clk), .rstn(rstn), .cfg_fft_n(cfg_fft_n), .cfg_hop_log2(cfg_hop_log2),
        .s_axis_tdata(a_tdata), .s_axis_tvalid(a_tvalid), .s_axis_tready(a_tready),
        .m_axis_tdata(z_tdata), .m_axis_tlast(z_tlast),
        .m_axis_tvalid(z_tvalid), .m_axis_tready(z_tready),
        .frame_count(frame_count), .busy(stft_busy), .sat_event(stft_sat), .tlast_err(stft_terr)
    );

    // identity mask: spectrum link with random throttling
    assign y_tvalid = z_tvalid & link_gate;
    assign z_tready = y_tready & link_gate;

    vrm_istft #(.MAX_N(MAX_N), .WIN_FILE(WIN_FILE), .TWID_FILE(TWID_FILE)) u_istft (
        .clk(clk), .rstn(rstn), .cfg_fft_n(cfg_fft_n), .cfg_hop_log2(cfg_hop_log2),
        .s_axis_tdata(z_tdata), .s_axis_tlast(z_tlast),
        .s_axis_tvalid(y_tvalid), .s_axis_tready(y_tready),
        .m_axis_tdata(o_tdata), .m_axis_tlast(o_tlast),
        .m_axis_tvalid(o_tvalid), .m_axis_tready(o_tready),
        .busy(istft_busy), .sat_event(istft_sat), .tlast_err(istft_terr), .frame_partial()
    );

    // ------------------------------------------------------------------ stimulus
    int in_idx = 0;
    assign a_tvalid = (in_idx < nsamp) && in_gate && rstn;
    assign a_tdata  = a_tvalid ? stim[in_idx] : 32'h0;
    assign o_tready = out_gate;

    always @(posedge clk) begin
        if (stress != 0) begin
            in_gate   <= ($urandom_range(0, 99) < 85);
            link_gate <= ($urandom_range(0, 99) < 80);
            out_gate  <= ($urandom_range(0, 99) < 70);
        end else begin
            in_gate <= 1'b1; link_gate <= 1'b1; out_gate <= 1'b1;
        end
        if (a_tvalid && a_tready) in_idx <= in_idx + 1;
    end

    // ------------------------------------------------------------------ monitors
    int fo, fs, fp;
    longint cycle = 0;
    int out_cnt = 0, spec_frame = 0, spec_k = 0, istft_frames = 0;
    int sat_cnt = 0, terr_cnt = 0, proto_err = 0;
    logic [31:0] o_hold; logic o_stall;
    // explicitly sign-extended 32-bit views for CSV printing
    int o_l, o_r, z_re, z_im;
    always @(*) begin
        o_l  = {{16{o_tdata[15]}}, o_tdata[15:0]};
        o_r  = {{16{o_tdata[31]}}, o_tdata[31:16]};
        z_re = {{(32-DATA_W){z_tdata[2*DATA_W-1]}}, z_tdata[2*DATA_W-1:DATA_W]};
        z_im = {{(32-DATA_W){z_tdata[DATA_W-1]}},   z_tdata[DATA_W-1:0]};
    end

    always @(posedge clk) begin
        cycle <= cycle + 1;
        if (stft_sat || istft_sat)   sat_cnt  <= sat_cnt + 1;
        if (stft_terr || istft_terr) terr_cnt <= terr_cnt + 1;

        // AXI4-Stream rule check on the output: data stable while stalled
        if (o_stall && (!o_tvalid || o_tdata != o_hold)) proto_err <= proto_err + 1;
        o_stall <= o_tvalid && !o_tready;
        o_hold  <= o_tdata;
        if (!o_tvalid && o_tdata != 32'h0) proto_err <= proto_err + 1;   // 

        if (o_tvalid && o_tready) begin
            $fdisplay(fo, "%0d,%0d,%0d", out_cnt, o_l, o_r);
            out_cnt <= out_cnt + 1;
            if (o_tlast) begin
                $fdisplay(fp, "%0d,istft,%0d", istft_frames, cycle);
                istft_frames <= istft_frames + 1;
            end
        end

        if (y_tvalid && y_tready) begin
            if (spec_frame < spec_frames)
                $fdisplay(fs, "%0d,%0d,%0d,%0d", spec_frame, spec_k, z_re, z_im);
            if (z_tlast) begin
                $fdisplay(fp, "%0d,stft,%0d", spec_frame, cycle);
                spec_frame <= spec_frame + 1;
                spec_k <= 0;
            end else begin
                spec_k <= spec_k + 1;
            end
        end
    end

    // ------------------------------------------------------------------ control
    initial begin
        if (!$value$plusargs("STIM=%s", stim_file)) stim_file = "stim_sine.mem";
        if (!$value$plusargs("NSAMP=%d", nsamp)) nsamp = 48000;
        if (!$value$plusargs("N=%d", n_fft)) n_fft = 2048;
        if (!$value$plusargs("HOP=%d", hop_log2)) hop_log2 = 1;
        if (!$value$plusargs("OUT=%s", out_file)) out_file = "out.csv";
        if (!$value$plusargs("SPEC=%s", spec_file)) spec_file = "spec.csv";
        if (!$value$plusargs("PERF=%s", perf_file)) perf_file = "perf.csv";
        if (!$value$plusargs("SPEC_FRAMES=%d", spec_frames)) spec_frames = 3;
        if (!$value$plusargs("STRESS=%d", stress)) stress = 1;

        hop       = n_fft >> hop_log2;
        cfg_hop_log2 = hop_log2[1:0];
        nfr       = (nsamp - n_fft) / hop + 1;
        n_out_exp = nfr * hop;
        cfg_fft_n = n_fft[15:0];
        in_gate = 1'b0; link_gate = 1'b0; out_gate = 1'b0; o_stall = 1'b0; o_hold = 0;

        $readmemh(stim_file, stim, 0, nsamp - 1);
        fo = $fopen(out_file, "w");  $fdisplay(fo, "idx,L,R");
        fs = $fopen(spec_file, "w"); $fdisplay(fs, "frame,k,re,im");
        fp = $fopen(perf_file, "w"); $fdisplay(fp, "frame,block,cycle");
        $display("[TB] N=%0d hop=%0d (code %0d) nsamp=%0d frames=%0d expected outputs=%0d stress=%0d",
                 n_fft, hop, hop_log2, nsamp, nfr, n_out_exp, stress);
        $display("[TB] window=%s twiddle=%s", WIN_FILE, TWID_FILE);

        repeat (10) @(posedge clk);
        rstn = 1'b1;

        fork
            begin
                wait (out_cnt == n_out_exp);
                repeat (20) @(posedge clk);
            end
            begin
                repeat (400_000_000) @(posedge clk);
                $display("[TB] TIMEOUT: out_cnt=%0d / %0d", out_cnt, n_out_exp);
            end
        join_any

        $fclose(fo); $fclose(fs); $fclose(fp);
        $display("[TB] done: cycles=%0d outputs=%0d stft_frames=%0d istft_frames=%0d",
                 cycle, out_cnt, frame_count, istft_frames);
        $display("[TB] sat_events=%0d tlast_errors=%0d axis_protocol_errors=%0d",
                 sat_cnt, terr_cnt, proto_err);
        if (out_cnt == n_out_exp && terr_cnt == 0 && proto_err == 0) $display("[TB] RESULT: PASS (stream level)");
        else                                                        $display("[TB] RESULT: FAIL");
        $finish;
    end

endmodule
