// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : fft_stage_iterative
// Description : Iterative radix-2 DIF FFT/IFFT engine, in-place on an external
//               single-clock simple dual-port RAM, one shared multiplier,
//               in-place bit-reversal at the end (natural-order output).
//
// Numerics (selected by parameters, defaults reproduce the original core):
//   DATA_W      : complex component width (RAM word = {re, im}, 2*DATA_W bits)
//   TWID_W      : twiddle component width, Q1.(TWID_W-1)
//                 (RAM word = {Wr, Wi}, 2*TWID_W bits)
//   ROUND       : 0 = floor (arithmetic shift), 1 = round-half-up on every
//                 right shift (sum/diff >>>1 and the twiddle product shift)
//   IFFT_SCALE  : 1 = IFFT also divides by 2 per stage (original behaviour,
//                 FFT->IFFT round trip = x/N)
//                 0 = IFFT is unscaled (FFT = DFT/N, IFFT = IDFT*N/N,
//                 round trip = x). The forward FFT always scales by 1/2.
//   TWID_STRIDE : 0 = twiddle table must hold W_N of the active N
//                 (original contract, address = j << stage)
//                 1 = twiddle table always holds W_MAX_N, address is strided
//                 by MAX_N/N: ((j << stage) << (log2(MAX_N) - log2(N)))
//
// Saturation: both butterfly outputs are saturated to DATA_W bits. With the
// default parameters the A output can never exceed DATA_W bits, so the
// original behaviour (saturation on B only) is reproduced bit-exactly.
//
// Original defaults (DATA_W=16, TWID_W=16, ROUND=0, IFFT_SCALE=1,
// TWID_STRIDE=0) are bit-exact with the original RTL.
// Recommended STFT setting: DATA_W=24, TWID_W=18, ROUND=1, IFFT_SCALE=0,
// TWID_STRIDE=1.
// ============================================================================
`timescale 1ns / 1ps

module fft_stage_iterative #(
    parameter MAX_N       = 2048,
    parameter DATA_W      = 16,
    parameter TWID_W      = 16,
    parameter ROUND       = 0,
    parameter IFFT_SCALE  = 1,
    parameter TWID_STRIDE = 0
) (
    input  wire clk, rstn,
    input  wire start, mode,                 // mode: 0 = FFT, 1 = IFFT
    input  wire [15:0] fft_n,
    output reg  ram_data_we,
    output reg  [$clog2(MAX_N)-1:0] ram_data_addr,
    output reg  [2*DATA_W-1:0] ram_data_wdata,
    input  wire [2*DATA_W-1:0] ram_data_rdata,
    output reg  ram_twid_re,
    output reg  [$clog2(MAX_N/2)-1:0] ram_twid_addr,
    input  wire [2*TWID_W-1:0] ram_twid_rdata,
    output reg  done, busy,
    output reg  sat_event                    // 1-cycle pulse: a butterfly output saturated
);

    localparam ADDR_W = $clog2(MAX_N);
    localparam SW     = DATA_W + 1;          // sum / difference width
    localparam PW     = SW + TWID_W;         // product width
    localparam CW     = PW + 1;              // combined product width
    localparam integer RND1  = (ROUND != 0) ? 1 : 0;
    localparam integer RNDTW = (ROUND != 0) ? (1 << (TWID_W - 2)) : 0;

    // State machine encoding (unchanged from the original core)
    localparam S_IDLE=0, S_INIT=1, S_STAGE_LOOP=2, S_J_LOOP=3, S_K_LOOP=4,
               S_READ_A=5, S_WAIT_RD_A=6, S_CAP_A=7,
               S_READ_TW=8, S_WAIT_TW=9, S_CAP_TW=10,
               S_READ_B=11, S_WAIT_RD_B=12, S_CAP_B=13,
               S_CALC=14,
               S_MUL1_LOAD=15, S_MUL1_WAIT=16, S_MUL1_CAP=17,
               S_MUL2_LOAD=18, S_MUL2_WAIT=19, S_MUL2_CAP=20,
               S_MUL3_LOAD=21, S_MUL3_WAIT=22, S_MUL3_CAP=23,
               S_MUL4_LOAD=24, S_MUL4_WAIT=25, S_MUL4_CAP=26,
               S_COMBINE=27,
               S_WRITE_A=28, S_WRITE_B=29,
               S_NEXT_K=30, S_NEXT_J=31,
               S_NEXT_STAGE=32,
               S_BR_LOOP=33, S_BR_WAIT_A=34, S_BR_CAP_A_REQ_B=35,
               S_BR_WAIT_B=36, S_BR_CAP_B_WR_A=37, S_BR_WR_B=38,
               S_DONE=39;

    reg [5:0] state;
    reg [15:0] N;
    reg [3:0] log2N, stage;
    reg mode_r;                              // mode latched at start
    reg [ADDR_W:0] stride, j, k, max_k;
    reg [ADDR_W-1:0] addr_a, addr_b;

    // Datapath registers
    reg signed [DATA_W-1:0] Ar, Ai, Br, Bi;
    reg signed [TWID_W-1:0] Wr, Wi;
    reg signed [SW-1:0] sum_r, sum_i, diff_r, diff_i;
    reg signed [SW-1:0] mul_a;
    reg signed [TWID_W-1:0] mul_b;
    (* use_dsp = "yes" *) wire signed [PW-1:0] mul_result;
    reg signed [PW-1:0] mul_reg;
    reg signed [PW-1:0] prod1, prod2, prod3, prod4;
    reg signed [CW-1:0] prod_re, prod_im;

    // Bit-reversal registers
    reg [ADDR_W:0] br_i;
    reg [2*DATA_W-1:0] reg_br_a, reg_br_b;

    // ---------------------------------------------------------
    // COMBINATIONAL LOGIC
    // ---------------------------------------------------------
    assign mul_result = mul_a * mul_b;

    wire [ADDR_W-1:0] base_addr = (k << (log2N - stage)) + j;

    // Twiddle address: original contract or stride into a W_MAX_N table
    wire [ADDR_W:0]   tw_base = j << stage;
    wire [ADDR_W-1:0] tw_addr = (TWID_STRIDE != 0) ? (tw_base << (ADDR_W - log2N))
                                                   : tw_base[ADDR_W-1:0];

    // Per-stage scaling: forward always /2, inverse /2 only when IFFT_SCALE=1
    wire scale_en = (mode_r == 1'b0) || (IFFT_SCALE != 0);

    wire signed [SW-1:0] add_r = Ar + Br;    // operands sign-extended to SW
    wire signed [SW-1:0] add_i = Ai + Bi;
    wire signed [SW-1:0] sub_r = Ar - Br;
    wire signed [SW-1:0] sub_i = Ai - Bi;

    // round-half-up >>> 1 (RND1 = 0 gives floor); result fits in DATA_W bits
    wire signed [SW-1:0] add_r_h = (add_r + RND1) >>> 1;
    wire signed [SW-1:0] add_i_h = (add_i + RND1) >>> 1;
    wire signed [SW-1:0] sub_r_h = (sub_r + RND1) >>> 1;
    wire signed [SW-1:0] sub_i_h = (sub_i + RND1) >>> 1;

    // Saturation of a CW-bit signed value to DATA_W bits
    function [DATA_W-1:0] sat_w;
        input signed [CW-1:0] v;
        begin
            if (v > $signed({{(CW-DATA_W){1'b0}}, 1'b0, {(DATA_W-1){1'b1}}}))
                sat_w = {1'b0, {(DATA_W-1){1'b1}}};
            else if (v < $signed({{(CW-DATA_W){1'b1}}, 1'b1, {(DATA_W-1){1'b0}}}))
                sat_w = {1'b1, {(DATA_W-1){1'b0}}};
            else
                sat_w = v[DATA_W-1:0];
        end
    endfunction

    wire signed [DATA_W-1:0] wa_r = sat_w(sum_r);
    wire signed [DATA_W-1:0] wa_i = sat_w(sum_i);
    wire signed [DATA_W-1:0] wb_r = sat_w(prod_re);
    wire signed [DATA_W-1:0] wb_i = sat_w(prod_im);
    wire sat_a = (wa_r != sum_r) || (wa_i != sum_i);
    wire sat_b = (wb_r != prod_re) || (wb_i != prod_im);

    // Bit-reverse over ADDR_W bits, then right-align to the active log2N
    function [ADDR_W-1:0] bit_reverse_full(input [ADDR_W-1:0] val);
        integer i;
        begin
            for (i = 0; i < ADDR_W; i = i + 1) begin
                bit_reverse_full[ADDR_W - 1 - i] = val[i];
            end
        end
    endfunction

    wire [ADDR_W-1:0] br_rev_i = bit_reverse_full(br_i[ADDR_W-1:0]) >> (ADDR_W - log2N);

    // ---------------------------------------------------------
    // SEQUENTIAL FSM
    // ---------------------------------------------------------
    always @(posedge clk) if (!rstn) mul_reg <= 0; else mul_reg <= mul_result;

    always @(posedge clk) begin
        if (!rstn) begin
            state <= S_IDLE; busy <= 0; done <= 0; sat_event <= 0;
            ram_data_we <= 0; ram_twid_re <= 0;
            N <= 0; log2N <= 0; stage <= 0; mode_r <= 0;
            stride <= 0; j <= 0; k <= 0; max_k <= 0; br_i <= 0;
            addr_a <= 0; addr_b <= 0;
            Ar <= 0; Ai <= 0; Br <= 0; Bi <= 0; Wr <= 0; Wi <= 0;
            sum_r <= 0; sum_i <= 0; diff_r <= 0; diff_i <= 0;
            mul_a <= 0; mul_b <= 0;
            prod1 <= 0; prod2 <= 0; prod3 <= 0; prod4 <= 0;
            prod_re <= 0; prod_im <= 0;
            ram_data_addr <= 0; ram_data_wdata <= 0; ram_twid_addr <= 0;
            reg_br_a <= 0; reg_br_b <= 0;
        end else begin
            ram_data_we <= 0;
            ram_twid_re <= 0;
            sat_event   <= 0;
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin
                        N <= fft_n;
                        mode_r <= mode;
                        case (fft_n)
                            16'd256:  log2N <= 8;
                            16'd512:  log2N <= 9;
                            16'd1024: log2N <= 10;
                            16'd2048: log2N <= 11;
                            default:  log2N <= 8;
                        endcase
                        busy <= 1;
                        state <= S_INIT;
                    end
                end

                S_INIT: begin stage <= 0; stride <= N >> 1; state <= S_STAGE_LOOP; end

                S_STAGE_LOOP: if (stage >= log2N) begin br_i <= 0; state <= S_BR_LOOP; end
                              else begin j <= 0; max_k <= (1 << stage); state <= S_J_LOOP; end

                S_J_LOOP: if (j >= stride) state <= S_NEXT_STAGE; else begin k <= 0; state <= S_K_LOOP; end
                S_K_LOOP: if (k >= max_k) state <= S_NEXT_J; else begin
                    addr_a <= base_addr;
                    addr_b <= base_addr + stride;
                    state <= S_READ_A;
                end

                S_READ_A: begin ram_data_addr <= addr_a; state <= S_WAIT_RD_A; end
                S_WAIT_RD_A: state <= S_CAP_A;
                S_CAP_A: begin
                    Ar <= ram_data_rdata[2*DATA_W-1:DATA_W];
                    Ai <= ram_data_rdata[DATA_W-1:0];
                    state <= S_READ_TW;
                end

                S_READ_TW: begin ram_twid_addr <= tw_addr; ram_twid_re <= 1; state <= S_WAIT_TW; end
                S_WAIT_TW: state <= S_CAP_TW;
                S_CAP_TW: begin
                    Wr <= ram_twid_rdata[2*TWID_W-1:TWID_W];
                    if (mode_r) Wi <= -$signed(ram_twid_rdata[TWID_W-1:0]);
                    else        Wi <=  ram_twid_rdata[TWID_W-1:0];
                    ram_twid_re <= 0;
                    state <= S_READ_B;
                end

                S_READ_B: begin ram_data_addr <= addr_b; state <= S_WAIT_RD_B; end
                S_WAIT_RD_B: state <= S_CAP_B;
                S_CAP_B: begin
                    Br <= ram_data_rdata[2*DATA_W-1:DATA_W];
                    Bi <= ram_data_rdata[DATA_W-1:0];
                    state <= S_CALC;
                end

                S_CALC: begin
                    if (scale_en) begin
                        sum_r  <= add_r_h; sum_i  <= add_i_h;
                        diff_r <= sub_r_h; diff_i <= sub_i_h;
                    end else begin
                        sum_r  <= add_r;   sum_i  <= add_i;
                        diff_r <= sub_r;   diff_i <= sub_i;
                    end
                    state <= S_MUL1_LOAD;
                end

                S_MUL1_LOAD: begin mul_a <= diff_r; mul_b <= Wr; state <= S_MUL1_WAIT; end
                S_MUL1_WAIT: state <= S_MUL1_CAP;
                S_MUL1_CAP:  begin prod1 <= mul_reg; state <= S_MUL2_LOAD; end
                S_MUL2_LOAD: begin mul_a <= diff_i; mul_b <= Wi; state <= S_MUL2_WAIT; end
                S_MUL2_WAIT: state <= S_MUL2_CAP;
                S_MUL2_CAP:  begin prod2 <= mul_reg; state <= S_MUL3_LOAD; end
                S_MUL3_LOAD: begin mul_a <= diff_r; mul_b <= Wi; state <= S_MUL3_WAIT; end
                S_MUL3_WAIT: state <= S_MUL3_CAP;
                S_MUL3_CAP:  begin prod3 <= mul_reg; state <= S_MUL4_LOAD; end
                S_MUL4_LOAD: begin mul_a <= diff_i; mul_b <= Wr; state <= S_MUL4_WAIT; end
                S_MUL4_WAIT: state <= S_MUL4_CAP;
                S_MUL4_CAP:  begin prod4 <= mul_reg; state <= S_COMBINE; end

                S_COMBINE: begin
                    prod_re <= ($signed({prod1[PW-1], prod1}) - $signed({prod2[PW-1], prod2}) + RNDTW) >>> (TWID_W - 1);
                    prod_im <= ($signed({prod3[PW-1], prod3}) + $signed({prod4[PW-1], prod4}) + RNDTW) >>> (TWID_W - 1);
                    state <= S_WRITE_A;
                end

                S_WRITE_A: begin
                    ram_data_addr <= addr_a; ram_data_we <= 1;
                    ram_data_wdata <= {wa_r, wa_i};
                    sat_event <= sat_a;
                    state <= S_WRITE_B;
                end
                S_WRITE_B: begin
                    ram_data_addr <= addr_b; ram_data_we <= 1;
                    ram_data_wdata <= {wb_r, wb_i};
                    sat_event <= sat_b;
                    state <= S_NEXT_K;
                end

                S_NEXT_K: begin k <= k + 1; state <= S_K_LOOP; end
                S_NEXT_J: begin j <= j + 1; state <= S_J_LOOP; end
                S_NEXT_STAGE: begin stage <= stage + 1; stride <= stride >> 1; state <= S_STAGE_LOOP; end

                // =========================================================
                // IN-PLACE BIT-REVERSAL
                // =========================================================
                S_BR_LOOP: begin
                    if (br_i >= N) begin
                        state <= S_DONE;
                    end else if (br_i < br_rev_i) begin
                        // swap only when index < reversed index (avoids double swap)
                        ram_data_addr <= br_i[ADDR_W-1:0];
                        state <= S_BR_WAIT_A;
                    end else begin
                        br_i <= br_i + 1;
                    end
                end

                S_BR_WAIT_A: state <= S_BR_CAP_A_REQ_B;

                S_BR_CAP_A_REQ_B: begin
                    reg_br_a <= ram_data_rdata;
                    ram_data_addr <= br_rev_i;
                    state <= S_BR_WAIT_B;
                end

                S_BR_WAIT_B: state <= S_BR_CAP_B_WR_A;

                S_BR_CAP_B_WR_A: begin
                    reg_br_b <= ram_data_rdata;
                    ram_data_addr <= br_i[ADDR_W-1:0];
                    ram_data_we <= 1;
                    ram_data_wdata <= ram_data_rdata;
                    state <= S_BR_WR_B;
                end

                S_BR_WR_B: begin
                    ram_data_addr <= br_rev_i;
                    ram_data_we <= 1;
                    ram_data_wdata <= reg_br_a;
                    br_i <= br_i + 1;
                    state <= S_BR_LOOP;
                end

                S_DONE: begin done <= 1; busy <= 0; state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
