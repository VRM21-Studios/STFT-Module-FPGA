`timescale 1ns / 1ps

/* ============================================================================
 * VRM21 Studios - VRM21-FFT-Module-FPGA
 * MODULE      : vrm_ram_core (Multi-Style)
 * DESCRIPTION : Simple dual-port RAM (1 write port, 1 synchronous read port,
 *               single clock) with selectable implementation style.
 *               RAM_STYLE : "ultra", "block", "distributed", "auto"
 *               INIT_FILE : optional $readmemh file ("" = zero-initialised).
 *                           Used for ROM-style tables (twiddle, window).
 * READ        : rd_data is registered and updates only when re = 1
 *               (1-cycle latency, holds its value otherwise).
 * ============================================================================ */

module vrm_ram_core #(
    parameter DATA_WIDTH = 32,
    parameter ADDR_WIDTH = 12,
    parameter RAM_STYLE  = "auto",  // "ultra", "block", "distributed", "auto"
    parameter INIT_FILE  = ""
)(
    input  wire                  clk,
    input  wire                  rstn,

    input  wire                  we,
    input  wire [ADDR_WIDTH-1:0] wr_addr,
    input  wire [DATA_WIDTH-1:0] wr_data,

    input  wire                  re,
    input  wire [ADDR_WIDTH-1:0] rd_addr,
    output reg  [DATA_WIDTH-1:0] rd_data
);

    generate
        if (RAM_STYLE == "ultra") begin : gen_uram
            (* ram_style = "ultra" *)
            reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

            integer i;
            initial begin
                for (i = 0; i < (1<<ADDR_WIDTH); i = i + 1) ram[i] = {DATA_WIDTH{1'b0}};
                if (INIT_FILE != "") $readmemh(INIT_FILE, ram);
            end

            always @(posedge clk) begin
                if (!rstn) begin
                    rd_data <= {DATA_WIDTH{1'b0}};
                end else begin
                    if (we) ram[wr_addr] <= wr_data;
                    if (re) rd_data <= ram[rd_addr];
                end
            end

        end else if (RAM_STYLE == "block") begin : gen_bram
            (* ram_style = "block" *)
            reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

            integer i;
            initial begin
                for (i = 0; i < (1<<ADDR_WIDTH); i = i + 1) ram[i] = {DATA_WIDTH{1'b0}};
                if (INIT_FILE != "") $readmemh(INIT_FILE, ram);
            end

            always @(posedge clk) begin
                if (!rstn) begin
                    rd_data <= {DATA_WIDTH{1'b0}};
                end else begin
                    if (we) ram[wr_addr] <= wr_data;
                    if (re) rd_data <= ram[rd_addr];
                end
            end

        end else if (RAM_STYLE == "distributed") begin : gen_dist
            (* ram_style = "distributed" *)
            reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

            integer i;
            initial begin
                for (i = 0; i < (1<<ADDR_WIDTH); i = i + 1) ram[i] = {DATA_WIDTH{1'b0}};
                if (INIT_FILE != "") $readmemh(INIT_FILE, ram);
            end

            always @(posedge clk) begin
                if (!rstn) begin
                    rd_data <= {DATA_WIDTH{1'b0}};
                end else begin
                    if (we) ram[wr_addr] <= wr_data;
                    if (re) rd_data <= ram[rd_addr];   // synchronous read, 1-cycle latency
                end
            end

        end else begin : gen_auto
            (* ram_style = "auto" *)
            reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

            integer i;
            initial begin
                for (i = 0; i < (1<<ADDR_WIDTH); i = i + 1) ram[i] = {DATA_WIDTH{1'b0}};
                if (INIT_FILE != "") $readmemh(INIT_FILE, ram);
            end

            always @(posedge clk) begin
                if (!rstn) begin
                    rd_data <= {DATA_WIDTH{1'b0}};
                end else begin
                    if (we) ram[wr_addr] <= wr_data;
                    if (re) rd_data <= ram[rd_addr];
                end
            end
        end
    endgenerate

endmodule
