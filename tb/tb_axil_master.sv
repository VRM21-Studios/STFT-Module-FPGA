`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Testbench   : tb_axil_master
// Description : AXI4-Lite master BFM.
//               write: AW and W driven together and monitored in parallel
//                        with fork...join; waits for BVALID.
//               read : clock-edge delay before sampling ARREADY / RVALID
//                        (do @(posedge clk); while (!x);).
//               Handshakes are evaluated on the values sampled at the clock
//               edge, i.e. the same values the DUT registers see.
// ============================================================================

module tb_axil_master #(
    parameter int AW = 6
) (
    input  logic          clk,
    output logic [AW-1:0] awaddr,
    output logic          awvalid,
    input  logic          awready,
    output logic [31:0]   wdata,
    output logic [3:0]    wstrb,
    output logic          wvalid,
    input  logic          wready,
    input  logic [1:0]    bresp,
    input  logic          bvalid,
    output logic          bready,
    output logic [AW-1:0] araddr,
    output logic          arvalid,
    input  logic          arready,
    input  logic [31:0]   rdata,
    input  logic [1:0]    rresp,
    input  logic          rvalid,
    output logic          rready
);

    int resp_errors = 0;

    initial begin
        awaddr = '0; awvalid = 1'b0; wdata = '0; wstrb = '0; wvalid = 1'b0; bready = 1'b0;
        araddr = '0; arvalid = 1'b0; rready = 1'b0;
    end

    task automatic write(input logic [AW-1:0] addr, input logic [31:0] data);
        @(posedge clk);
        awaddr  <= addr;  awvalid <= 1'b1;
        wdata   <= data;  wstrb   <= 4'hF; wvalid <= 1'b1;
        bready  <= 1'b1;
        fork
            begin do @(posedge clk); while (!awready); awvalid <= 1'b0; end
            begin do @(posedge clk); while (!wready);  wvalid  <= 1'b0; end
        join
        do @(posedge clk); while (!bvalid);
        if (bresp != 2'b00) resp_errors++;
        bready <= 1'b0;
    endtask

    task automatic read(input logic [AW-1:0] addr, output logic [31:0] data);
        @(posedge clk);
        araddr <= addr; arvalid <= 1'b1; rready <= 1'b1;
        do @(posedge clk); while (!arready);
        arvalid <= 1'b0;
        do @(posedge clk); while (!rvalid);
        data = rdata;
        if (rresp != 2'b00) resp_errors++;
        rready <= 1'b0;
    endtask

endmodule
