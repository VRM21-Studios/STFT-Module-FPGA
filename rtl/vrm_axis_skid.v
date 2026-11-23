`timescale 1ns / 1ps

// ============================================================================
// VRM21 Studios - STFT-Module-FPGA
// Module      : vrm_axis_skid
// Description : Two-entry AXI4-Stream register slice (skid buffer).
//               - s_axis_tready is a register output (no combinational path
//                 from m_axis_tready to s_axis_tready).
//               - m_axis_tdata / m_axis_tlast are forced to zero while
//                 m_axis_tvalid = 0.
//               - Full throughput (one beat per clock) when downstream is ready.
// ============================================================================

module vrm_axis_skid #(
    parameter DATA_W = 32
) (
    input  wire              clk,
    input  wire              rstn,

    input  wire [DATA_W-1:0] s_axis_tdata,
    input  wire              s_axis_tlast,
    input  wire              s_axis_tvalid,
    output wire              s_axis_tready,

    output wire [DATA_W-1:0] m_axis_tdata,
    output wire              m_axis_tlast,
    output wire              m_axis_tvalid,
    input  wire              m_axis_tready
);

    reg [DATA_W:0] main_d, skid_d;           // {tlast, tdata}
    reg            main_v, skid_v;

    wire s_fire = s_axis_tvalid && !skid_v;
    wire m_free = !main_v || m_axis_tready;  // main register can take a new beat

    assign s_axis_tready = !skid_v;
    assign m_axis_tvalid = main_v;
    assign m_axis_tdata  = main_v ? main_d[DATA_W-1:0] : {DATA_W{1'b0}};
    assign m_axis_tlast  = main_v ? main_d[DATA_W]     : 1'b0;

    always @(posedge clk) begin
        if (!rstn) begin
            main_v <= 1'b0;
            skid_v <= 1'b0;
            main_d <= {(DATA_W+1){1'b0}};
            skid_d <= {(DATA_W+1){1'b0}};
        end else begin
            if (m_free) begin
                if (skid_v) begin
                    main_d <= skid_d;
                    main_v <= 1'b1;
                    skid_v <= 1'b0;
                end else if (s_fire) begin
                    main_d <= {s_axis_tlast, s_axis_tdata};
                    main_v <= 1'b1;
                end else begin
                    main_v <= 1'b0;
                end
            end else if (s_fire) begin
                skid_d <= {s_axis_tlast, s_axis_tdata};
                skid_v <= 1'b1;
            end
        end
    end

endmodule
