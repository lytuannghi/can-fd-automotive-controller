// =============================================================================
// File: can_fifo_buffer.v
// Module: can_fifo_buffer
// Project: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
// Description: 4-Deep Message FIFO Buffer for Transmit & Receive Subsystems.
//              Supports payloads up to 64 bytes per CAN-FD message.
// =============================================================================

`timescale 1ns / 1ps

module can_fifo_buffer (
    input  wire        clk,
    input  wire        rst_n,

    // Push Interface (Write)
    input  wire        push,
    input  wire [28:0] wr_id,
    input  wire        wr_ide,
    input  wire        wr_fdf,       // 1: CAN-FD, 0: Classical
    input  wire        wr_brs,       // Bit Rate Switch
    input  wire [3:0]  wr_dlc,
    input  wire [511:0] wr_payload,  // 64 Bytes Payload (512 bits)
    output wire        fifo_full,

    // Pop Interface (Read)
    input  wire        pop,
    output wire [28:0] rd_id,
    output wire        rd_ide,
    output wire        rd_fdf,
    output wire        rd_brs,
    output wire [3:0]  rd_dlc,
    output wire [511:0] rd_payload,
    output wire        fifo_empty,
    output wire [2:0]  fifo_count
);

    // 4 Storage Slots
    reg [28:0]  mem_id      [0:3];
    reg         mem_ide     [0:3];
    reg         mem_fdf     [0:3];
    reg         mem_brs     [0:3];
    reg [3:0]   mem_dlc     [0:3];
    reg [511:0] mem_payload [0:3];

    reg [1:0] wr_ptr;
    reg [1:0] rd_ptr;
    reg [2:0] count;

    assign fifo_empty = (count == 3'd0);
    assign fifo_full  = (count == 3'd4);
    assign fifo_count = count;

    assign rd_id      = mem_id[rd_ptr];
    assign rd_ide     = mem_ide[rd_ptr];
    assign rd_fdf     = mem_fdf[rd_ptr];
    assign rd_brs     = mem_brs[rd_ptr];
    assign rd_dlc     = mem_dlc[rd_ptr];
    assign rd_payload = mem_payload[rd_ptr];

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= 2'd0;
            rd_ptr <= 2'd0;
            count  <= 3'd0;
            for (i = 0; i < 4; i = i + 1) begin
                mem_id[i]      <= 29'd0;
                mem_ide[i]     <= 1'b0;
                mem_fdf[i]     <= 1'b0;
                mem_brs[i]     <= 1'b0;
                mem_dlc[i]     <= 4'd0;
                mem_payload[i] <= 512'd0;
            end
        end else begin
            case ({push && !fifo_full, pop && !fifo_empty})
                2'b10: begin // Push only
                    mem_id[wr_ptr]      <= wr_id;
                    mem_ide[wr_ptr]     <= wr_ide;
                    mem_fdf[wr_ptr]     <= wr_fdf;
                    mem_brs[wr_ptr]     <= wr_brs;
                    mem_dlc[wr_ptr]     <= wr_dlc;
                    mem_payload[wr_ptr] <= wr_payload;
                    wr_ptr              <= wr_ptr + 2'd1;
                    count               <= count + 3'd1;
                end
                2'b01: begin // Pop only
                    rd_ptr <= rd_ptr + 2'd1;
                    count  <= count - 3'd1;
                end
                2'b11: begin // Simultaneous Push and Pop
                    mem_id[wr_ptr]      <= wr_id;
                    mem_ide[wr_ptr]     <= wr_ide;
                    mem_fdf[wr_ptr]     <= wr_fdf;
                    mem_brs[wr_ptr]     <= wr_brs;
                    mem_dlc[wr_ptr]     <= wr_dlc;
                    mem_payload[wr_ptr] <= wr_payload;
                    wr_ptr              <= wr_ptr + 2'd1;
                    rd_ptr              <= rd_ptr + 2'd1;
                end
                default: ;
            endcase
        end
    end

endmodule
