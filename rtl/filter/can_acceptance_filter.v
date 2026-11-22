// =============================================================================
// File: can_acceptance_filter.v
// Module: can_acceptance_filter
// Project: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
// Description: Multi-Filter Acceptance Filtering Unit.
//              Supports 4 independent filter registers with individual masks
//              for Standard 11-bit IDs and Extended 29-bit IDs.
// =============================================================================

`timescale 1ns / 1ps

module can_acceptance_filter (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        check_enable, // Strobe to perform filter check on received ID
    input  wire        rx_ide,       // 0: Standard 11-bit, 1: Extended 29-bit
    input  wire [28:0] rx_id,        // Received Identifier

    // 4 Filter Banks (Filter ID & Filter Mask)
    input  wire [28:0] f0_id,   input wire [28:0] f0_mask,  input wire f0_en,
    input  wire [28:0] f1_id,   input wire [28:0] f1_mask,  input wire f1_en,
    input  wire [28:0] f2_id,   input wire [28:0] f2_mask,  input wire f2_en,
    input  wire [28:0] f3_id,   input wire [28:0] f3_mask,  input wire f3_en,

    output reg         match_valid,  // Filter match strobe
    output reg  [1:0]  matched_filter// Matched filter index (0 to 3)
);

    wire match_f0 = f0_en && ((rx_id & f0_mask) == (f0_id & f0_mask));
    wire match_f1 = f1_en && ((rx_id & f1_mask) == (f1_id & f1_mask));
    wire match_f2 = f2_en && ((rx_id & f2_mask) == (f2_id & f2_mask));
    wire match_f3 = f3_en && ((rx_id & f3_mask) == (f3_id & f3_mask));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            match_valid    <= 1'b0;
            matched_filter <= 2'd0;
        end else begin
            if (check_enable) begin
                if (match_f0) begin
                    match_valid    <= 1'b1;
                    matched_filter <= 2'd0;
                end else if (match_f1) begin
                    match_valid    <= 1'b1;
                    matched_filter <= 2'd1;
                end else if (match_f2) begin
                    match_valid    <= 1'b1;
                    matched_filter <= 2'd2;
                end else if (match_f3) begin
                    match_valid    <= 1'b1;
                    matched_filter <= 2'd3;
                end else begin
                    match_valid    <= 1'b0;
                end
            end
        end
    end

endmodule
