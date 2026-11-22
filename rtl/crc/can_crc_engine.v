// =============================================================================
// File: can_crc_engine.v
// Module: can_crc_engine
// Project: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
// Description: Multi-Polynomial CRC Engine with Stuff Count Generator.
//              - Classical CAN: CRC-15 (Poly 0x4599)
//              - CAN-FD (DLC <= 16 bytes): CRC-17 (Poly 0x1685B, init 0x10000)
//              - CAN-FD (DLC > 16 bytes):  CRC-21 (Poly 0x102899, init 0x100000)
//              - Stuff Count: 3-bit Gray code + parity bit
// =============================================================================

`timescale 1ns / 1ps

module can_crc_engine (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        init,        // Reset CRC registers before frame
    input  wire        calc_enable, // Enable bit-by-bit CRC calculation
    input  wire        data_in,     // Incoming bit to be included in CRC
    input  wire        is_can_fd,   // 1: CAN-FD, 0: Classical CAN
    input  wire        payload_gt16,// 1: Payload > 16 bytes (use CRC-21), 0: <= 16 bytes (use CRC-17)

    input  wire        stuff_bit_in,// Strobe when dynamic bit stuffing occurs
    output reg  [3:0]  stuff_count, // [3:1] Gray code count, [0] parity

    output reg  [14:0] crc15_out,
    output reg  [16:0] crc17_out,
    output reg  [20:0] crc21_out
);

    // Polynomial definitions
    localparam [14:0] POLY_CRC15 = 15'h4599;
    localparam [16:0] POLY_CRC17 = 17'h1685B;
    localparam [20:0] POLY_CRC21 = 21'h102899;

    // Feedback calculations
    wire fb_crc15 = data_in ^ crc15_out[14];
    wire fb_crc17 = data_in ^ crc17_out[16];
    wire fb_crc21 = data_in ^ crc21_out[20];

    // Stuff counter tracking
    reg [2:0] raw_stuff_cnt;

    // Gray code mapping for stuff count: 0->0, 1->1, 2->3, 3->2, 4->6, 5->7, 6->5, 7->4
    reg [2:0] gray_code;
    always @(*) begin
        case (raw_stuff_cnt)
            3'd0: gray_code = 3'b000;
            3'd1: gray_code = 3'b001;
            3'd2: gray_code = 3'b011;
            3'd3: gray_code = 3'b010;
            3'd4: gray_code = 3'b110;
            3'd5: gray_code = 3'b111;
            3'd6: gray_code = 3'b101;
            3'd7: gray_code = 3'b100;
        endcase
    end

    wire parity_bit = ^gray_code; // Even parity over Gray code

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc15_out     <= 15'h0000;
            crc17_out     <= 17'h10000;
            crc21_out     <= 21'h100000;
            raw_stuff_cnt <= 3'd0;
            stuff_count   <= 4'd0;
        end else begin
            if (init) begin
                crc15_out     <= 15'h0000;
                crc17_out     <= 17'h10000;   // ISO CAN-FD init: bit 16 = 1
                crc21_out     <= 21'h100000;  // ISO CAN-FD init: bit 20 = 1
                raw_stuff_cnt <= 3'd0;
                stuff_count   <= 4'd0;
            end else begin
                // Update stuff count on stuffing bit
                if (stuff_bit_in) begin
                    raw_stuff_cnt <= raw_stuff_cnt + 3'd1;
                end
                stuff_count <= {gray_code, parity_bit};

                // Bit-by-bit CRC calculation
                if (calc_enable) begin
                    // CRC-15
                    if (fb_crc15)
                        crc15_out <= {crc15_out[13:0], 1'b0} ^ POLY_CRC15;
                    else
                        crc15_out <= {crc15_out[13:0], 1'b0};

                    // CRC-17
                    if (fb_crc17)
                        crc17_out <= {crc17_out[15:0], 1'b0} ^ POLY_CRC17;
                    else
                        crc17_out <= {crc17_out[15:0], 1'b0};

                    // CRC-21
                    if (fb_crc21)
                        crc21_out <= {crc21_out[19:0], 1'b0} ^ POLY_CRC21;
                    else
                        crc21_out <= {crc21_out[19:0], 1'b0};
                end
            end
        end
    end

endmodule
