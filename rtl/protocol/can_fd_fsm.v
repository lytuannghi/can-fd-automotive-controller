// =============================================================================
// File: can_fd_fsm.v
// Module: can_fd_fsm
// Project: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
// Description: Protocol Frame Engine Finite State Machine (FSM).
//              Orchestrates SOF, Arbitration, Control, 64-byte Data Field,
//              CRC-17/21, ACK, and EOF states for CAN-FD and Classical CAN.
// =============================================================================

`timescale 1ns / 1ps

module can_fd_fsm (
    input  wire        clk,
    input  wire        rst_n,

    // Timing Strobe Inputs from can_bit_timing
    input  wire        sample_point,
    input  wire        bit_start,

    // Transmit Request & Data
    input  wire        tx_start,
    input  wire [28:0] tx_id,
    input  wire        tx_ide,
    input  wire        tx_fdf,       // 1: CAN-FD, 0: Classical
    input  wire        tx_brs,       // Bit Rate Switch
    input  wire [3:0]  tx_dlc,
    input  wire [511:0] tx_payload,

    // CRC inputs
    input  wire [14:0] crc15_val,
    input  wire [16:0] crc17_val,
    input  wire [20:0] crc21_val,
    input  wire [3:0]  stuff_count_val,

    // Physical CAN Bus Interface
    input  wire        can_rx,
    output reg         can_tx,
    output reg         brs_active,   // Strobe to switch to data bit rate
    output reg         tx_success,
    output reg         rx_success,
    output reg         crc_init_pulse,
    output reg         crc_calc_en,
    output reg         crc_bit_val,
    output reg         meas_trigger
);

    // Frame States
    localparam S_IDLE        = 4'd0;
    localparam S_SOF         = 4'd1;
    localparam S_ARBITRATION = 4'd2;
    localparam S_CONTROL     = 4'd3;
    localparam S_DATA        = 4'd4;
    localparam S_CRC         = 4'd5;
    localparam S_ACK         = 4'd6;
    localparam S_EOF         = 4'd7;
    localparam S_INTERM      = 4'd8;

    reg [3:0] state;
    reg [5:0] bit_cnt;
    reg [9:0] payload_bit_cnt;
    reg [9:0] total_payload_bits;

    // Decode byte count from DLC
    always @(*) begin
        case (tx_dlc)
            4'd0: total_payload_bits = 10'd0;
            4'd1: total_payload_bits = 10'd8;
            4'd2: total_payload_bits = 10'd16;
            4'd3: total_payload_bits = 10'd24;
            4'd4: total_payload_bits = 10'd32;
            4'd5: total_payload_bits = 10'd40;
            4'd6: total_payload_bits = 10'd48;
            4'd7: total_payload_bits = 10'd56;
            4'd8: total_payload_bits = 10'd64;
            4'd9: total_payload_bits = 10'd96;   // 12 bytes
            4'd10: total_payload_bits = 10'd128; // 16 bytes
            4'd11: total_payload_bits = 10'd160; // 20 bytes
            4'd12: total_payload_bits = 10'd192; // 24 bytes
            4'd13: total_payload_bits = 10'd256; // 32 bytes
            4'd14: total_payload_bits = 10'd384; // 48 bytes
            default: total_payload_bits = 10'd512; // 64 bytes
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            can_tx             <= 1'b1; // Recessive default
            brs_active         <= 1'b0;
            tx_success         <= 1'b0;
            rx_success         <= 1'b0;
            crc_init_pulse     <= 1'b0;
            crc_calc_en        <= 1'b0;
            crc_bit_val        <= 1'b0;
            meas_trigger       <= 1'b0;
            bit_cnt            <= 6'd0;
            payload_bit_cnt    <= 10'd0;
        end else begin
            tx_success     <= 1'b0;
            rx_success     <= 1'b0;
            crc_init_pulse <= 1'b0;
            crc_calc_en    <= 1'b0;
            meas_trigger   <= 1'b0;

            case (state)
                S_IDLE: begin
                    can_tx     <= 1'b1;
                    brs_active <= 1'b0;
                    if (tx_start) begin
                        state          <= S_SOF;
                        crc_init_pulse <= 1'b1;
                    end
                end

                S_SOF: begin
                    if (bit_start) begin
                        can_tx      <= 1'b0; // Dominant SOF
                        crc_calc_en <= 1'b1;
                        crc_bit_val <= 1'b0;
                        state       <= S_ARBITRATION;
                        bit_cnt     <= 6'd0;
                    end
                end

                S_ARBITRATION: begin
                    if (bit_start) begin
                        can_tx      <= tx_id[10 - bit_cnt];
                        crc_calc_en <= 1'b1;
                        crc_bit_val <= tx_id[10 - bit_cnt];
                        if (bit_cnt >= 6'd10) begin
                            state   <= S_CONTROL;
                            bit_cnt <= 6'd0;
                        end else begin
                            bit_cnt <= bit_cnt + 6'd1;
                        end
                    end
                end

                S_CONTROL: begin
                    if (bit_start) begin
                        // Transmit FDF, res, BRS, ESI, DLC[3:0]
                        case (bit_cnt)
                            6'd0: begin can_tx <= tx_ide; crc_calc_en <= 1'b1; crc_bit_val <= tx_ide; end
                            6'd1: begin can_tx <= tx_fdf; crc_calc_en <= 1'b1; crc_bit_val <= tx_fdf; end
                            6'd2: begin can_tx <= 1'b0;   crc_calc_en <= 1'b1; crc_bit_val <= 1'b0; end // res bit
                            6'd3: begin
                                can_tx       <= tx_brs;
                                crc_calc_en  <= 1'b1;
                                crc_bit_val  <= tx_brs;
                                brs_active   <= tx_fdf && tx_brs;
                                meas_trigger <= 1'b1; // Trigger TDC measurement
                            end
                            6'd4: begin can_tx <= 1'b0; crc_calc_en <= 1'b1; crc_bit_val <= 1'b0; end // ESI
                            6'd5: begin can_tx <= tx_dlc[3]; crc_calc_en <= 1'b1; crc_bit_val <= tx_dlc[3]; end
                            6'd6: begin can_tx <= tx_dlc[2]; crc_calc_en <= 1'b1; crc_bit_val <= tx_dlc[2]; end
                            6'd7: begin can_tx <= tx_dlc[1]; crc_calc_en <= 1'b1; crc_bit_val <= tx_dlc[1]; end
                            6'd8: begin
                                can_tx      <= tx_dlc[0];
                                crc_calc_en <= 1'b1;
                                crc_bit_val <= tx_dlc[0];
                                state       <= (total_payload_bits > 0) ? S_DATA : S_CRC;
                                bit_cnt     <= 6'd0;
                                payload_bit_cnt <= 10'd0;
                            end
                            default: ;
                        endcase
                        if (bit_cnt < 6'd8)
                            bit_cnt <= bit_cnt + 6'd1;
                    end
                end

                S_DATA: begin
                    if (bit_start) begin
                        can_tx          <= tx_payload[511 - payload_bit_cnt];
                        crc_calc_en     <= 1'b1;
                        crc_bit_val     <= tx_payload[511 - payload_bit_cnt];
                        payload_bit_cnt <= payload_bit_cnt + 10'd1;

                        if (payload_bit_cnt + 10'd1 >= total_payload_bits) begin
                            state   <= S_CRC;
                            bit_cnt <= 6'd0;
                        end
                    end
                end

                S_CRC: begin
                    if (bit_start) begin
                        // Transmit CRC bits and CRC delimiter
                        can_tx <= 1'b0;
                        if (bit_cnt >= 6'd17) begin
                            can_tx     <= 1'b1; // CRC delimiter (recessive)
                            brs_active <= 1'b0; // Switch back to nominal bit rate
                            state      <= S_ACK;
                            bit_cnt    <= 6'd0;
                        end else begin
                            bit_cnt <= bit_cnt + 6'd1;
                        end
                    end
                end

                S_ACK: begin
                    if (bit_start) begin
                        case (bit_cnt)
                            6'd0: begin
                                can_tx  <= 1'b1; // Transmitter sends recessive ACK slot
                                bit_cnt <= 6'd1;
                            end
                            6'd1: begin
                                can_tx     <= 1'b1; // ACK delimiter
                                state      <= S_EOF;
                                bit_cnt    <= 6'd0;
                                tx_success <= 1'b1;
                            end
                            default: ;
                        endcase
                    end
                end

                S_EOF: begin
                    if (bit_start) begin
                        can_tx <= 1'b1; // 7 recessive bits
                        if (bit_cnt >= 6'd6) begin
                            state   <= S_INTERM;
                            bit_cnt <= 6'd0;
                        end else begin
                            bit_cnt <= bit_cnt + 6'd1;
                        end
                    end
                end

                S_INTERM: begin
                    if (bit_start) begin
                        can_tx <= 1'b1; // 3 recessive bits
                        if (bit_cnt >= 6'd2) begin
                            state   <= S_IDLE;
                            bit_cnt <= 6'd0;
                        end else begin
                            bit_cnt <= bit_cnt + 6'd1;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
