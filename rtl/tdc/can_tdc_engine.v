// =============================================================================
// File: can_tdc_engine.v
// Module: can_tdc_engine
// Project: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
// Description: Transmitter Delay Compensation (TDC) Engine.
//              Measures physical transceiver loop delay (TX pin to RX pin)
//              and generates Secondary Sample Point (SSP) for bit verification
//              during the high-speed data phase (> 2 Mbps).
// =============================================================================

`timescale 1ns / 1ps

module can_tdc_engine (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        tdc_enable,    // Enable TDC engine
    input  wire [5:0]  ssp_offset,    // Configurable SSP offset in clock cycles
    input  wire        can_tx,        // Transmitted bit
    input  wire        can_rx,        // Received loopback bit
    input  wire        meas_trigger,  // Strobe on FDF/BRS edge to start measurement

    output reg  [7:0]  measured_delay,// Measured loop delay in clock cycles
    output reg         ssp_pulse,     // Strobe at Secondary Sample Point
    output reg         bit_err_tdc    // Bit error detected at SSP
);

    reg        measuring;
    reg [7:0]  delay_cnt;
    reg [7:0]  ssp_countdown;
    reg        ssp_active;
    reg        expected_tx_bit;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            measuring       <= 1'b0;
            delay_cnt       <= 8'd0;
            measured_delay  <= 8'd0;
            ssp_countdown   <= 8'd0;
            ssp_active      <= 1'b0;
            ssp_pulse       <= 1'b0;
            bit_err_tdc     <= 1'b0;
            expected_tx_bit <= 1'b1;
        end else begin
            ssp_pulse   <= 1'b0;
            bit_err_tdc <= 1'b0;

            if (tdc_enable) begin
                // Measurement start on TX transition
                if (meas_trigger && !measuring) begin
                    measuring       <= 1'b1;
                    delay_cnt       <= 8'd0;
                    expected_tx_bit <= can_tx;
                end else if (measuring) begin
                    delay_cnt <= delay_cnt + 8'd1;
                    // Stop measurement when RX matches TX transition
                    if (can_rx == expected_tx_bit || delay_cnt == 8'hFF) begin
                        measuring      <= 1'b0;
                        measured_delay <= delay_cnt;
                        // Arm SSP countdown: delay + configured offset
                        ssp_countdown  <= delay_cnt + {2'b00, ssp_offset};
                        ssp_active     <= 1'b1;
                    end
                end

                // Secondary Sample Point Generation
                if (ssp_active) begin
                    if (ssp_countdown == 8'd1) begin
                        ssp_pulse  <= 1'b1;
                        ssp_active <= 1'b0;
                        // Compare transmitted bit vs loopback received bit
                        if (can_rx != expected_tx_bit) begin
                            bit_err_tdc <= 1'b1;
                        end
                    end else begin
                        ssp_countdown <= ssp_countdown - 8'd1;
                    end
                end
            end
        end
    end

endmodule
