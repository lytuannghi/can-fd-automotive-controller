// =============================================================================
// File: can_fault_confinement.v
// Module: can_fault_confinement
// Project: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
// Description: Fault Confinement State Machine and Error Counters per ISO 11898-1:2015.
//              - Transmit Error Counter (TEC)
//              - Receive Error Counter (REC)
//              - Node States: Error Active, Error Passive, Bus-Off
//              - Automatic Bus-Off Recovery Monitoring
// =============================================================================

`timescale 1ns / 1ps

module can_fault_confinement (
    input  wire        clk,
    input  wire        rst_n,

    // Error Events
    input  wire        tx_success,   // Successful frame transmission
    input  wire        rx_success,   // Successful frame reception
    input  wire        tx_err_event, // Bit error, ACK error, Stuff error on TX (+8)
    input  wire        rx_err_event, // CRC error, Form error, Stuff error on RX (+1)

    // Bus-off Recovery Sequence Monitor
    input  wire        eleven_recessive_bits, // Strobe on 11 consecutive recessive bits

    // Status Outputs
    output reg  [7:0]  tec,          // Transmit Error Counter (0 to 255)
    output reg  [7:0]  rec,          // Receive Error Counter (0 to 255)
    output wire        err_warning,  // TEC >= 96 or REC >= 96
    output wire        err_passive,  // TEC >= 128 or REC >= 128
    output wire        bus_off       // TEC > 255
);

    reg [7:0] bus_off_recovery_cnt;
    reg       bus_off_state;

    assign err_warning = (tec >= 8'd96) || (rec >= 8'd96);
    assign err_passive = (tec >= 8'd128) || (rec >= 8'd128);
    assign bus_off     = bus_off_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tec                  <= 8'd0;
            rec                  <= 8'd0;
            bus_off_state        <= 1'b0;
            bus_off_recovery_cnt <= 8'd0;
        end else begin
            if (bus_off_state) begin
                // Monitor 128 occurrences of 11 consecutive recessive bits
                if (eleven_recessive_bits) begin
                    if (bus_off_recovery_cnt >= 8'd127) begin
                        bus_off_state        <= 1'b0;
                        bus_off_recovery_cnt <= 8'd0;
                        tec                  <= 8'd0;
                        rec                  <= 8'd0;
                    end else begin
                        bus_off_recovery_cnt <= bus_off_recovery_cnt + 8'd1;
                    end
                end
            end else begin
                // TEC Update
                if (tx_err_event) begin
                    if (tec >= 8'd248) begin
                        tec           <= 8'hFF;
                        bus_off_state <= 1'b1; // Transition to Bus-Off
                    end else begin
                        tec <= tec + 8'd8;
                    end
                end else if (tx_success && tec > 8'd0) begin
                    tec <= tec - 8'd1;
                end

                // REC Update
                if (rx_err_event) begin
                    if (rec < 8'hFE)
                        rec <= rec + 8'd1;
                end else if (rx_success && rec > 8'd0) begin
                    rec <= rec - 8'd1;
                end
            end
        end
    end

endmodule
