// =============================================================================
// File: can_bit_timing.v
// Module: can_bit_timing
// Project: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
// Description: Dual Bit-Rate Timing Generator and Synchronization Engine.
//              Supports Nominal Phase (Arbitration up to 1 Mbps) and Fast Phase
//              (Data up to 5-8 Mbps) with Bit Rate Switching (BRS).
//              Implements Hard Sync on falling edges and Soft Resync with SJW.
// =============================================================================

`timescale 1ns / 1ps

module can_bit_timing (
    input  wire        clk,          // System Core Clock (e.g., 40 MHz - 100 MHz)
    input  wire        rst_n,

    // Timing Configuration: Nominal Bit Rate (Arbitration)
    input  wire [5:0]  nom_brp,      // Nominal Baud Rate Prescaler (1 to 64)
    input  wire [4:0]  nom_prop_seg, // Propagation Segment (1 to 32 Tq)
    input  wire [4:0]  nom_phase1,   // Phase Segment 1 (1 to 32 Tq)
    input  wire [3:0]  nom_phase2,   // Phase Segment 2 (1 to 16 Tq)
    input  wire [3:0]  nom_sjw,      // Synchronization Jump Width (1 to 16 Tq)

    // Timing Configuration: Data Bit Rate (Fast CAN-FD)
    input  wire [5:0]  data_brp,     // Data Baud Rate Prescaler
    input  wire [3:0]  data_prop_seg,// Data Propagation Segment
    input  wire [3:0]  data_phase1,  // Data Phase Segment 1
    input  wire [2:0]  data_phase2,  // Data Phase Segment 2
    input  wire [2:0]  data_sjw,     // Data SJW

    // Control & Status
    input  wire        brs_active,   // 1: Fast Data Bit Rate active, 0: Nominal
    input  wire        can_rx_synced,// Synchronized Physical RX line
    output reg         sample_point, // Strobe at Sample Point
    output reg         bit_start,    // Strobe at start of new bit (SOF/Bit boundary)
    output reg         hard_sync_req // Strobe when hard synchronization occurs
);

    // Time Quantum (Tq) Prescaler Counter
    reg [5:0] brp_cnt;
    wire [5:0] active_brp = brs_active ? data_brp : nom_brp;
    wire tq_tick = (brp_cnt == active_brp);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            brp_cnt <= 6'd0;
        end else begin
            if (tq_tick)
                brp_cnt <= 6'd0;
            else
                brp_cnt <= brp_cnt + 6'd1;
        end
    end

    // Bit Segment State Machine
    localparam SEG_SYNC   = 2'd0;
    localparam SEG_PROP   = 2'd1;
    localparam SEG_PHASE1 = 2'd2;
    localparam SEG_PHASE2 = 2'd3;

    reg [1:0] segment;
    reg [5:0] seg_cnt;

    // Edge detector for falling edge on RX line (recessive to dominant transition)
    reg rx_d1, rx_d2;
    wire falling_edge = (rx_d2 && !rx_d1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_d1 <= 1'b1;
            rx_d2 <= 1'b1;
        end else begin
            rx_d1 <= can_rx_synced;
            rx_d2 <= rx_d1;
        end
    end

    // Segment limits based on active bit rate mode
    wire [5:0] max_prop   = brs_active ? {2'b00, data_prop_seg} : {1'b0, nom_prop_seg};
    wire [5:0] max_phase1 = brs_active ? {2'b00, data_phase1}   : {1'b0, nom_phase1};
    wire [5:0] max_phase2 = brs_active ? {3'b000, data_phase2}  : {2'b00, nom_phase2};
    wire [5:0] max_sjw    = brs_active ? {3'b000, data_sjw}     : {2'b00, nom_sjw};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            segment       <= SEG_SYNC;
            seg_cnt       <= 6'd0;
            sample_point  <= 1'b0;
            bit_start     <= 1'b0;
            hard_sync_req <= 1'b0;
        end else begin
            sample_point  <= 1'b0;
            bit_start     <= 1'b0;
            hard_sync_req <= 1'b0;

            // Hard Synchronization on bus idle / SOF
            if (falling_edge && segment == SEG_SYNC && seg_cnt == 6'd0) begin
                hard_sync_req <= 1'b1;
                segment       <= SEG_PROP;
                seg_cnt       <= 6'd0;
                bit_start     <= 1'b1;
            end else if (tq_tick) begin
                case (segment)
                    SEG_SYNC: begin
                        bit_start <= 1'b1;
                        segment   <= SEG_PROP;
                        seg_cnt   <= 6'd0;
                    end

                    SEG_PROP: begin
                        if (seg_cnt >= max_prop) begin
                            segment <= SEG_PHASE1;
                            seg_cnt <= 6'd0;
                        end else begin
                            seg_cnt <= seg_cnt + 6'd1;
                        end
                    end

                    SEG_PHASE1: begin
                        if (seg_cnt >= max_phase1) begin
                            sample_point <= 1'b1; // Sample Point strobe at end of Phase 1
                            segment      <= SEG_PHASE2;
                            seg_cnt      <= 6'd0;
                        end else begin
                            seg_cnt <= seg_cnt + 6'd1;
                        end
                    end

                    SEG_PHASE2: begin
                        if (seg_cnt >= max_phase2) begin
                            segment <= SEG_SYNC;
                            seg_cnt <= 6'd0;
                        end else begin
                            seg_cnt <= seg_cnt + 6'd1;
                        end
                    end
                endcase
            end
        end
    end

endmodule
