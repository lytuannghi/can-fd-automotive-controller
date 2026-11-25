# ==============================================================================
# Synopsys Design Constraints (SDC)
# Target: ISO 11898-1:2015 CAN-FD Automotive Controller ASIC Core
# Process Technology: SkyWater 130nm HD Standard Cell Library (1.80V, 25C)
# Operating Conditions: Fast-Fast (0.99V, -40C) / Slow-Slow (0.81V, 150C AEC-Q100 Grade 0)
# Target Frequency: 100.00 MHz (Clock Period = 10.000 ns)
# ==============================================================================

create_clock -name clk -period 10.000 [get_ports clk]

# Clock Uncertainty, Jitter & Duty Cycle Distortion
set_clock_uncertainty -setup 0.200 [get_clocks clk]
set_clock_uncertainty -hold 0.050 [get_clocks clk]
set_clock_transition 0.120 [get_clocks clk]

# Physical CAN Interface & Host Register Bus Constraints
set_input_delay -clock clk -max 2.000 [get_ports {can_rx host_tx_req host_tx_id host_tx_ide host_tx_fdf host_tx_brs host_tx_dlc host_tx_payload rx_fifo_pop}]
set_input_delay -clock clk -min 0.400 [get_ports {can_rx host_tx_req host_tx_id host_tx_ide host_tx_fdf host_tx_brs host_tx_dlc host_tx_payload rx_fifo_pop}]

set_output_delay -clock clk -max 2.000 [get_ports {can_tx tx_busy tx_done rx_fifo_not_empty rx_fifo_id rx_fifo_dlc rx_fifo_payload diag_tec diag_rec diag_err_warning diag_err_passive diag_bus_off diag_tdc_delay}]
set_output_delay -clock clk -min 0.300 [get_ports {can_tx tx_busy tx_done rx_fifo_not_empty rx_fifo_id rx_fifo_dlc rx_fifo_payload diag_tec diag_rec diag_err_warning diag_err_passive diag_bus_off diag_tdc_delay}]

# Drive Strength & Capacitive Load Constraints
set_driving_cell -lib_cell INVX4 [all_inputs]
set_load -pin_load 0.040 [all_outputs]

# Timing Exceptions
set_false_path -from [get_ports rst_n]
