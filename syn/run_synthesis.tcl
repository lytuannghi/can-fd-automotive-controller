# ==============================================================================
# Synthesis Automation TCL Script
# Design: can_fd_controller_top
# ==============================================================================

set DESIGN_NAME "can_fd_controller_top"

set_app_var target_library "slow_leakage_1v08.db"
set_app_var link_library "* slow_leakage_1v08.db dw_foundation.sldb"

analyze -format verilog {
    ../rtl/bit_timing/can_bit_timing.v
    ../rtl/crc/can_crc_engine.v
    ../rtl/tdc/can_tdc_engine.v
    ../rtl/filter/can_acceptance_filter.v
    ../rtl/fault/can_fault_confinement.v
    ../rtl/fifo/can_fifo_buffer.v
    ../rtl/protocol/can_fd_fsm.v
    ../rtl/can_fd_controller_top.v
}

elaborate $DESIGN_NAME
current_design $DESIGN_NAME
link
check_design > reports/check_design.rpt

read_sdc can_fd_timing.sdc

compile_ultra -gate_clock

report_timing -delay_type max -max_paths 10 > reports/timing_report.txt
report_area -hierarchy > reports/area_report.txt
report_power -hierarchy > reports/power_report.txt
report_qor > reports/qor_report.txt

write -format verilog -hierarchy -output netlist/can_fd_controller_top_netlist.v
write_sdc netlist/can_fd_controller_top_synthesized.sdc
exit
