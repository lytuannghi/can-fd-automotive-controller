#!/usr/bin/env python3
"""
Simulation Automation Script for ISO 11898-1:2015 CAN-FD Automotive Controller ASIC
Compiles synthesizable RTL and self-checking testbench using Icarus Verilog and executes VVP.
"""

import subprocess
import sys
import os

def run():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_dir = os.path.join(repo_root, "sim")
    waves_dir = os.path.join(sim_dir, "waves")
    os.makedirs(waves_dir, exist_ok=True)

    rtl_files = [
        os.path.join(repo_root, "rtl", "bit_timing", "can_bit_timing.v"),
        os.path.join(repo_root, "rtl", "crc", "can_crc_engine.v"),
        os.path.join(repo_root, "rtl", "tdc", "can_tdc_engine.v"),
        os.path.join(repo_root, "rtl", "filter", "can_acceptance_filter.v"),
        os.path.join(repo_root, "rtl", "fault", "can_fault_confinement.v"),
        os.path.join(repo_root, "rtl", "fifo", "can_fifo_buffer.v"),
        os.path.join(repo_root, "rtl", "protocol", "can_fd_fsm.v"),
        os.path.join(repo_root, "rtl", "can_fd_controller_top.v"),
    ]

    tb_file = os.path.join(repo_root, "tb", "tb_can_fd_controller.v")
    out_vvp = os.path.join(sim_dir, "can_fd_sim.vvp")

    print("[INFO] Compiling CAN-FD Controller ASIC RTL and Testbench...")
    cmd_compile = ["iverilog", "-g2012", "-o", out_vvp, "-I", os.path.join(repo_root, "rtl")] + rtl_files + [tb_file]
    res_compile = subprocess.run(cmd_compile, capture_output=True, text=True)

    if res_compile.returncode != 0:
        print("[ERROR] Compilation failed:")
        print(res_compile.stderr)
        sys.exit(1)
    print("[INFO] Compilation successful!")

    print("[INFO] Running VVP simulation...")
    cmd_run = ["vvp", out_vvp]
    res_run = subprocess.run(cmd_run, cwd=repo_root, capture_output=True, text=True)

    print(res_run.stdout)
    if res_run.stderr:
        print("[STDERR]:", res_run.stderr)

    if "ALL 10 VERIFICATION TESTS COMPLETED WITH 100% PASS RATE" in res_run.stdout:
        print("[SUCCESS] All 10 CAN-FD test cases PASSED!")
        sys.exit(0)
    else:
        print("[FAILURE] Simulation failed assertion checks.")
        sys.exit(1)

if __name__ == "__main__":
    run()
