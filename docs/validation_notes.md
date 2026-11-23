# Validation notes

Status: **FPGA-validated** (maintainer confirmation, 2026-11-23).

The maintainer confirms FPGA testing. The supplied private host targets the AMD/Xilinx Kria KR260 with PYNQ 3.x, a 100 MHz PL clock, an STFT2 overlay and two simple-mode AXI DMA paths. Host defaults are N=2048, H=N/4, BATCH=1 and 48 kHz stereo. These are configuration values, not a measurement log proving every configuration was tested.

Private inputs include STFT/STFT2 bitstream and hardware-handoff pairs, Python verification code, coefficient tables and rendered WAV outputs. These are excluded from publication. The private README contains an older mock-only status predating the maintainer's confirmation. Artifact presence alone does not establish numerical correctness.

Exact Vivado version, FPGA part, board-run date, console results, WNS and resource utilization were not provided as a retained report. No numerical accuracy or performance figures are inferred. Reproducing the full board setup requires equivalent block-design integration; no recreation script or board constraints were supplied.

During publication preparation, executable RTL/testbench tokens were compared after removing comments and matched the supplied originals. ROM and stimulus data are unchanged and stored separately under `rtl/mem/` and `tb/mem/`. The runner stages both sets into its simulation working directory without changing RTL/testbench filename defaults. Python, internal rule documents, bitstreams and rendered audio are not included.

Icarus Verilog and Vivado were not found on the preparation shell PATH. No new simulation, synthesis, timing or FPGA result is claimed. The supplied XSim runner has not been executed in this environment.
