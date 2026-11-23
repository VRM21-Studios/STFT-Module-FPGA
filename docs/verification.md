# Verification Guide

## Supplied Testbenches

| File | Purpose |
|---|---|
| tb_stft_istft.sv | Direct-core identity chain, randomized stalls and CSV capture |
| tb_stft_istft_axi.sv | AXI control/status, streaming, packetization and recovery |
| tb_axil_master.sv | AXI4-Lite bus-functional master |
| tb/mem/stim_sine.mem / tb/mem/stim_noise.mem | Packed stereo hexadecimal stimuli |

The benches use SystemVerilog, including typed string parameters. Use a simulator supporting those constructs. The default runner targets Vivado Simulator; no simulator-compatibility changes were made to the original benches.

## AXI Integration Coverage

| Scenario | Stimulus and checks |
|---|---|
| T1 | v2 IDs, defaults, shadow/active values, invalid geometry, W1C, watchdog clamp/disable, unmapped reads |
| T2 | N=2048, H=N/2, B=1; randomized gaps/stalls and register polling |
| T3 | Partial iSTFT frame triggers watchdog; recovered hop compared with T2 |
| T4 | Blocked STFT output triggers watchdog flush |
| T5 | Out-of-range iSTFT input triggers saturation status/counter |
| T6 | N=256, H=N/2, B=1 with noise |
| T7 | N=2048, H=N/4, B=1 with sine |
| T8 | N=2048, H=N/4, B=4 with noise |
| T9 | N=1024, H=N/8, B=3 with sine |
| T10 | N=256, H=N/4, B=2; deliberate per-frame tlast tests error reporting |

Stream monitors check payload stability under stalls and zero invalid output. These scenarios are not an exhaustive matrix of all geometry and arithmetic inputs.

## Running the AXI Test

From the repository root in a Vivado-configured PowerShell shell:

```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_sim.ps1
```

The runner compiles the RTL and SystemVerilog files, elaborates `tb_stft_istft_axi` and runs to completion. It copies ROM tables from `rtl/mem/` and stimuli from `tb/mem/` into `build/sim`, where outputs are generated. The staged filenames match the existing RTL and testbench defaults. The runner has passed PowerShell syntax parsing but has not been executed with Vivado in the preparation environment.

The AXI bench defaults to NS_MAIN=12000 and NS_ALT=6000. Optional plusargs also include TIMEOUT. Changes must respect available stimulus length. The bench adjusts each streaming run to complete whole packets.

## Direct-Core Test

Elaborate `tb_stft_istft` with the same compiled sources and staged data. From `build/sim` after compilation:

```powershell
xelab tb_stft_istft -s stft_core_sim
xsim stft_core_sim -runall
```

| Plusarg | Default | Meaning |
|---|---|---|
| STIM | stim_sine.mem | Packed stereo source |
| NSAMP | 48000 | Input samples |
| N | 2048 | FFT size |
| HOP | 1 | Hop code 1/2/3 |
| STRESS | 1 | Randomized stalls |
| OUT | out.csv | Reconstructed audio |
| SPEC | spec.csv | Captured spectrum |
| PERF | perf.csv | Frame-end timestamps |
| SPEC_FRAMES | 3 | Captured spectral frames |

The direct bench's PASS is a stream-level result. It does not independently prove floating-point reconstruction accuracy.

## Acceptance and Outputs

Require the final `[TB] RESULT: PASS`, no timeout and zero unexpected protocol or AXI response errors. Deliberate saturation, watchdog and tlast-error scenarios are expected events checked by the AXI bench. Both benches use printed summaries and `$finish`; a zero simulator exit code alone is insufficient.

AXI CSV files include `axi_spec_<tag>.csv` with `frame,k,re,im` and `axi_out_<tag>.csv` with `idx,L,R`. The direct bench emits audio, spectrum and performance CSVs. Generated output files are ignored by Git.

Numerical validation requires an independent fixed-point model with identical coefficients, shifts, rounding, saturation and startup alignment. Compare steady-state reconstruction after handling overlap startup and tail padding. The private Python environment is excluded; no public golden-comparison script or retained pass log is supplied.

## Available Results

The maintainer confirms FPGA validation. No new simulation was performed during publication preparation; the table describes supplied bench coverage rather than a newly executed run. See [validation notes](validation_notes.md).
