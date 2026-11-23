# Build and Hardware Integration

## Source Setup

Use the RTL in `rtl/` as synthesis sources. The two top-level IPs are `vrm_stft_axi` and `vrm_istft_axi`; their dependencies are included in that directory. Simulation sources stay under `tb/`; simulation-only stimulus data stays under `tb/mem/`.

Add `rtl/mem/window_2048_q17.mem` and `rtl/mem/twiddle_2048_q17.mem` as memory initialization files and keep their names resolvable by `$readmemh`. The window has 2048 entries; the twiddle table has 1024 complex entries for the maximum-size transform. Missing tables can invalidate processing even when control registers respond correctly.

The window helper's standalone default `window_2048.mem` is not supplied. The STFT/iSTFT chains explicitly override it with `window_2048_q17.mem` and 18-bit coefficients.

## Top-Level Parameters

| Parameter | Supplied default | Integration meaning |
|---|---|---|
| C_S_AXI_ADDR_WIDTH | 6 | AXI-Lite local byte-address width |
| MAX_N | 2048 | Maximum storage and coefficient geometry |
| DATA_W | 24 | Internal signed component width |
| TWID_W | 18 | Signed twiddle component width |
| COEF_W | 18 | Unsigned window coefficient width |
| WIN_FILE | window_2048_q17.mem | Window initialization |
| TWID_FILE | twiddle_2048_q17.mem | Twiddle initialization |

These are elaboration parameters, separate from runtime FFT_N/HOP/BATCH registers. The supplied integration uses this combination. Altering widths or MAX_N requires matching coefficients and renewed review; generic parameter syntax is not evidence that arbitrary combinations work.

## Block-Design Integration

Use a common stream/control clock and appropriately synchronized active-low reset. No asynchronous clock crossing is implemented inside the IPs.

The private KR260 host setup uses:

| DMA path | MM2S side | S2MM side |
|---|---|---|
| axi_dma_0 | 32-bit audio to STFT | 64-bit STFT spectrum |
| axi_dma_1 | 64-bit spectrum to iSTFT | 32-bit reconstructed audio |

The host setup uses simple-mode DMA, a 26-bit length field and polled completion. Connect AXI4-Lite control to the processor address space. Local register offsets do not prescribe system base addresses; assign those in the block design.

The host configuration specifies a 100 MHz PL clock. It is a configuration reference, not a published timing result. Clock constraints and system implementation remain part of the board project.

## Bring-Up Sequence

1. Build/load an overlay with matching hardware-handoff metadata.
2. Confirm DMA widths and packet-length capacity.
3. Read the two v2 IP IDs and check reset defaults.
4. Disable streaming and program matching geometry.
5. Write CTRL=5 on each IP, then verify active values and running status.
6. Arm receive transfers, prime analysis and run identity reconstruction.
7. Inspect flags/counters and compare aligned samples before adding spectral processing.

For N=2048, H=N/4 and B=1, configure FFT_N=2048, HOP=2 and BATCH=1. Choose a watchdog limit appropriate for the software schedule and commit it with the geometry.

## Reconfiguration and Recovery

Runtime writes affect shadow registers until commit. Commit resets framing and overlap history, so use it at a quiescent boundary and re-prime afterward. It is not a seamless parameter transition.

After watchdog or software reset, recover unfinished DMA transactions as well as the IPs. A core flush does not guarantee the DMA received a terminating marker. See the [streaming contract](streaming_contract.md).

## Package Boundaries

The public package excludes the private host, bitstreams, hardware-handoff files and rendered audio. A complete Vivado project, block-design recreation Tcl and board XDC were not supplied. Create equivalent integration from the documented interfaces.
