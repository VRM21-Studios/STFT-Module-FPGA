# Design Rationale

The module hierarchy, dataflow and memory organization are documented in [architecture.md](architecture.md). This document explains the mathematical and hardware choices behind that structure.

## Processing Model

For transform length N, hop H and frame index f, the framer produces x[fH+n] for n=0..N-1. Its ring buffer holds 2*MAX_N stereo words. It waits for N samples before the first frame, then triggers at each hop boundary. When a previous frame is still being issued, input backpressure prevents overwrite of pending data.

Analysis is `vrm_stft_framer` → `vrm_windowing_core` → lane mapping → `vrm_fft_stream_core`. Synthesis is inverse `vrm_fft_stream_core` → lane mapping → `vrm_windowing_core` → `vrm_overlap_add`.

## Periodic Window and Overlap-Add

The intended periodic window is:

$$
w_N[n]=\sqrt{\frac{1-\cos(2\pi n/N)}{2}},\qquad 0\le n<N.
$$

The implementation stores a 2048-entry quantized window and selects coefficients at stride MAX_N/N for smaller transforms. For K=N/H overlapping frames, the steady-state squared-window sum is:

$$
\sum_{j=0}^{K-1}w_N^2[n+jH]=\frac{N}{2H}=\frac{K}{2}.
$$

Analysis and synthesis each apply one copy of the window. Overlap-add divides by K/2 through an additional right shift of hop_code-1. Unity gain follows in exact arithmetic for an identity spectral path and a fully populated overlap region. Startup, tails, quantization and saturation affect actual output.

## Two-Real Stereo Transform

The FFT consumes z[n]=w[n](xL[n]+j*xR[n]). By linearity, Z[k]=XL[k]+j*XR[k], where XL and XR are the individual windowed-channel transforms. Recover them using the conjugate partner at (-k) mod N:

$$
X_L[k]=\frac{Z[k]+Z^*[(-k)\bmod N]}{2},\qquad
X_R[k]=\frac{Z[k]-Z^*[(-k)\bmod N]}{2j}.
$$

Individual channel spectra have conjugate symmetry; the combined Z generally does not. Treating Z as one real-channel spectrum mixes stereo channels incorrectly. After separate processing form Y=YL+j*YR for synthesis. Preserve each channel's conjugate symmetry when a real reconstruction is intended.

## Transform Architecture

The stream core loads a frame, starts an iterative radix-2 DIF transform and streams natural-order bins after bit reversal. It uses frame RAM and a twiddle ROM. The engine reuses a multiplier across the real products for complex multiplication.

Its control moves through load, run, prefetch and output. It does not accept the next frame concurrently with output from the same transform instance. Arithmetic reuse trades throughput for resources. Sustainable audio rate depends on N, H, clock rate and stalls rather than stream width alone.

## Overlap-Add Memory

The synthesizer uses a circular N-word accumulator with two lanes. For each frame the first H positions are complete and are emitted and cleared; remaining positions retain partial sums. The pointer advances by H. Accumulator width is IN_W+MAX_HOP_LOG2, giving 27 bits per lane in the supplied configuration.

The first frame after reset masks old accumulator contents rather than clearing the whole RAM. Subsequent frames use accumulated values from the new processing history. Valid/control state defines which memory contents are meaningful.

## Control and Recovery

The shared controller keeps shadow and active settings. FFT length, hop and batch commit together when all three are legal. Watchdog configuration is applied on every commit, including an invalid geometry commit. Commit, soft reset and watchdog trip flush state rather than drain a packet.

The watchdog observes stalled stream work and, for synthesis, partially received frames. A handshake or transform-busy state counts as progress. It recovers stalls; it does not time out FFT execution while busy remains asserted.

## Reuse Boundaries

The IPs may be instantiated separately around an external spectral stage. System clocks/resets, DMA setup and finite-audio boundaries belong to the integrator. The existing helper `fft_stage_iterative` retains its name for compatibility. Its standalone defaults differ from those explicitly selected by the STFT chains.
