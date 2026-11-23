# Latency and Data Format

## Audio and Spectrum Packing

| Interface | Bits | Interpretation |
|---|---|---|
| Audio | [15:0] | Signed int16 left, Q1.15 |
| Audio | [31:16] | Signed int16 right, Q1.15 |
| AXI wrapper spectrum | [31:0] | Signed int32 real |
| AXI wrapper spectrum | [63:32] | Signed int32 imaginary |
| Direct-core spectrum | [DATA_W-1:0] | Signed imaginary |
| Direct-core spectrum | [2*DATA_W-1:DATA_W] | Signed real |

The AXI wrapper layout gives little-endian software an int32 sequence `[re0, im0, re1, im1, ...]`. Every spectrum beat is one complex bin. All N bins are transferred; this is not a half-spectrum interface.

## Default Numeric Contract

| Quantity | Width / signedness | Scale or operation |
|---|---|---|
| PCM input/output | 16-bit signed | Integer / 2^15 |
| Internal samples | 24-bit signed | Integer / 2^22 |
| Window coefficient | 18-bit unsigned | Integer / 2^17 |
| Twiddle component | 18-bit signed | Integer / 2^17 |
| OLA accumulator lane | 27-bit signed | Same fractional scale as internal samples |
| AXI spectrum component | 32-bit signed | Sign extension of internal 24-bit component |

The unsigned window representation can express +1 exactly. Signed twiddle components have the usual asymmetric signed fixed-point range. The 32-bit spectrum container is not Q1.31.

For the analysis window, SHIFT=15+17-22=10. The product of PCM integer and window integer is rounded and shifted by 10 into the internal scale. The synthesis window uses SHIFT=17. Final overlap-add uses OUT_SHIFT=22-15=7 plus hop_code-1 for normalization.

The forward FFT divides by two per radix-2 stage, producing DFT/N. The inverse path selects IFFT_SCALE=0 and does not divide per stage. For int16-valued xL and xR, the nominal analysis integer scale is:

$$
Z_{int}[k]\approx\frac{128}{N}\operatorname{DFT}\{w[n](xL[n]+j xR[n])\}[k].
$$

This expression describes the scale; fixed-point rounding and saturation prevent exact equality in general.

## Rounding and Saturation

The STFT chain enables round-half-up on applicable right shifts: add a positive half-step, then arithmetic-shift. Negative exact ties therefore round toward positive infinity. This is not round-to-even.

Window products, butterfly results and final audio are saturated to their output widths. The inverse AXI boundary also saturates signed int32 input components into DATA_W. SAT_CNT counts cycles with a combined saturation event, rather than every clipped lane or individual arithmetic operation. A sticky flag records that at least one such event occurred.

## Frame Geometry and Latency

The first analysis frame requires N accepted samples. A host can prime N-H samples, then supply H more for each output frame. Analysis emits N bins and synthesis emits H samples per frame.

For a finite accepted input of S>=N samples, the number of complete frames is:

$$
F=\left\lfloor\frac{S-N}{H}\right\rfloor+1.
$$

The first K-1 synthesis hops after flush contain fewer than K overlapping frames. Under the identity model, the fully populated overlap region begins at output index N-H. Appending padding can generate missing trailing frames; trimming must follow the application's chosen signal alignment.

There is no single fixed cycle latency for the subsystem. Frame collection, iterative FFT execution, output serialization and backpressure all contribute. At sample rate fs, H/fs is the input hop period; it is a processing budget, not a measured hardware latency. For N=2048 at 48 kHz:

| H | Hop period | Overlap |
|---:|---:|---:|
| 1024 | 21.333 ms | 50% |
| 512 | 10.667 ms | 75% |
| 256 | 5.333 ms | 87.5% |

BUSY_CYC measures the last continuous FFT-engine busy interval. It excludes frame collection and output stalls, so it is not an end-to-end latency measurement. No measured cycle counts or maximum sample rate are published here.
