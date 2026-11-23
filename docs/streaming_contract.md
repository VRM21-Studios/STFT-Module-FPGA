# Streaming and DMA Contract

## Handshake and Packet Boundaries

A transfer occurs when tvalid and tready are both asserted at a clock edge. While stalled, a source must hold payload and tlast stable. The wrappers suppress valid output while disabled or flushing. Normal backpressure preserves stream state; reset, commit and watchdog flush intentionally discard it.

All N complex bins are transferred in natural order. Direct cores use per-frame tlast. Wrapper batching changes packet markers without changing frame boundaries inside the processing chain.

| Stream | Beats per packet | Bytes per beat | Packet bytes |
|---|---:|---:|---:|
| STFT spectrum output | B*N | 8 | B*N*8 |
| iSTFT spectrum input | B*N | 8 | B*N*8 |
| iSTFT audio output | B*H | 4 | B*H*4 |
| Steady-state STFT audio input | B*H | 4 | B*H*4 |

The analysis input ignores tlast and accepts continuous audio. Its first frame requires N samples; the steady-state B*H input size applies after priming N-H samples. The priming segment itself produces no complete frame.

## Example: N=2048, H=N/4

| Batch | Steady-state input audio | Spectrum | Output audio |
|---:|---:|---:|---:|
| 1 | 512 samples / 2048 bytes | 2048 bins / 16384 bytes | 512 samples / 2048 bytes |
| 4 | 2048 samples / 8192 bytes | 8192 bins / 65536 bytes | 2048 samples / 8192 bytes |
| 16 | 8192 samples / 32768 bytes | 32768 bins / 262144 bytes | 8192 samples / 32768 bytes |

Priming is 1536 stereo samples, or 6144 bytes. A 14-bit DMA length field cannot describe one 16384-byte spectrum frame. The private host configuration uses a 26-bit length field. Verify that the entire selected packet fits the configured DMA capacity.

## Recommended Transaction Sequence

1. Quiesce the existing stream and ensure no previous DMA transfer is pending.
2. Program both IPs with matching N, HOP and BATCH; commit while idle.
3. Poll running and active geometry. Clear/check sticky flags.
4. For analysis, arm spectrum receive before submitting priming plus audio input.
5. For synthesis, arm audio receive before submitting spectrum input.
6. Wait for both directions to complete with bounded host-side timeouts.
7. Check frame counts, saturation, tlast errors and watchdog status.

A host using intermediate spectral processing may execute the two IP paths sequentially. A direct hardware link can stream between them when widths, packing and packet geometry match.

## Finite Inputs and Draining

With S input samples, F=floor((S-N)/H)+1 complete frames exist for S>=N. Those frames produce F*H synthesis samples. Samples left after the last complete analysis frame do not create another frame automatically.

To finish a wrapper packet without an open batch, choose F as a multiple of B. Exactly S=N+(F-1)*H samples generates F frames. Finite audio reconstruction also needs tail padding to supply trailing overlap contributions and a documented trim/alignment rule. Batch completion and waveform reconstruction are distinct requirements.

The first K-1 output hops after reset have incomplete overlap. Compare numerical identity over the fully populated region, then evaluate startup and tail behavior separately. Do not compare unaligned finite buffers when evaluating gain or reconstruction error.

## Reset and Recovery

A commit changes geometry by flushing state, rather than waiting for a packet boundary. A watchdog trip likewise aborts the current transaction. The stream may stop before a DMA receive obtains tlast. Stop/reset affected DMA channels, reconfigure both IPs as needed, discard partial buffers and re-prime analysis before restarting.

The inverse wrapper counts N input bins per frame regardless of incoming tlast. A wrong marker sets an error; it does not redefine frame length. Sending tlast on every frame when B>1 violates the packet contract even though the transform may continue consuming samples.
