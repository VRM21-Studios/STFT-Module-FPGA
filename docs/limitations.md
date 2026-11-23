# Limitations and Integration Constraints

| Area | Constraint |
|---|---|
| Runtime geometry | Supplied wrappers accept N=256/512/1024/2048, hop codes 1/2/3 and B=1..16 |
| Coefficients | Supplied tables target MAX_N=2048 and 18-bit components |
| Coefficient updates | File initialization; no upload through the STFT register map |
| Arithmetic | Rounding and saturation introduce error; identity is not assumed bit-exact to PCM input |
| Throughput | Iterative frame processing; no universal sample-rate or cycle-latency guarantee |
| Stereo | Two-real packing requires conjugate-pair handling for independent channels |
| Finite streams | Explicit priming, padding, trimming and whole-batch completion |
| Reconfiguration | Commit flushes state; no seamless active-audio transition |
| Recovery | Flush may abort packets; software must recover pending DMA transfers |
| Clocks | Single-clock IP; external crossings need appropriate bridges |
| Board recreation | No board XDC, complete Vivado project or block-design recreation Tcl supplied |
| Public software | Private PYNQ application and numerical golden model excluded |
| Measurements | No retained timing/utilization or quantitative accuracy report supplied |

Supported geometry applies to the supplied parameterization. A smaller MAX_N is not automatically safe because the controller's accepted FFT sizes remain fixed. A larger MAX_N or altered coefficient width needs matching tables and renewed review of addressing and scaling.

The helper `fft_stage_iterative` retains its supplied name. Standalone helper defaults differ from explicit STFT/iSTFT instantiations, especially coefficient filename/width and inverse scaling. Integrate through the documented tops unless those differences have been reviewed.
