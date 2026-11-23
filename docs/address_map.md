# AXI4-Lite Address Map

Both wrappers use these local byte offsets with a 32-bit data bus and default 6-bit address bus. Decoding uses bits [5:2]; software should issue word-aligned accesses. One read and one write may be outstanding. AW and W are captured independently. Responses are OKAY; unmapped reads return zero and unmapped writes have no register effect.

## Registers

| Offset | Name | Access | Reset | Meaning |
|---:|---|---|---|---|
| 0x00 | CTRL | RW / command | 0 | Enable, soft reset, commit |
| 0x04 | STATUS | RO / W1C | 0 | Live status and sticky events |
| 0x08 | FFT_N | RW | 2048 | Shadow transform length |
| 0x0C | FFT_N_ACT | RO | 2048 | Active transform length |
| 0x10 | WDOG | RW | 0x08000000 | Shadow stall limit, cycles |
| 0x14 | WDOG_ACT | RO | 0x08000000 | Active stall limit |
| 0x18 | FRAME_CNT | RO | 0 | Accepted output frame/hop completions |
| 0x1C | SAT_CNT | RO | 0 | Saturation event cycles |
| 0x20 | WDOG_CNT | RO | 0 | Watchdog trips |
| 0x24 | BUSY_CYC | RO | 0 | Last transform-busy duration |
| 0x28 | ID | RO | IP-specific | STFT 0x53544602; iSTFT 0x49534602 |
| 0x2C | HOP | RW | 1 | Shadow log2(N/H) |
| 0x30 | HOP_ACT | RO | 1 | Active hop code |
| 0x34 | BATCH | RW | 1 | Shadow frames per packet |
| 0x38 | BATCH_ACT | RO | 1 | Active frames per packet |

The shared controller's standalone IP_ID default differs from the v2 wrapper overrides. Use the wrapper IDs above for identification.

## CTRL Fields

| Bit | Meaning | Behavior |
|---:|---|---|
| 0 | Enable | Read/write; each CTRL write replaces this state |
| 1 | Soft reset | Write one to flush and clear event counters/flags |
| 2 | Commit | Write one to apply shadow settings and flush |

Command bits read as zero. Common writes are 0 to disable, 1 to enable without commit, 3 to enable with soft reset and 5 to enable with commit. Writing only bit 1 or 2 also writes enable=0. Commands require WSTRB[0].

Soft reset clears FRAME_CNT, SAT_CNT and WDOG_CNT. It does not load shadow geometry. Commit leaves event counters intact. BUSY_CYC follows its interval tracker; do not assume the soft-reset command clears it.

## STATUS Fields

| Bit | Meaning | Access |
|---:|---|---|
| 0 | FFT engine busy | RO |
| 1 | Core running / out of internal reset | RO |
| 2 | Saturation observed | W1C |
| 3 | tlast position error observed | W1C |
| 4 | Watchdog trip observed | W1C |
| 5 | Invalid geometry on a commit | W1C |

Write ones to bits 2–5 to clear selected flags. A concurrent event takes priority. Reads do not acknowledge events. Busy=0 does not prove the entire pipeline drained: framing or output stages can still hold data.

## Geometry Commit

Legal FFT_N values are 256, 512, 1024 and 2048. HOP codes are 1, 2 or 3. BATCH is 1–16 with the supplied wrappers. Geometry updates atomically only if all three are legal. Otherwise active geometry is retained and STATUS[5] is set. A rejected commit still flushes the core.

HOP/BATCH writes with bits above their storage field set are stored as zero for rejection. FFT_N retains the low 16 bits. Write complete legal values and verify active readback.

WDOG applies on every commit even if geometry is invalid. Zero disables it; nonzero values below 4096 are clamped to 4096. Shadow FFT_N and WDOG writes use byte-masked merging; HOP and BATCH writes use WSTRB[0].

## Watchdog and Counters

The watchdog counts consecutive cycles with pending work and no progress. Stream handshakes or FFT busy count as progress. A partially received synthesis frame also counts as pending. A trip flushes the core, sets STATUS[4] and increments WDOG_CNT. It cannot detect a transform stuck with busy permanently asserted.

The default 0x08000000 cycles is 1.34217728 seconds at 100 MHz. Choose a limit appropriate for host scheduling and DMA delays. A flush may leave a DMA transaction without tlast, so recover the DMA with the IP.

FRAME_CNT increments at the final accepted STFT bin or iSTFT hop sample, including frames inside a batch. SAT_CNT counts the combined event per cycle. The 32-bit counters wrap on overflow. BUSY_CYC describes transform computation, not the full audio transaction.
