# CAN comparison: current project vs h7_zdt_test

## Debugger evidence: task stack overflow

The next hardware screenshot shows `g_boot_stage=0x11` (display task entered)
and `g_boot_error=5` (FreeRTOS stack overflow hook). This establishes a detected
task stack overflow; the clock was not the demonstrated reason for that halt.
The display task had only 512 bytes; ARMCC reports a 440-byte call chain plus
unknown stack usage, excluding the additional task/exception context margin.
Its allocation is now 2048 bytes, also synchronized into the root `.ioc`.
`g_overflow_task_name` and `g_overflow_task` retain the failing task on any new
overflow. `g_display_stack_free_words` records the display task's minimum unused
stack in 32-bit words after each refresh. Hardware confirmation is pending.

## Hardware follow-up: reference-clock build failed to boot

The user flashed the first comparison build and reported no display and no
UART output. Its HSI/PLL2 profile is therefore NOT a working fix. The default
has been restored to `CAN_USE_REFERENCE_CLOCK=0`: original HSE, original PLL1
and original CAN timing on both buses. The exact startup failure has not been
located with a debugger. The original ACK-error problem is still unresolved.

Use `build/can_recovery_hse25.hex` for this recovery iteration. Expect
`clock=25000000 bitrate=500000`, not the 80 MHz value described below.
The remaining sections describe the first comparison build and its rationale;
this follow-up supersedes its default clock and bench instructions.

## Evidence and scope

The user reports that the archived project works on the same board and wiring.
The original current-project log shows external loopback 10/10, followed by
LEC=3 (ACK error), TEC=128 or 176, REC=0, and occasionally LEC=5 (bit0 error).
This is a CAN link-layer failure. Enqueue success does not prove transmission
or motor acknowledgement. External loopback does not validate the external RX
path. The old diagnostic claiming that ACK error proves the transceiver works
was incorrect and has been replaced.

No hardware flash or live CAN measurement was performed in this session.
The changes below are a compiled comparison/debug build, not a confirmed
hardware fix. HSE frequency, signal integrity and mode-switch effects remain
unmeasured; do not label any of them the proven root cause yet.

## Comparison

| Item | Original current project | Working archive | Debug build |
| --- | --- | --- | --- |
| CAN peripheral/pins | FDCAN2 PB12/PB13 AF9 | Same | Same |
| Motor address / first packet ID | 3 / 0x300 | Same | Same |
| CAN clock | Assumed 25 MHz HSE | HSI -> PLL2Q, 80 MHz | Reference clock |
| Prescaler / Seg1 / Seg2 / SJW | 1 / 37 / 12 / 4 at runtime | 8 / 15 / 4 / 2 | Reference timing |
| Nominal rate / sample point | 500 kbps / 76% if HSE is 25 MHz | 500 kbps / 80% | 500 kbps / 80% |
| Automatic retry | Enabled | Disabled | Disabled |
| RX / TX FIFO elements | 16 / 16 at runtime | 16 / 3 | 16 / 3 |
| Startup | External loopback then normal | Normal | Normal |
| Receive notification | Watermark 1 | New message | New message |
| Initial query | Position / flags | Version: 1F 6B | Version, then position / flags |
| Console | UART4, 115200 8N1 | USART3 | UART4, 115200 8N1 |
| HAL DLC representation | n << 16 | Unshifted DLC code | Existing HAL encoding retained |

## Implementation

- `Core/Inc/fdcan.h`: `CAN_USE_REFERENCE_CLOCK=1` selects HSI/PLL2Q.
  Setting it to 0 selects the original HSE timing for a subsequent comparison.
- PLL1 and PLL2 share PLLSRC. PLL1 therefore also uses HSI in reference mode:
  64 MHz / 4 * (34 + 3072/8192) = 550 MHz. Existing CPU, APB and SPI nominal
  frequencies are preserved. HSI tolerance still applies; frequencies printed
  by HAL are calculated values, not measurements.
- FDCAN1 timing was updated for the shared 80 MHz clock: nominal 1 Mbps,
  optional FD data 2.5 Mbps. Its motor protocol remains unchanged.
- ZDT RAM now occupies words 0..75; DM starts at word 128. Neither layout
  overlaps. The old 16+16 ZDT allocation used words 0..127 and also did not
  overlap DM; RAM overlap was not established as the original failure.
- Startup loopback is disabled; optional loopback PASS requires all 10 frames.
  Loopback traffic is excluded from real-bus receive counters and reply parsing.
- CAN setup now checks receive-notification activation. `g` prints the selected
  clock, calculated bitrate, CCCR/TEST/NBTP/TXBRP, HAL errors and GPIO registers.
- UART5 fallback parsing resets the payload index after ID collection, retains
  the original DLC and rejects oversized payload lengths. Previously the index
  started at 4 and DLC was decremented to zero, causing overflow/missed replies.

The `.ioc` retains the pre-existing HSE configuration for now. The comparison
profile is implemented in C; CubeMX regeneration can overwrite the clock and
entry timing changes. Reconcile the `.ioc` after the bench result selects the
desired clock profile, before regenerating code.

## Bench verification

1. Flash `MDK-ARM/Engineering_Training_Project/Engineering_Training_Project.hex`.
2. Keep the wiring and motor configuration that worked with the archive.
3. Open UART4 at 115200 8N1. Expect `clock=80000000 bitrate=500000` and no
   startup loopback banner. Startup sends only read commands.
4. Send `q` to enable statistics, `b` for the exact reference version query,
   then `g` for the register/error snapshot. Do not use `u` during CAN comparison.
5. Success requires real `raw_rx` and `zdt_rx` increments and the
   `TX+RX PATH OK` message. `OK` on a command alone is not success.
6. If ACK errors persist, retain the full startup/q/g output. Compare actual
   PB13 bit timing and PB12 return waveform between both firmwares, then check
   physical CANH/CANL. The reported HAL clock cannot verify crystal frequency.

Keil ARMCC 5.06 build passed with 0 errors and 0 warnings.
Code=53090, RO-data=2906, RW-data=760, ZI-data=197000 bytes.
Build log: `build/can_compare_build.log`.
