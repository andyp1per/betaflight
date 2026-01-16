# Bidirectional DShot Telemetry on RP2350 - Implementation Notes

## Current Status (January 2026)

**Status:** Working at ~99.5% decode rate at 0 RPM, ~40% at speed. PC-based state machine management.
**Tested with:** BlueJay ESCs on RP2350B (HELLBENDER_0001 config) at 8kHz PID loop

## What Works

- **4x oversampling with edge detection** - PIO samples 128 bits, C code detects edges
- **Edge normalization** - Handles timing jitter in start bit detection
- **FIFO drain before transmit** - Prevents FIFO stalls mid-sample
- **ArduPilot-style run-length decoding** - Proven algorithm for edge-to-GCR conversion
- **`wait` instructions for edge sync** - More reliable than `jmp pin` (RP2350 erratum E9)

## Key Discoveries

### FIFO Synchronization is Critical
If the RX FIFO isn't drained before starting a new transmit, the PIO can stall
mid-sample when autopush blocks on a full FIFO. This causes "first word valid,
rest all 1s" corruption. **Solution:** Drain RX FIFO before `pio_sm_put()`, but only
when SM is in a safe state (PC=0 or PC>=23).

### PC-Based State Machine Management
The PIO state machine must be managed carefully to avoid duplicate pulses or missed updates:
- **PC=0**: SM is blocked on `pull`, ready for new data
- **PC=1-14**: SM is transmitting, don't interrupt
- **PC=15-16**: SM is waiting for ESC response (`wait` instructions)
- **PC=17-22**: SM is sampling telemetry, don't interrupt
- **PC=23-31**: SM finished receive, about to wrap to PC=0

**Strategy:**
- If PC=0 or PC>=23: Safe to drain RX FIFO and send new data
- If PC=15-16 for first time: Skip (normal ESC turnaround ~25µs)
- If PC=15-16 for 2+ consecutive calls: ESC didn't respond, restart SM
- If PC=1-14 or 17-22: Skip (mid-cycle, would cause issues)

This prevents duplicate pulses (which occurred when restarting during normal turnaround)
while still recovering from truly stuck states (ESC didn't respond).

### Edge Normalization Handles Timing Jitter
The `wait 0 pin` instruction detects the falling edge at slightly different points
relative to the actual start bit. This shifts the entire sampling window. We compensate
by normalizing edge positions so the first detected edge is always at position 8
(corresponding to bit 2 at 4x oversampling for the expected GCR pattern).

### RP2350 PIO Erratum E9
The `jmp pin` instruction is broken on RP2350. Use `wait 1 pin` / `wait 0 pin`
instead for edge detection. This provides precise, deterministic edge synchronization.

### GCR Encoding Details
- 21-bit telemetry: 1 start bit + 20 data bits (4 x 5-bit GCR symbols)
- Each GCR symbol decodes to 4-bit nibble
- Checksum: all 4 nibbles XOR together should equal 0xF
- eRPM=0 special value: 0xFFF (decoded 16-bit value is 0xFFF0)

## Build Commands

```bash
# Production build
make CONFIG=HELLBENDER_0001

# With debug output (for troubleshooting)
make CONFIG=HELLBENDER_0001 EXTRA_FLAGS="-DUSE_CLI_DEBUG_PRINT"
```

Output: `obj/betaflight_*_RP2350B_HELLBENDER_0001.uf2`

## PIO Program Structure (32 instructions)

```
Instructions 0-13:  Transmit (inverted DShot command)
Instruction  14:    set pindirs, 0 (switch to input mode)
Instruction  15:    wait 1 pin 0 (wait for line to go HIGH - idle)
Instruction  16:    wait 0 pin 0 (wait for falling edge - start bit)
Instructions 17-22: 4x oversampling loop (4 outer × 32 inner = 128 samples)
Instructions 23-31: Padding (nop to fill 32-instruction slot)
```

Key points:
- `wait` instructions provide precise edge sync (unlike broken `jmp pin`)
- 8 PIO cycles per sample at 4x oversampling rate
- Autopush at 32 bits sends 4 words to RX FIFO (fits default 4-word depth)
- FIFO must be drained before next TX to prevent autopush stalls

## Files

- `dshot.pio` - PIO assembly program (transmit + receive)
- `dshot_pio_programs.h` - Auto-generated header (regenerate after .pio changes)
- `dshot_pico.c` - Motor control, SM management, FIFO drain before TX
- `dshot_bidir_pico.c` - Telemetry decoder with edge detection algorithm

## Regenerating PIO Header

After modifying `dshot.pio`:
```bash
./tools/pioasm/pioasm -o c-sdk src/platform/PICO/dshot.pio src/platform/PICO/dshot_pio_programs.h
```

## Debugging (with USE_CLI_DEBUG_PRINT)

1. Build with `-DUSE_CLI_DEBUG_PRINT`
2. Connect via USB terminal and enter CLI (`#`)
3. Type `@` to enter Debug Mode
4. Observe debug output:
   - `Raw: ...` - 4 words of samples
   - `Edges(N): ...` - detected edge positions
   - `M%d: SUCCESS erpm=... (rate: ...)` - success with decode rate
   - `M%d FAIL Raw: ...` - failed decode with raw samples
   - `M%d: Invalid GCR: ...` - invalid GCR codes detected

## Known Issues / Future Work

- **~60% error rate when motor spinning** - At 0 RPM, error rate is ~0.5%. When motor
  is spinning, error rate jumps to ~60%. Likely timing-related - motor electrical noise
  or ESC response timing changes under load. Needs investigation.
- **~50% update skip rate** - At 8kHz PID loop, the ~95µs DShot cycle leaves only ~30µs
  margin. We catch the SM mid-cycle about half the time, resulting in skipped updates.
  Effectively running at ~4kHz motor updates. Could improve with faster PIO clock or
  lower PID rate.
- **RPM wrapping at high speed** - Reported values wrap past ~2000 RPM. May be
  eRPM to RPM conversion issue or EDT (Extended Telemetry) not being handled.

## Implementation History

1. **Fixed-Interval (3x Oversampling):** Initial attempt. High error rate.
2. **Fixed-Interval (Synchronized):** Fragile to ESC timing variations.
3. **PIO Edge Detection (jmp pin):** Didn't work - RP2350 erratum E9.
4. **Software Edge Detection Loop:** Worked but timing jitter issues.
5. **`wait` Instructions + Edge Normalization:** 98% success rate at 0 RPM.
6. **FIFO Drain Fix:** Resolved M2 channel failures (was 0%, now 98%).
7. **PC-Based SM Management:** Fixed duplicate pulses. Now 99.5% at 0 RPM.
   - Only send when SM at PC=0 or PC>=23 (safe states)
   - Track consecutive waits at PC=15-16 to detect stuck vs normal turnaround
   - Skip updates when SM mid-cycle to prevent duplicates

## Reference

- Betaflight STM32 decoder: `src/main/drivers/dshot_bitbang_decode.c`
- ArduPilot bdshot decoder: `libraries/AP_HAL_ChibiOS/RCOutput_bdshot.cpp`
- Protocol spec: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
- pico-bidir-dshot reference: https://github.com/bastian2001/pico-bidir-dshot
