# Bidirectional DShot Telemetry on RP2350 - Implementation Notes

## Current Status (January 2026)

**Status:** Working at ~99.5% decode rate at 0 RPM, ~40% at speed. PC-based state machine management.
**Tested with:** BlueJay ESCs on RP2350B (HELLBENDER_0001 config) at 8kHz PID loop
**Timing:** 48-cycle TX bit period, 7-cycle sample period (5.5x oversampling)

## What Works

- **5.5x oversampling with edge detection** - PIO samples 128 bits (7 cycles/sample), C code detects edges
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

### Edge Normalization is REQUIRED (Don't Remove It!)
The `wait 0 pin` instruction detects the falling edge, but the exact sample position
where that edge appears varies due to:
1. Sub-cycle timing - edge can occur at any point within a PIO clock cycle
2. ESC response timing variations
3. Sampling grid alignment relative to actual edge

**Why normalization matters:** The run-length decoder calculates bit lengths as:
```c
len = (diff * 2 + 5) / 11;  // round(diff / 5.5) using integer math
```
Without normalization, if first edge is at position 5 instead of 11:
- First run: 5 samples → rounds to 1 bit (should be 2 bits)
- All subsequent run lengths are off by 1 bit
- Result: ~15% error rate instead of 0.5%

**The fix:** Normalize all edge positions so first edge is at position 11:
```c
int16_t offset = 11 - (int16_t)edgePositions[0];
for (int i = 0; i < edgeCount; i++) {
    edgePositions[i] = (uint16_t)((int16_t)edgePositions[i] + offset);
}
```
This aligns the sampling grid to bit boundary 2 (11 ÷ 5.5 = 2 bits), ensuring consistent
decoding regardless of actual edge detection timing.

**DO NOT:** Try to "fix" this by starting run-length decode from first edge position
instead of position 0 - this causes 100% error rate because the algorithm expects
to decode leading bits before the first edge.

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
Instruction  14:    nop [7] - settling delay (~280ns) before switching to input
Instruction  15:    set pindirs, 0 (switch to input mode)
Instruction  16:    wait 1 pin 0 (wait for line to go HIGH - idle)
Instruction  17:    wait 0 pin 0 (wait for falling edge - start bit)
Instructions 18-23: 4x oversampling loop (4 outer × 32 inner = 128 samples)
Instructions 24-25: Switch back to output, drive HIGH
Instructions 26-31: Padding nops
```

**Transmit timing (48 cycles per bit, matches F405 reference):**
- '1' bit: nop [29] + set pins [13] + jmp = 32 cycles LOW, 16 cycles HIGH (67% duty)
- '0' bit: nop [13] + set pins [29] + jmp = 16 cycles LOW, 32 cycles HIGH (33% duty)
- At 150MHz / (48 cycles × clkdiv): ~600ns short pulse, ~1.1µs long pulse

**Receive timing (7 cycles per sample for 5.5x oversampling):**
- Sample loop: in pins (1) + nop [3] (4) + jmp (2) = 7 cycles per sample
- Telemetry bit = 38.4 PIO cycles (TX bit × 4/5)
- 7 cycles per sample gives 38.4/7 = 5.49x oversampling
- 21 bits × 5.5 = 115 samples needed, fits in 128 with margin
- Run-length decoding uses exact 5.5 rate: len = (diff × 2 + 5) / 11

Key points:
- `wait` instructions provide precise edge sync (unlike broken `jmp pin`)
- 5.5x oversampling is the maximum that fits in 128-sample buffer
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
   - Only send when SM at PC=0 or PC>=24 (safe states)
   - Track consecutive waits at PC=16-17 to detect stuck vs normal turnaround
   - Skip updates when SM mid-cycle to prevent duplicates
8. **48-Cycle Bit Period:** Changed from 40 to 48 cycles for finer timing control.
   - Matches F405 reference timing (~600ns/1.1µs)
   - Required adjusting receive sample loop from nop [4] to nop [6] (8→10 cycles)
   - Maintains 99.5% decode rate at 0 RPM
9. **Settling Delay Before Input Mode:** Added 8-cycle (~280ns) delay after TX before
   switching to input mode. Prevents ringing from affecting edge detection.
10. **5.5x Oversampling:** Changed from 10 to 7 cycles per sample for maximum oversampling.
    - 5.5x is the maximum that fits 21 bits in 128-sample buffer (6.4x would need 134 samples)
    - Uses exact 5.5 rate in decoder: len = (diff × 2 + 5) / 11
    - Moved `set x` before `wait 0 pin` to reduce first-sample delay to 2 cycles
    - Normalization target changed from 8 to 11 (bit 2 at 5.5x)

## Things That Don't Work (Don't Try These Again)

- **Removing edge normalization:** Causes ~15% error rate at 0 RPM (vs 0.5% with it)
- **Starting run-length decode from first edge instead of position 0:** 100% error rate
- **Using `jmp pin` for edge detection:** RP2350 erratum E9 makes it unreliable
- **6x oversampling (6 cycles/sample):** Actually gives 6.4x, requiring 134 samples for
  21 bits - exceeds 128-sample buffer, causing last edge to be cut off (100% error)

## Reference

- Betaflight STM32 decoder: `src/main/drivers/dshot_bitbang_decode.c`
- ArduPilot bdshot decoder: `libraries/AP_HAL_ChibiOS/RCOutput_bdshot.cpp`
- Protocol spec: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
- pico-bidir-dshot reference: https://github.com/bastian2001/pico-bidir-dshot
