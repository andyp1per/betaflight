# Bidirectional DShot Telemetry on RP2350 - Implementation Notes

## Current Status (January 2026)

**Status:** Working at ~98% decode rate with 4x oversampling edge detection.
**Tested with:** BlueJay ESCs on RP2350B (HELLBENDER_0001 config)

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
rest all 1s" corruption. **Solution:** Drain RX FIFO in transmit loop before `pio_sm_put()`.

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

- **~2% error rate from edge jitter** - Some edge timing variations cause decode errors.
  Could potentially improve with more sophisticated edge detection or voting.
- **RPM wrapping at high speed** - Reported values wrap past ~2000 RPM. May be
  eRPM to RPM conversion issue or EDT (Extended Telemetry) not being handled.
- **Configurator shows ~10% error** - Higher than debug output suggests. May be
  related to debug output overhead or telemetry stats calculation.

## Implementation History

1. **Fixed-Interval (3x Oversampling):** Initial attempt. High error rate.
2. **Fixed-Interval (Synchronized):** Fragile to ESC timing variations.
3. **PIO Edge Detection (jmp pin):** Didn't work - RP2350 erratum E9.
4. **Software Edge Detection Loop:** Worked but timing jitter issues.
5. **`wait` Instructions + Edge Normalization:** Current approach. 98% success rate.
6. **FIFO Drain Fix:** Resolved M2 channel failures (was 0%, now 98%).

## Reference

- Betaflight STM32 decoder: `src/main/drivers/dshot_bitbang_decode.c`
- ArduPilot bdshot decoder: `libraries/AP_HAL_ChibiOS/RCOutput_bdshot.cpp`
- Protocol spec: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
- pico-bidir-dshot reference: https://github.com/bastian2001/pico-bidir-dshot
