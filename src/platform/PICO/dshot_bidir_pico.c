/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#if defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)

#include "dshot_pico.h"

// Maximum time to wait for telemetry reception to complete
#define DSHOT_TELEMETRY_TIMEOUT 2000

// TODO use with TELEMETRY for cli report (or not)
FAST_DATA_ZERO_INIT dshotTelemetryCycleCounters_t dshotDMAHandlerCycleCounters;

#define DSHOT_BIDIR_BIT_PERIOD 40

// Bidirectional DShot telemetry protocol:
// - FC sends inverted DShot frame (idle HIGH, 1 = LOW pulse, 0 = HIGH pulse)
// - ESC responds ~30us later with 21-bit GCR-encoded telemetry at 5/4x bitrate
// - Telemetry contains 12-bit eRPM data + 4-bit checksum
//
// This implementation uses edge detection (ported from pico-bidir-dshot by bastian2001)
// instead of fixed-delay 3x oversampling. Edge detection is self-synchronizing and
// more robust to ESC timing variations (works with BlueJay, AM32, BLHeli32, etc.)

bool dshot_program_bidir_init(PIO pio, uint sm, int offset, uint pin)
{
    bprintf("dshot_program_bidir_init on pin %d",pin);
#ifdef DSHOT_DEBUG_PIO
    pio_sm_config config = dshot_600_bidir_debug_program_get_default_config(offset);
#else
    pio_sm_config config = dshot_600_bidir_program_get_default_config(offset);
#endif

    sm_config_set_set_pins(&config, pin, 1);
    sm_config_set_in_pins(&config, pin);
    sm_config_set_jmp_pin(&config, pin);  // Required for edge detection receive (jmp pin instruction)
    pio_gpio_init(pio, pin);
    bprintf("dshot bidir on pin %d done init PIO->gpio for pio",pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true); // set pin to output

    gpio_set_pulls(pin, true, false); // Pull up - idle 1 when awaiting bidir telemetry input (PIO pindirs 0).
    sm_config_set_out_shift(&config, PIO_SHIFT_LEFT, PIO_NO_AUTO_PUSHPULL, 32);
    sm_config_set_in_shift(&config, PIO_SHIFT_LEFT, PIO_NO_AUTO_PUSHPULL, 32);  // Left shift: first sample at bit 0

    float clocks_per_us = clock_get_hz(clk_sys) / 1000000;
#ifdef TEST_DSHOT_SLOW
    sm_config_set_clkdiv(&config, (1.0e4f + dshotGetPeriodTiming()) / DSHOT_BIDIR_BIT_PERIOD * clocks_per_us);
#else
    sm_config_set_clkdiv(&config, dshotGetPeriodTiming() / DSHOT_BIDIR_BIT_PERIOD * clocks_per_us);
#endif

    return PICO_OK == pio_sm_init(pio, sm, offset, &config);
}

// GCR decode lookup table (maps 5-bit GCR to 4-bit nibble, -1 = invalid)
static const int8_t gcrDecodeLut[32] = {
    -1, -1, -1, -1, -1, -1, -1, -1,
    -1,  9, 10, 11, -1, 13, 14, 15,
    -1, -1,  2,  3, -1,  5,  6,  7,
    -1,  0,  8,  1, -1,  4, 12, -1
};

// Decode telemetry from edge detection PIO (single 32-bit word with 20 GCR bits)
// The edge detection PIO decodes differential encoding in hardware by measuring
// pulse widths: long LOW = 0 bit, long HIGH = 1 bit. This gives us GCR bits directly.
static uint32_t decodeTelemetryRaw(int ind, const uint32_t raw)
{
    UNUSED(ind);

    if (raw == 0) {
        // No response from ESC
        return DSHOT_TELEMETRY_INVALID;
    }

    // Edge detection PIO outputs 20 GCR bits directly (no XOR decode needed)
    // With LEFT shift, 20 bits are in bits 19:0
    uint32_t bits20 = raw & 0xFFFFF;

    // GCR decode: map 4 groups of 5 bits to 4 nibbles
    // Same extraction as STM32: LSB group first
    uint32_t data = gcrDecodeLut[bits20 & 0x1F];              // bits 4:0 -> nibble 0
    data |= gcrDecodeLut[(bits20 >> 5) & 0x1F] << 4;          // bits 9:5 -> nibble 1
    data |= gcrDecodeLut[(bits20 >> 10) & 0x1F] << 8;         // bits 14:10 -> nibble 2
    data |= gcrDecodeLut[(bits20 >> 15) & 0x1F] << 12;        // bits 19:15 -> nibble 3

    // Check for invalid GCR codes
    if (data > 0xFFFF) {
#ifdef PICO_TRACE
        static int badgcrs;
        if (badgcrs % 50000 < 4) {
            bprintf("\ndshot telem bad gcr [%d] raw=%08x bits20=%05x", badgcrs, raw, bits20);
        }
        badgcrs++;
#endif
        return DSHOT_TELEMETRY_INVALID;
    }

    // Verify checksum (XOR of all nibbles should be 0xF)
    uint32_t checksum = (data >> 12) ^ (data >> 8) ^ (data >> 4) ^ data;
    if ((checksum & 0xF) != 0xF) {
#ifdef PICO_TRACE
        bprintf("\ncheckSum = %x (should be 0xf), raw=%08x", checksum & 0xF, raw);
#endif
        return DSHOT_TELEMETRY_INVALID;
    }

    // Return 12-bit telemetry value (discard 4-bit checksum)
    return (data >> 4) & 0xFFF;
}

bool dshotTelemetryWait(void)
{
    bool telemetryWait = false;
#ifdef USE_DSHOT_TELEMETRY
    // Wait for telemetry reception to complete
    bool telemetryPending;
    const timeUs_t startTimeUs = micros();

    bprintf("dshotTelemetryWait");
    do {
        telemetryPending = false;
/*
  TODO TBC this could be something like
        for (unsigned motorIndex = 0; motorIndex < dshotMotorCount && !telemetryPending; motorIndex++) {
            const motorOutput_t *motor = &dshotMotors[motorIndex];

            // call a function that (taking into account which PIO program we are running), tells
            // us on the basis of
            //    uint8_t pio_pc = pio_sm_get_pc(motor->pio, motor->pio_sm) - motor->offset;
            // whether we have finished receiving a sequence of bidir telemetry bits and it's
            // therefore safe to send out a dshot command
            telemetryPending |= SafeToSend(motor);

            or, we can keep track of state based on whether we have called
               dshotwrite
               dshotdecodetelemetry successfully
               dshotdecodetelemetry unsuccessfully
            and, if necessary, keep calling dshotdecodetelemetry (up to a timeout) until telemetry received
        }

  I think telemetryPending tracks when we are safe to transmit
  and telemetryWait is for debugging, telling us if we had to wait
  (but no code looks at the return value of this function)

 */
        telemetryWait |= telemetryPending;

        if (cmpTimeUs(micros(), startTimeUs) > DSHOT_TELEMETRY_TIMEOUT) {
            break;
        }
    } while (telemetryPending);

    if (telemetryWait) {
        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 2, debug[2] + 1);
    }

    bprintf("dshotTelemetryWait returning %d",telemetryWait);
#endif
    return telemetryWait;
}

bool dshotDecodeTelemetry(void)
{
///////    bprintf("dshotDecodeTelemtry");
#ifndef USE_DSHOT_TELEMETRY
    return true;
#else

    if (!useDshotTelemetry) {
        return true;
    }

#ifdef USE_DSHOT_TELEMETRY_STATS
    const timeMs_t currentTimeMs = millis();
#endif

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < dshotMotorCount; motorIndex++) {
        const motorOutput_t *motor = &dshotMotors[motorIndex];

        int fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);
        if (fifo_words < 1) {
#ifdef PICO_TRACE
            static int notelem;
            if ((notelem % 250000) < 4) {
                bprintf("NO TELEM [%d] dshot motor %d, sm at pc-offset = %d, rx:%d, tx:%d",
                        notelem,
                        motorIndex,
                        pio_sm_get_pc(motor->pio, motor->pio_sm) - motor->offset,
                        pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm),
                        pio_sm_get_tx_fifo_level(motor->pio, motor->pio_sm)
                       );
            }
            notelem++;
#endif
            continue;
        }

        // Edge detection PIO returns a single word with 21 bits of telemetry
        if (fifo_words > 1) {
            // FIFO has more than one telemetry item - discard old ones, keep most recent
            bprintf("*** fifo_words: %d", fifo_words);
            while (fifo_words > 1) {
                (void)pio_sm_get(motor->pio, motor->pio_sm); // discard
                fifo_words = pio_sm_get_rx_fifo_level(motor->pio, motor->pio_sm);
            }
        }

        const uint32_t raw = pio_sm_get_blocking(motor->pio, motor->pio_sm);

        uint32_t rawValue = decodeTelemetryRaw(motorIndex, raw);

        DEBUG_SET(DEBUG_DSHOT_TELEMETRY_COUNTS, 0, debug[0] + 1);
        dshotTelemetryState.readCount++;

        if (rawValue != DSHOT_TELEMETRY_INVALID) {
            // Check EDT enable or store raw value
            if ((rawValue == 0x0E00) && (dshotCommandGetCurrent(motorIndex) == DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE)) {
                bprintf("\n** received dshot 0x0E00");
                dshotTelemetryState.motorState[motorIndex].telemetryTypes = 1 << DSHOT_TELEMETRY_TYPE_STATE_EVENTS;
            } else {
                dshotTelemetryState.motorState[motorIndex].rawValue = rawValue;
            }
        } else {
            dshotTelemetryState.invalidPacketCount++;
        }

#ifdef USE_DSHOT_TELEMETRY_STATS
        updateDshotTelemetryQuality(&dshotTelemetryQuality[motorIndex], rawValue != DSHOT_TELEMETRY_INVALID, currentTimeMs);
#endif
    }

    dshotTelemetryState.rawValueState = DSHOT_RAW_VALUE_STATE_NOT_PROCESSED;
    return true;
#endif
}

#endif // defined(USE_DSHOT) && defined(USE_DSHOT_TELEMETRY)
