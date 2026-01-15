/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>

typedef enum {
    CLI_MODE_OFF,
    CLI_MODE_ON,
    CLI_MODE_DEBUG
} cliMode_e;

extern cliMode_e cliMode;

void cliProcess(void);
struct mspPort_s;
void cliEnter(struct mspPort_s *mspPort, bool interactive);

#ifdef USE_CLI_DEBUG_PRINT
void cliPrint(const char *str);
void cliPrintLinefeed(void);
void cliPrintLine(const char *str);
void cliPrintf(const char *format, ...);
void cliPrintLinef(const char *format, ...);
void cliWriterFlush(void);
#endif
