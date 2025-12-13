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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/utils.h"

#include "config/feature.h"

#include "sensors/acceleration_init.h"
#include "sensors/boardalignment.h"

#include "acceleration.h"

FAST_DATA_ZERO_INIT acc_t acc;                       // acc access functions
#ifdef USE_CRSF_ACCGYRO_TELEMETRY
static FAST_DATA_ZERO_INIT uint32_t downSampleCount;               // accel sensor sample counter for telemetry
static FAST_DATA_ZERO_INIT float downSampleSum[XYZ_AXIS_COUNT];   // summed samples used for downsampling for telemetry
#endif

static void applyAccelerationTrims(const flightDynamicsTrims_t *accelerationTrims)
{
    acc.accADC[X] -= accelerationTrims->raw[X];
    acc.accADC[Y] -= accelerationTrims->raw[Y];
    acc.accADC[Z] -= accelerationTrims->raw[Z];
}

void accUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!acc.dev.readFn(&acc.dev)) {
        return;
    }
    acc.isAccelUpdatedAtLeastOnce = true;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const int16_t val =  acc.dev.ADCRaw[axis];
        acc.accADC[axis] = val;
    }

    if (acc.dev.accAlign == ALIGN_CUSTOM) {
        alignSensorViaMatrix(acc.accADC, &acc.dev.rotationMatrix);
    } else {
        alignSensorViaRotation(acc.accADC, acc.dev.accAlign);
    }

    if (!accIsCalibrationComplete()) {
        performAcclerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    if (featureIsEnabled(FEATURE_INFLIGHT_ACC_CAL)) {
        performInflightAccelerationCalibration(&accelerometerConfigMutable()->accelerometerTrims);
    }

    applyAccelerationTrims(accelerationRuntime.accelerationTrims);

#ifdef USE_CRSF_ACCGYRO_TELEMETRY
    // using simple averaging for telemetry downsampling
    downSampleSum[X] += acc.accADC[X];
    downSampleSum[Y] += acc.accADC[Y];
    downSampleSum[Z] += acc.accADC[Z];
    downSampleCount++;
#endif

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const float val = acc.accADC[axis];
        acc.accADC[axis] = accelerationRuntime.accLpfCutHz ? pt2FilterApply(&accelerationRuntime.accFilter[axis], val) : val;
    }
}

#ifdef USE_CRSF_ACCGYRO_TELEMETRY
float accelGetDownsampled(int axis)
{
    return downSampleSum[axis] / downSampleCount;
}

void accelStartDownsampledCycle(void)
{
    downSampleCount = 1;
    downSampleSum[X] = acc.accADC[X];
    downSampleSum[Y] = acc.accADC[Y];
    downSampleSum[Z] = acc.accADC[Z];
}
#endif

#endif
