/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
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

#include "platform.h"

#include "common/maths.h"

#include "flight/car.h"

#include "rx/rx.h"

float carMapThrottleToMotorOutput(uint16_t throttleInput, uint16_t inputCenter, uint16_t inputDeadband,
    uint16_t outputLow, uint16_t outputNeutral, uint16_t outputHigh, bool reversed)
{
    const uint16_t constrainedInput = constrain(throttleInput, PWM_RANGE_MIN, PWM_RANGE_MAX);
    const uint16_t constrainedCenter = constrain(inputCenter, PWM_RANGE_MIN, PWM_RANGE_MAX);
    const uint16_t deadbandLow = constrainedCenter - MIN(constrainedCenter - PWM_RANGE_MIN, inputDeadband);
    const uint16_t deadbandHigh = constrainedCenter + MIN(PWM_RANGE_MAX - constrainedCenter, inputDeadband);

    if (constrainedInput < deadbandLow) {
        const float magnitude = (float)(deadbandLow - constrainedInput) / MAX(1, deadbandLow - PWM_RANGE_MIN);
        const uint16_t target = reversed ? outputHigh : outputLow;
        return outputNeutral + ((float)target - outputNeutral) * magnitude;
    }

    if (constrainedInput > deadbandHigh) {
        const float magnitude = (float)(constrainedInput - deadbandHigh) / MAX(1, PWM_RANGE_MAX - deadbandHigh);
        const uint16_t target = reversed ? outputLow : outputHigh;
        return outputNeutral + ((float)target - outputNeutral) * magnitude;
    }

    return outputNeutral;
}

bool carUpdateDrivePermission(bool *permitted, bool rxValid, bool throttleNeutral)
{
    if (!rxValid) {
        *permitted = false;
    } else if (throttleNeutral) {
        *permitted = true;
    }

    return *permitted;
}
