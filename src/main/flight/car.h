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

#pragma once

#include <stdbool.h>
#include <stdint.h>

float carMapThrottleToMotorOutput(uint16_t throttleInput, uint16_t inputCenter, uint16_t inputDeadband,
    uint16_t outputLow, uint16_t outputNeutral, uint16_t outputHigh, bool reversed);

// Drive is allowed once RX is valid and the throttle has been seen at neutral; revoked when RX is invalid.
bool carUpdateDrivePermission(bool *permitted, bool rxValid, bool throttleNeutral);
