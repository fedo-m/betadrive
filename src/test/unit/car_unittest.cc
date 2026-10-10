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

#include <gtest/gtest.h>

extern "C" {
#include "flight/car.h"
}

TEST(CarMixerTest, NormalDirectionUsesCenteredPwmRanges)
{
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1000, 1500, 50, 1000, 1500, 2000, false), 1000);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1450, 1500, 50, 1000, 1500, 2000, false), 1500);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1500, 1500, 50, 1000, 1500, 2000, false), 1500);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1550, 1500, 50, 1000, 1500, 2000, false), 1500);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(2000, 1500, 50, 1000, 1500, 2000, false), 2000);
}

TEST(CarMixerTest, ReversedDirectionMirrorsAroundNeutral)
{
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1000, 1500, 50, 1000, 1500, 2000, true), 2000);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1450, 1500, 50, 1000, 1500, 2000, true), 1500);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1500, 1500, 50, 1000, 1500, 2000, true), 1500);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1550, 1500, 50, 1000, 1500, 2000, true), 1500);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(2000, 1500, 50, 1000, 1500, 2000, true), 1000);
}

TEST(CarMixerTest, AsymmetricOutputsStillPreserveNeutral)
{
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1000, 1500, 25, 1100, 1520, 1900, false), 1100);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1500, 1500, 25, 1100, 1520, 1900, false), 1520);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(2000, 1500, 25, 1100, 1520, 1900, false), 1900);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(1000, 1500, 25, 1100, 1520, 1900, true), 1900);
    EXPECT_FLOAT_EQ(carMapThrottleToMotorOutput(2000, 1500, 25, 1100, 1520, 1900, true), 1100);
}

TEST(CarMixerTest, DrivePermissionRequiresNeutralAndIsRevokedOnRxLoss)
{
    bool permitted = false;
    EXPECT_FALSE(carUpdateDrivePermission(&permitted, true, false));
    EXPECT_TRUE(carUpdateDrivePermission(&permitted, true, true));
    EXPECT_TRUE(carUpdateDrivePermission(&permitted, true, false));
    EXPECT_FALSE(carUpdateDrivePermission(&permitted, false, true));
    EXPECT_FALSE(carUpdateDrivePermission(&permitted, true, false));
    EXPECT_TRUE(carUpdateDrivePermission(&permitted, true, true));
}
