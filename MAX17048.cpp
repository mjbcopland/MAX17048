/* MAX17048 Driver Library
 * Modified January 2016 by Michael Copland for the Arduino framework
 *
 * Copyright (c) 2013 Neil Thiessen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "MAX17048.h"
#include <Arduino.h>
#include <Wire.h>

const uint8_t MAX17048::RCOMP0 = 0x97, MAX17048::m_ADDR = 0x36;

MAX17048::MAX17048() {
  Wire.begin();
}

bool MAX17048::open() {
  // Probe for the MAX17048 using a Zero Length Transfer
  Wire.beginTransmission(m_ADDR);
  return (Wire.endTransmission() == 0);
}

void MAX17048::reset() {
  // Write the POR command
  write(REG_CMD, 0x5400);
}

void MAX17048::quickStart() {
  // Read the current 16-bit register value
  uint16_t value = read(REG_MODE);

  if (bitRead(value, 14) != true) {
    // Set the QuickStart bit
    bitSet(value, 14);

    // Write the value back out
    write(REG_MODE, value);
  }
}

bool MAX17048::sleepEnabled() {
  // Read the 16-bit register value
  uint16_t value = read(REG_MODE);

  // Return the status of the EnSleep bit
  return bitRead(value, 13);
}

void MAX17048::sleepEnabled(bool enabled) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_MODE);

  if (bitRead(value, 13) != enabled) {
    // Set or clear the EnSleep bit
    bitWrite(value, 13, enabled);

    // Write the value back out
    write(REG_MODE, value);
  }
}

bool MAX17048::hibernating() {
  // Read the 16-bit register value
  uint16_t value = read(REG_MODE);

  // Return the status of the HibStat bit
  return bitRead(value, 12);
}

float MAX17048::hibernateThreshold() {
  // Read the 16-bit register value
  uint16_t value = read(REG_HIBRT);

  // Extract the hibernate threshold
  return highByte(value) * 0.208;
}

void MAX17048::hibernateThreshold(float threshold) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_HIBRT);

  // Mask off the old value
  value &= 0x00FF;

  // Do a smart update
  if (threshold > 0.0) {
      if (threshold < 53.04)
          value |= (uint16_t)(threshold / 0.208) << 8;
      else
          value |= 0xFF00;
  }

  // Write the 16-bit register
  write(REG_HIBRT, value);
}

float MAX17048::activeThreshold() {
  // Read the 16-bit register value
  uint16_t value = read(REG_HIBRT);

  // Extract the active threshold
  return (value & 0x00FF) * 0.00125;
}

void MAX17048::activeThreshold(float threshold) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_HIBRT);

  // Mask off the old value
  value &= 0xFF00;

  // Do a smart update
  if (threshold > 0.0) {
      if (threshold < 0.31875)
          value |= (uint8_t)(threshold / 0.00125);
      else
          value |= 0x00FF;
  }

  // Write the 16-bit register
  write(REG_HIBRT, value);
}

uint16_t MAX17048::version() {
  // Return the 16-bit production version
  return read(REG_VERSION);
}

uint8_t MAX17048::compensation() {
  // Read the 16-bit register value
  uint16_t value = read(REG_CONFIG);

  // Return only the upper byte
  return highByte(value);
}

void MAX17048::compensation(uint8_t rcomp) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_CONFIG);

  // Update the register value
  value &= 0x00FF;
  value |= rcomp << 8;

  // Write the value back out
  write(REG_CONFIG, value);
}

void MAX17048::tempCompensation(float temp) {
  // Calculate the new RCOMP value
  uint8_t rcomp;
  if (temp > 20.0) {
      rcomp = RCOMP0 + (temp - 20.0) * -0.5;
  } else {
      rcomp = RCOMP0 + (temp - 20.0) * -5.0;
  }

  // Update the RCOMP value
  compensation(rcomp);
}

bool MAX17048::sleeping() {
  // Read the 16-bit register value
  uint16_t value = read(REG_CONFIG);

  // Return the status of the SLEEP bit
  return bitRead(value, 7);
}

void MAX17048::sleep(bool sleep) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_CONFIG);

  if (bitRead(value, 7) != sleep) {
    // Set or clear the SLEEP bit
    bitWrite(value, 7, sleep);

    // Write the value back out
    write(REG_CONFIG, value);
  }
}

bool MAX17048::socChangeAlertEnabled() {
  // Read the 16-bit register value
  uint16_t value = read(REG_CONFIG);

  // Return the status of the ALSC bit
  return bitRead(value, 6);
}

void MAX17048::socChangeAlertEnabled(bool enabled) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_CONFIG);

  if (bitRead(value, 6) != enabled) {
    // Set or clear the ALSC bit
    bitWrite(value, 6, enabled);
    
    // Write the value back out
    write(REG_CONFIG, value);
  }
}

bool MAX17048::alerting() {
  // Read the 16-bit register value
  uint16_t value = read(REG_CONFIG);

  // Return the status of the ALRT bit
  return bitRead(value, 5);
}

void MAX17048::clearAlert() {
  // Read the current 16-bit register value
  uint16_t value = read(REG_CONFIG);

  if (bitRead(value, 5) != false) {
    // Clear the ALRT bit
    bitClear(value, 5);

    // Write the value back out
    write(REG_CONFIG, value);
  }
}

uint8_t MAX17048::emptyAlertThreshold() {
  // Read the 16-bit register value
  uint16_t value = read(REG_CONFIG);

  // Extract the threshold
  return 32 - (value & 0x001F);
}

void MAX17048::emptyAlertThreshold(uint8_t threshold) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_CONFIG);

  // Range check threshold
  threshold = constrain(threshold, 1, 32);

  // Update the register value
  value &= 0xFFE0;
  value |= 32 - threshold;

  // Write the 16-bit register
  write(REG_CONFIG, value);
}

float MAX17048::vAlertMinThreshold() {
  // Read the 16-bit register value
  uint16_t value = read(REG_VALRT);

  // Extract the alert threshold
  return highByte(value) * 0.02;
}

void MAX17048::vAlertMinThreshold(float threshold) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_VALRT);

  // Mask off the old value
  value &= 0x00FF;

  // Do a smart update
  if (threshold > 0.0) {
      if (threshold < 5.1)
          value |= (uint16_t)(threshold / 0.02) << 8;
      else
          value |= 0xFF00;
  }

  // Write the 16-bit register
  write(REG_VALRT, value);
}

float MAX17048::vAlertMaxThreshold() {
  // Read the 16-bit register value
  uint16_t value = read(REG_VALRT);

  // Extract the active threshold
  return (value & 0x00FF) * 0.02;
}

void MAX17048::vAlertMaxThreshold(float threshold) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_VALRT);

  // Mask off the old value
  value &= 0xFF00;

  // Do a smart update
  if (threshold > 0.0) {
      if (threshold < 5.1)
          value |= (uint8_t)(threshold / 0.02);
      else
          value |= 0x00FF;
  }

  // Write the 16-bit register
  write(REG_VALRT, value);
}

float MAX17048::vResetThreshold() {
  // Read the 16-bit register value
  uint16_t value = read(REG_VRESET_ID);

  // Extract the threshold
  return (value >> 9) * 0.04;
}

void MAX17048::vResetThreshold(float threshold) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_VRESET_ID);

  // Mask off the old value
  value &= 0x01FF;

  // Do a smart update
  if (threshold > 0.0) {
      if (threshold < 5.08)
          value |= (uint16_t)(threshold / 0.04) << 9;
      else
          value |= 0xFE00;
  }

  // Write the 16-bit register
  write(REG_VRESET_ID, value);
}

bool MAX17048::comparatorEnabled() {
  // Read the 16-bit register value
  uint16_t value = read(REG_VRESET_ID);

  // Return the status of the Dis bit
  return bitRead(value, 8);
}

void MAX17048::comparatorEnabled(bool enabled) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_VRESET_ID);

  if (bitRead(value, 8) != enabled) {
    // Set or clear the Dis bit
    bitWrite(value, 8, enabled);

    // Write the value back out
    write(REG_VRESET_ID, value);
  }
}

uint8_t MAX17048::id() {
  // Read the 16-bit register value
  uint16_t value = read(REG_VRESET_ID);

  // Return only the ID bits
  return lowByte(value);
}

bool MAX17048::vResetAlertEnabled() {
  // Read the 16-bit register value
  uint16_t value = read(REG_STATUS);

  // Return the status of the EnVR bit
  return bitRead(value, 14);
}

void MAX17048::vResetAlertEnabled(bool enabled) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_STATUS);

  if (bitRead(value, 14) != enabled) {
    // Set or clear the EnVR bit
    bitWrite(value, 14, enabled);

    // Write the value back out
    write(REG_STATUS, value);
  }
}

uint8_t MAX17048::alertFlags() {
  // Read the 16-bit register value
  uint16_t value = read(REG_STATUS);

  // Return only the flag bits
  return highByte(value) & 0x3F;
}

void MAX17048::clearAlertFlags(uint8_t flags) {
  // Read the current 16-bit register value
  uint16_t value = read(REG_STATUS);

  // Clear the specified flag bits
  value &= ~((flags & 0x3F) << 8);

  // Write the value back out
  write(REG_STATUS, value);
}

float MAX17048::vcell() {
  // Read the 16-bit raw Vcell value
  uint16_t value = read(REG_VCELL);

  // Return Vcell in volts
  return value * 0.000078125;
}

float MAX17048::soc() {
  // Read the 16-bit raw SOC value
  uint16_t value = read(REG_SOC);

  // Return SOC in percent
  return value * 0.00390625;
}

uint8_t MAX17048::soc_int() {
  // Read the 16-bit raw SOC value
  uint16_t value = read(REG_SOC);

  // Return only the top byte
  return highByte(value);
}

float MAX17048::crate() {
  // Read the 16-bit raw C/Rate value
  uint16_t value = read(REG_CRATE);

  // Return C/Rate in %/hr
  return value * 0.208;
}

MAX17048::operator float() {
  // Return the current floating point SOC reading
  return soc();
}

MAX17048::operator uint8_t() {
  // Return the current integer SOC reading
  return soc_int();
}

uint16_t MAX17048::read(uint8_t reg) {
  // Read the 16-bit register
  Wire.requestFrom(m_ADDR, 2, reg, 1, true);

  // Return the combined 16-bit value
  return (Wire.read() << 8) | Wire.read();
}

void MAX17048::write(uint8_t reg, uint16_t data) {
  Wire.beginTransmission(m_ADDR);
  Wire.write(reg);
  Wire.write(highByte(data));
  Wire.write( lowByte(data));
  Wire.endTransmission();
}

// Initialise a MAX17048 object
MAX17048 Gauge;