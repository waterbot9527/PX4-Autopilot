/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_identity.cpp
 *
 * Raspberry Pi board identity implementation.
 *
 * Reads the unique CPU serial from /proc/cpuinfo and provides it
 * as the board UUID / PX4 GUID. This ensures each Raspberry Pi has
 * a unique identifier visible in MAVLink AUTOPILOT_VERSION.uid/uid2.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// board_get_uuid32, board_get_px4_guid and board_get_px4_guid_formated
// are declared in board_common.h (included via px4_config.h).
// BOARD_OVERRIDE_UUID is intentionally NOT defined, so the generic
// implementation in platforms/common/board_identity.c is skipped,
// and these definitions take its place.

__EXPORT void board_get_uuid32(uuid_uint32_t uuid_words)
{
	// Read the unique 64-bit CPU serial from /proc/cpuinfo.
	// Format: "Serial          : 10000000a1b2c3d4"
	uint32_t chip_uuid[PX4_CPU_UUID_WORD32_LENGTH];
	memset((uint8_t *)chip_uuid, 0, PX4_CPU_UUID_WORD32_LENGTH * 4);

	FILE *fp = fopen("/proc/cpuinfo", "r");
	if (fp) {
		char line[128];
		while (fgets(line, sizeof(line), fp)) {
			if (strncmp(line, "Serial", 6) == 0) {
				const char *val = strchr(line, ':');
				if (val) {
					val++;
					while (*val == ' ' || *val == '\t') val++;
					uint64_t serial = strtoull(val, NULL, 16);
					// Store as uint32_t words
					chip_uuid[2] = (uint32_t)(serial >> 32);
					chip_uuid[1] = (uint32_t)(serial & 0xFFFFFFFF);
					chip_uuid[3] = 0;
					chip_uuid[0] = 0;
				}
				break;
			}
		}
		fclose(fp);
	}

	for (unsigned int i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uuid_words[i] = chip_uuid[i];
	}
}

__EXPORT int board_get_px4_guid(px4_guid_t px4_guid)
{
	// Build the 18-byte PX4 GUID: <2b SOC_ARCH_ID><4b zeros><8b CPU serial><4b zeros>
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);

	uint8_t *pb = (uint8_t *)px4_guid;
	pb[0] = (uint8_t)((PX4_SOC_ARCH_ID >> 8) & 0xFF);
	pb[1] = (uint8_t)(PX4_SOC_ARCH_ID & 0xFF);
	memset(&pb[2], 0, 4);

	// Serial bytes in big-endian order from uuid[2] and uuid[1]
	uint8_t serial_bytes[8];
	serial_bytes[0] = (uint8_t)((uuid[2] >> 24) & 0xFF);
	serial_bytes[1] = (uint8_t)((uuid[2] >> 16) & 0xFF);
	serial_bytes[2] = (uint8_t)((uuid[2] >> 8) & 0xFF);
	serial_bytes[3] = (uint8_t)(uuid[2] & 0xFF);
	serial_bytes[4] = (uint8_t)((uuid[1] >> 24) & 0xFF);
	serial_bytes[5] = (uint8_t)((uuid[1] >> 16) & 0xFF);
	serial_bytes[6] = (uint8_t)((uuid[1] >> 8) & 0xFF);
	serial_bytes[7] = (uint8_t)(uuid[1] & 0xFF);

	memcpy(&pb[6], serial_bytes, sizeof(serial_bytes));
	memset(&pb[14], 0, PX4_GUID_BYTE_LENGTH - 14);

	return PX4_GUID_BYTE_LENGTH;
}

__EXPORT int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	int offset = 0;

	// size should be 2 per byte + 1 termination => odd
	size = (size & 1) ? size : size - 1;

	// Discard from MSD
	for (unsigned int i = PX4_GUID_BYTE_LENGTH - size / 2; offset < size && i < PX4_GUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", px4_guid[i]);
	}

	return offset;
}
