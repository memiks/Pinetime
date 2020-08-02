/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef H_UTIL_FLASH_MAP_
#define H_UTIL_FLASH_MAP_

#ifdef __cplusplus
extern "C" {
#endif

/**
 *
 * Provides abstraction of flash regions for type of use.
 * I.e. dude where's my image?
 *
 * System will contain a map which contains flash areas. Every
 * region will contain flash identifier, offset within flash and length.
 *
 * 1. This system map could be in a file within filesystem (Initializer
 * must know/figure out where the filesystem is at).
 * 2. Map could be at fixed location for project (compiled to code)
 * 3. Map could be at specific place in flash (put in place at mfg time).
 *
 * Note that the map you use must be valid for BSP it's for,
 * match the linker scripts when platform executes from flash,
 * and match the target offset specified in download script.
 */
#include <stdbool.h>
#include <inttypes.h>

#include "syscfg/syscfg.h"


#define SYS_EOK (0)
#define SYS_ENOMEM (-1)
#define SYS_EINVAL (-2)
#define SYS_ETIMEOUT (-3)
#define SYS_ENOENT (-4)
#define SYS_EIO (-5)
#define SYS_EAGAIN (-6)
#define SYS_EACCES (-7)
#define SYS_EBUSY (-8)
#define SYS_ENODEV (-9)
#define SYS_ERANGE (-10)
#define SYS_EALREADY (-11)
#define SYS_ENOTSUP (-12)
#define SYS_EUNKNOWN (-13)
#define SYS_EREMOTEIO (-14)
#define SYS_EDONE (-15)

#define SYS_EPERUSER (-65535)


#define FLASH_AREA_BOOTLOADER 0
#define FLASH_AREA_IMAGE_0 1
#define FLASH_AREA_IMAGE_1 2
#define FLASH_AREA_IMAGE_SCRATCH 3
#define FLASH_AREA_REBOOT_LOG 16
#define FLASH_AREA_NFFS 17

struct flash_area {
    uint8_t fa_id;
    uint8_t fa_device_id;
    uint16_t pad16;
    uint32_t fa_off;
    uint32_t fa_size;
};

struct flash_sector {
    struct flash_area fsr_flash_area;
    uint32_t fsr_range_start;
    uint16_t fsr_first_sector;
    uint16_t fsr_sector_count;
    uint32_t fsr_sector_size;
    uint8_t fsr_align;
};

extern const struct flash_area *flash_map;
extern int flash_map_entries;

/*< Opens the area for use. id is one of the `fa_id`s */
int     flash_area_open(uint8_t id, const struct flash_area **);
void    flash_area_close(const struct flash_area *);
/*< Reads `len` bytes of flash memory at `off` to the buffer at `dst` */
int     flash_area_read(const struct flash_area *, uint32_t off, void *dst,
                     uint32_t len);
/*< Writes `len` bytes of flash memory at `off` from the buffer at `src` */
int     flash_area_write(const struct flash_area *, uint32_t off,
                     const void *src, uint32_t len);
/*< Erases `len` bytes of flash memory at `off` */
int     flash_area_erase(const struct flash_area *, uint32_t off, uint32_t len);
/*< Returns this `flash_area`s alignment */
uint8_t flash_area_align(const struct flash_area *);
/*< What is value is read from erased flash bytes. */
uint8_t flash_area_erased_val(const struct flash_area *);
/*< Reads len bytes from off, and checks if the read data is erased. Returns
    1 if empty (that is containing erased value), 0 if not-empty, and -1 on
    failure. */
int     flash_area_read_is_empty(const struct flash_area *fa, uint32_t off,
                     void *dst, uint32_t len);
/*< Given flash area ID, return info about sectors within the area. */
int     flash_area_get_sectors(int fa_id, uint32_t *count,
                     struct flash_sector *sectors);
/*< Returns the `fa_id` for slot, where slot is 0 (primary) or 1 (secondary).
    `image_index` (0 or 1) is the index of the image. Image index is
    relevant only when multi-image support support is enabled */
int     flash_area_id_from_multi_image_slot(int image_index, int slot);
/*< Returns the slot (0 for primary or 1 for secondary), for the supplied
    `image_index` and `area_id`. `area_id` is unique and is represented by
    `fa_id` in the `flash_area` struct. */
int     flash_area_id_to_multi_image_slot(int image_index, int area_id);

#ifdef __cplusplus
}
#endif

#endif /* H_UTIL_FLASH_MAP_ */
