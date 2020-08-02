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

#include <assert.h>
#include <inttypes.h>
#include <string.h>

#include "flash_map.h"

const struct flash_area *flash_map;
int flash_map_entries;

static int flash_area_find_idx(uint8_t id) {
  int i;

  if (flash_map == NULL) {
    return -1;
  }

  for (i = 0; i < flash_map_entries; i++) {
    if (flash_map[i].fa_id == id) {
      return i;
    }
  }

  return -1;
}

int flash_area_open(uint8_t id, const struct flash_area **fap) {
  int idx;

  if (flash_map == NULL) {
    return SYS_EACCES;
  }

  idx = flash_area_find_idx(id);
  if (idx == -1) {
    return SYS_ENOENT;
  }

  *fap = &flash_map[idx];

  return 0;
}


void    flash_area_close(const struct flash_area *fap);

/*< Reads `len` bytes of flash memory at `off` to the buffer at `dst` */
int     flash_area_read(const struct flash_area *fap, uint32_t off, void *dst,
                     uint32_t len)  { return -1; }

/*< Writes `len` bytes of flash memory at `off` from the buffer at `src` */
int     flash_area_write(const struct flash_area *fap, uint32_t off,
                     const void *src, uint32_t len)  { return -1; }

/*< Erases `len` bytes of flash memory at `off` */
int     flash_area_erase(const struct flash_area *fap, uint32_t off, uint32_t len)  { return -1; }

/*< Returns this `flash_area`s alignment */
uint8_t flash_area_align(const struct flash_area *fap)  { return -1; }

/*< What is value is read from erased flash bytes. */
uint8_t flash_area_erased_val(const struct flash_area *fap)  { return -1; }

/*< Reads len bytes from off, and checks if the read data is erased. Returns
    1 if empty (that is containing erased value), 0 if not-empty, and -1 on
    failure. */
int     flash_area_read_is_empty(const struct flash_area *fap, uint32_t off,
                     void *dst, uint32_t len)  { return -1; }

/*< Given flash area ID, return info about sectors within the area. */
int     flash_area_get_sectors(int fa_id, uint32_t *count,
                     struct flash_sector *sectors)  { return -1; }

/*< Returns the `fa_id` for slot, where slot is 0 (primary) or 1 (secondary).
    `image_index` (0 or 1) is the index of the image. Image index is
    relevant only when multi-image support support is enabled */
int     flash_area_id_from_multi_image_slot(int image_index, int slot) { return -1; }

/*< Returns the slot (0 for primary or 1 for secondary), for the supplied
    `image_index` and `area_id`. `area_id` is unique and is represented by
    `fa_id` in the `flash_area` struct. */
int     flash_area_id_to_multi_image_slot(int image_index, int area_id) { return -1; }

