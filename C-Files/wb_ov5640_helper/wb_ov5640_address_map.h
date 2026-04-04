/**
 * wb_ov5640_address_map.h
 *
 * Wishbone OV5640 camera controller — MMIO register address definitions.
 * C port of the Ada Wb_Ov5640_Address_Map package (wb_ov5640_address_map.ads),
 * targeting the NEORV32 RISC-V soft-core and its C software framework.
 *
 * Ada original: Wb_Ov5640_Address_Map (.ads)
 */

#ifndef WB_OV5640_ADDRESS_MAP_H
#define WB_OV5640_ADDRESS_MAP_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Register addresses
 * Ada: constant System.Address := System'To_Address(16#...#)
 * C  : plain uintptr_t constants cast to the target address.
 * ----------------------------------------------------------------------- */

/** Camera Control register. [0] = 1 from master → capture image.
 *  Ada: CTRL_Addr */
#define WB_OV5640_CTRL_ADDR                    ((uintptr_t)0x90010000U)

/** Camera status register. [0] = 1 = busy, [1] = 1 = done.
 *  Ada: STATUS_Addr */
#define WB_OV5640_STATUS_ADDR                  ((uintptr_t)0x90010004U)

/** Image Format register (not used).
 *  Ada: IMAGE_FORMAT_Addr */
#define WB_OV5640_IMAGE_FORMAT_ADDR            ((uintptr_t)0x90010008U)

/** Image Resolution register. [15:0] = image width, [31:16] = image height.
 *  Ada: IMAGE_RESOLUTION_Addr */
#define WB_OV5640_IMAGE_RESOLUTION_ADDR        ((uintptr_t)0x9001000CU)

/** 32-bit words the Wishbone master must read to gather the complete image
 *  (most likely not used in normal operation).
 *  Ada: MASTER_WORDS_TO_READ_Addr */
#define WB_OV5640_MASTER_WORDS_TO_READ_ADDR    ((uintptr_t)0x90010010U)

/** SCCB auto-programmer status register.
 *  [0] = start latched, [1] = program started, [2] = wrapper busy,
 *  [3] = done, [4] = error.
 *  Ada: SCCB_PROGRAM_STATUS_REG_Addr */
#define WB_OV5640_SCCB_PROGRAM_STATUS_REG_ADDR ((uintptr_t)0x90010014U)

/** OV5640 register address for manual SCCB write (written over Wishbone).
 *  Ada: SCCB_REG_ADDR_Addr */
#define WB_OV5640_SCCB_REG_ADDR_ADDR           ((uintptr_t)0x90010018U)

/** OV5640 register data for manual SCCB write (written over Wishbone).
 *  Ada: SCCB_REG_DATA_Addr */
#define WB_OV5640_SCCB_REG_DATA_ADDR           ((uintptr_t)0x9001001CU)

/** Manual SCCB programmer control register. [0] = start.
 *  Ada: SCCB_USER_CONTROL_Addr */
#define WB_OV5640_SCCB_USER_CONTROL_ADDR       ((uintptr_t)0x90010020U)

/** Manual SCCB programmer status register.
 *  [0] = busy, [1] = done, [2] = error.
 *  Ada: SCCB_USER_STATUS_Addr */
#define WB_OV5640_SCCB_USER_STATUS_ADDR        ((uintptr_t)0x90010024U)

/** Image Buffer base address.
 *  Ada: IMAGE_BUFFER_BASE_Addr */
#define WB_OV5640_IMAGE_BUFFER_BASE_ADDR       ((uintptr_t)0x90011000U)

#endif /* WB_OV5640_ADDRESS_MAP_H */
