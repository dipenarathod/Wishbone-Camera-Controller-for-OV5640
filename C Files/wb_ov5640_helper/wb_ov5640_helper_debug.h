/**
 * wb_ov5640_helper_debug.h
 *
 * Wishbone OV5640 camera controller — debug register print utilities.
 * C port of the Ada child package Wb_Ov5640_Helper.Debug
 * (wb_ov5640_helper-debug.ads / -debug.adb), targeting the NEORV32
 * RISC-V soft-core and its C software framework.
 *
 * NOTE: Uses neorv32_uart0_printf() from the NEORV32 C SDK.
 *
 * Ada original: Wb_Ov5640_Helper.Debug (.ads / .adb)
 */

#ifndef WB_OV5640_HELPER_DEBUG_H
#define WB_OV5640_HELPER_DEBUG_H

/**
 * Print a human-readable dump of the five main camera controller registers:
 *   CTRL, STATUS, IMAGE_RESOLUTION, SCCB_PROGRAM_STATUS, SCCB_USER_STATUS.
 *
 * Ada: procedure Print_Registers
 */
void wb_ov5640_debug_print_registers(void);

/**
 * Print a detailed, field-decoded view of the STATUS register alongside
 * the CTRL register value.  Individual flag bits are printed as booleans.
 *
 * Fields decoded:
 *   FSM state    — STATUS[4:2]
 *   Saw capture  — STATUS[5]
 *   Saw VSYNC    — STATUS[6]
 *   VSYNC edge   — STATUS[7]
 *   HREF edge    — STATUS[8]
 *   PCLK edge    — STATUS[9]
 *
 * Ada: procedure Print_Status_Register_Detail
 */
void wb_ov5640_debug_print_status_register_detail(void);

#endif /* WB_OV5640_HELPER_DEBUG_H */
