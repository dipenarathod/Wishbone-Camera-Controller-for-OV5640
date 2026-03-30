/**
 * wb_ov5640_helper_debug.c
 *
 * Wishbone OV5640 camera controller — debug register print utilities.
 * C port of the Ada child package body Wb_Ov5640_Helper.Debug
 * (wb_ov5640_helper-debug.adb), targeting the NEORV32 RISC-V soft-core
 * and its C software framework.
 *
 * Ada original: Wb_Ov5640_Helper.Debug (.adb)
 */

#include "wb_ov5640_helper_debug.h"
#include "wb_ov5640_helper.h"        /* io_read_reg via wb_ov5640_helper.h */
#include "wb_ov5640_address_map.h"
#include "io_helper.h"               /* io_read_reg, word_t               */
#include "neorv32.h"                 /* neorv32_uart0_printf               */

/* -----------------------------------------------------------------------
 * Ada: procedure Print_Registers
 *
 * Print the five main controller registers in decimal.
 * Ada used Word'Image() which prepends a leading space before the value;
 * the %u format specifier gives equivalent output without the space.
 * ----------------------------------------------------------------------- */
void wb_ov5640_debug_print_registers(void)
{
    neorv32_uart0_printf("CTRL Reg: %u\n",
        io_read_reg(WB_OV5640_CTRL_ADDR));

    neorv32_uart0_printf("Status Reg: %u\n",
        io_read_reg(WB_OV5640_STATUS_ADDR));

    neorv32_uart0_printf("Image Resolution Register: %u\n",
        io_read_reg(WB_OV5640_IMAGE_RESOLUTION_ADDR));

    neorv32_uart0_printf("SCCB Programmer Register: %u\n",
        io_read_reg(WB_OV5640_SCCB_PROGRAM_STATUS_REG_ADDR));

    neorv32_uart0_printf("SCCB Manual Programmer Register: %u\n",
        io_read_reg(WB_OV5640_SCCB_USER_STATUS_ADDR));
}

/* -----------------------------------------------------------------------
 * Ada: procedure Print_Status_Register_Detail
 *
 * Decode STATUS register bit-fields and print each flag individually.
 *
 * Bit layout (from Ada source):
 *   [4:2] FSM state   (3-bit unsigned, extracted by >> 2 after masking 0x1C)
 *   [5]   Saw capture request
 *   [6]   Saw VSYNC high
 *   [7]   Saw VSYNC edge
 *   [8]   Saw HREF edge
 *   [9]   Saw PCLK edge
 *
 * Ada used Interfaces.Shift_Right (Unsigned_32 (S and 16#1C#), 2) for the
 * FSM state; in C a plain right-shift of the masked uint32_t is identical.
 * Ada Boolean'Image prints "TRUE"/"FALSE"; neorv32_uart0_printf has no %b,
 * so we use a ternary to print the same strings.
 * ----------------------------------------------------------------------- */
void wb_ov5640_debug_print_status_register_detail(void)
{
    word_t s = io_read_reg(WB_OV5640_STATUS_ADDR);

    unsigned int fsm_state    = (unsigned int)((s & 0x1CU) >> 2U); /* bits [4:2] */
    bool saw_capture          = (s & 0x20U)  != 0U;                /* bit  5     */
    bool saw_vsync_high       = (s & 0x40U)  != 0U;                /* bit  6     */
    bool saw_vsync_edge       = (s & 0x80U)  != 0U;                /* bit  7     */
    bool saw_href_edge        = (s & 0x100U) != 0U;                /* bit  8     */
    bool saw_pclk_edge        = (s & 0x200U) != 0U;                /* bit  9     */

    neorv32_uart0_printf("CTRL Reg:   %u\n",
        io_read_reg(WB_OV5640_CTRL_ADDR));
    neorv32_uart0_printf("Status Reg: %u\n", s);
    neorv32_uart0_printf("FSM state: %u\n",  fsm_state);
    neorv32_uart0_printf("Saw Capture Request: %s\n",
        saw_capture    ? "TRUE" : "FALSE");
    neorv32_uart0_printf("Saw VSYNC high: %s\n",
        saw_vsync_high ? "TRUE" : "FALSE");
    neorv32_uart0_printf("Saw VSYNC edge: %s\n",
        saw_vsync_edge ? "TRUE" : "FALSE");
    neorv32_uart0_printf("Saw HREF edge: %s\n",
        saw_href_edge  ? "TRUE" : "FALSE");
    neorv32_uart0_printf("Saw PCLK edge: %s\n",
        saw_pclk_edge  ? "TRUE" : "FALSE");
}
