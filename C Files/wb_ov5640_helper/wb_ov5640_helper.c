/**
 * wb_ov5640_helper.c
 *
 * Wishbone OV5640 camera controller — helper function implementations.
 * C port of the Ada Wb_Ov5640_Helper package body (wb_ov5640_helper.adb),
 * targeting the NEORV32 RISC-V soft-core and its C software framework.
 *
 * Functions that are short enough for inlining are defined as
 * static inline in wb_ov5640_helper.h.  Only the non-trivial bodies
 * that call neorv32_uart0_printf() or contain loops with side-effects
 * are placed here.
 *
 * Ada original: Wb_Ov5640_Helper (.adb)
 */

#include "wb_ov5640_helper.h"
#include "neorv32.h"   /* neorv32_uart0_printf */

/* -----------------------------------------------------------------------
 * Capture control
 * ----------------------------------------------------------------------- */

/*
 * Ada: procedure Start_Capturing_Image
 *
 * Pulse CTRL[0] high, wait for the camera to become busy (acknowledging
 * the request), then clear the bit.
 */
void wb_ov5640_start_capturing_image(void)
{
    io_write_reg(WB_OV5640_CTRL_ADDR, 1U);
    wb_ov5640_wait_while_camera_becomes_busy();
    io_write_reg(WB_OV5640_CTRL_ADDR, 0U);
}

/* -----------------------------------------------------------------------
 * Streaming readiness check
 * ----------------------------------------------------------------------- */

/*
 * Ada: procedure Wait_For_Camera_Streaming
 *
 * Poll STATUS[7] (VSYNC edge flag) until the camera starts streaming.
 * Abort with an error message after 10 000 iterations.
 */
void wb_ov5640_wait_for_camera_streaming(void)
{
    unsigned int timeout = 0U;

    while ((io_read_reg(WB_OV5640_STATUS_ADDR) & 0x80U) == 0U) {
        timeout++;
        if (timeout > 10000U) {
            neorv32_uart0_printf("ERROR: Camera not streaming after timeout\n");
            return;
        }
    }
}

/* -----------------------------------------------------------------------
 * SCCB / I2C manual write
 * ----------------------------------------------------------------------- */

/*
 * Ada: procedure write_i2c
 *        (OV5640_reg_addres : Word; OV5640_reg_data : Word)
 *
 * Write ov5640_reg_data to the OV5640 register at ov5640_reg_address via
 * the Wishbone SCCB bridge.  The function is a no-op if the SCCB
 * auto-programmer has not successfully completed (Is_Camera_Programmed = false).
 *
 * Sequence:
 *   1. Write the target OV5640 register address.
 *   2. Write the data value.
 *   3. Assert SCCB_USER_CONTROL[0] to start the manual transaction.
 *   4. Poll SCCB_USER_STATUS[0] until the busy flag clears.
 *   5. De-assert SCCB_USER_CONTROL[0].
 */
void wb_ov5640_write_i2c(word_t ov5640_reg_address, word_t ov5640_reg_data)
{
    if (!wb_ov5640_is_camera_programmed()) {
        return;
    }

    io_write_reg(WB_OV5640_SCCB_REG_ADDR_ADDR,     ov5640_reg_address);
    io_write_reg(WB_OV5640_SCCB_REG_DATA_ADDR,     ov5640_reg_data);
    io_write_reg(WB_OV5640_SCCB_USER_CONTROL_ADDR, 1U); /* start transaction */

    /* Wait until the SCCB bridge clears the busy flag */
    while ((io_read_reg(WB_OV5640_SCCB_USER_STATUS_ADDR) & 0x1U) != 0U) {}

    io_write_reg(WB_OV5640_SCCB_USER_CONTROL_ADDR, 0U); /* de-assert start */
}

/* -----------------------------------------------------------------------
 * Image buffer bulk read
 * ----------------------------------------------------------------------- */

/*
 * Ada: procedure Read_Words_From_Image_Buffer (Dest : out Word_Array)
 *
 * In Ada the loop range was derived from the array's own bounds;
 * in C the caller passes the element count explicitly.
 */
void wb_ov5640_read_words_from_image_buffer(word_t *dest, unsigned int count)
{
    for (unsigned int i = 0U; i < count; i++) {
        dest[i] = wb_ov5640_read_word_from_image_buffer(i);
    }
}
