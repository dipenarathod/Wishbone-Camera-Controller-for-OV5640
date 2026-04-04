/**
 * wb_ov5640_tests.c
 *
 * Wishbone OV5640 camera controller — integration test / bring-up program.
 * C port of the Ada Wb_Ov5640_Tests procedure (wb_ov5640_tests.adb),
 * targeting the NEORV32 RISC-V soft-core and its C software framework.
 *
 * Ada original: Wb_Ov5640_Tests (.adb)
 *
 * Structural notes
 * ----------------
 * Ada allowed nested procedures (Test_Camera_SCCB_Status,
 * Test_Image_Capture_And_Read, Print_Result) that directly accessed the
 * enclosing procedure's local variables.  Standard C has no nested functions,
 * so the shared variables are declared as file-scope statics and the helpers
 * become static functions — identical behaviour, zero overhead.
 *
 * Ada: Tensor_Words(100, False)
 * C  : io_tensor_words(100, false) = (100*100 + 3) / 4 = 2500 words.
 * Computed once at compile time via IMAGE_WORDS to avoid a VLA on the stack,
 * which is inadvisable on embedded targets.
 */

#include <stdint.h>
#include <stdbool.h>

#include "neorv32.h"                  /* neorv32_uart0_setup, neorv32_uart0_printf */
#include "io_helper.h"                /* word_t, io_tensor_words                   */
#include "io_helper_debug.h"          /* io_print_tensor_q07                       */
#include "io_helper_time.h"           /* io_read_cycle, io_print_time              */
#include "wb_ov5640_helper.h"         /* all camera helper functions               */
#include "wb_ov5640_helper_debug.h"   /* wb_ov5640_debug_print_*                   */

/* -----------------------------------------------------------------------
 * Compile-time image sizing
 *
 * Ada: Image_Words : Natural := Tensor_Words (100, False);
 *      → (100 * 100 + 3) / 4 = 2500 words for a 100×100 pixel image.
 * ----------------------------------------------------------------------- */
#define IMAGE_DIM   100U
#define IMAGE_WORDS ((IMAGE_DIM * IMAGE_DIM + 3U) / 4U)   /* = 2500 */

/* -----------------------------------------------------------------------
 * File-scope state shared between the helper functions below.
 * Ada: local variables of Wb_Ov5640_Tests captured by nested procedures.
 * ----------------------------------------------------------------------- */
static word_t    captured_image[IMAGE_WORDS];
static uint64_t  start_cycles;
static uint64_t  end_cycles;
/* delta_cycles is declared for symmetry with the Ada source; the line that
 * assigned it was commented out in the original. */
static uint64_t  delta_cycles; /* unused — kept for source fidelity */

/* -----------------------------------------------------------------------
 * print_result — local helper
 *
 * Ada: procedure Print_Result (Name : String; Passed : Boolean)
 * ----------------------------------------------------------------------- */
static void print_result(const char *name, bool passed)
{
    if (passed) {
        neorv32_uart0_printf("%s PASS\n", name);
    } else {
        neorv32_uart0_printf("%s FAIL\n", name);
    }
}

/* -----------------------------------------------------------------------
 * test_camera_sccb_status — Test 1
 * Verify the SCCB auto-programmer completed successfully.
 *
 * Ada: procedure Test_Camera_SCCB_Status
 * ----------------------------------------------------------------------- */
static void test_camera_sccb_status(void)
{
    /* Commented-out diagnostic loop from the Ada original:
     * for (unsigned int count = 0; count < 10; count++) {
     *     neorv32_uart0_printf("%u\n",
     *         io_read_reg(WB_OV5640_SCCB_PROGRAM_STATUS_REG_ADDR));
     * }
     */
    bool result = wb_ov5640_is_camera_programmed();
    print_result("Camera Programmed Test: ", result);
}

/* -----------------------------------------------------------------------
 * test_image_capture_and_read
 * Trigger a capture, time it, then read the image buffer and time that.
 *
 * Ada: procedure Test_Image_Capture_And_Read
 * ----------------------------------------------------------------------- */
static void test_image_capture_and_read(void)
{
    /* Trigger capture and time it */
    start_cycles = io_read_cycle();
    wb_ov5640_start_capturing_image();
    wb_ov5640_wait_while_camera_busy();
    end_cycles = io_read_cycle();

    io_print_time("Time taken to capture image: ", end_cycles - start_cycles);

    /* Read the image buffer and time that */
    start_cycles = io_read_cycle();
    wb_ov5640_read_words_from_image_buffer(captured_image, IMAGE_WORDS);
    end_cycles = io_read_cycle();

    /* delta_cycles = end_cycles - start_cycles; -- commented out in original */
    io_print_time("Time taken to read image: ", end_cycles - start_cycles);

    /* io_print_tensor_q07("Image", captured_image, IMAGE_WORDS); -- commented out */
    neorv32_uart0_printf("Image captured\n");
}

/* -----------------------------------------------------------------------
 * main — program entry point
 *
 * Ada: begin ... end Wb_Ov5640_Tests;
 *
 * NOTE: neorv32_uart0_setup() replaces the Ada Uart0.Init(19200) call.
 * Adjust the second argument (config) to match your NEORV32 SDK version
 * and UART configuration (parity, flow control flags).
 * ----------------------------------------------------------------------- */
int main(void)
{
    /* Ada: Uart0.Init (19200) */
    neorv32_uart0_setup(19200, 0);

    neorv32_uart0_printf("Reunning Test Cases----------------\n"); /* sic — matches Ada */

    /* After programming registers, wait for ISP to stabilise (~1 second).
     * Ada: for I in 1..1000 loop  for J in 1..1_000_000 loop  null; */
    for (unsigned int i = 0U; i < 1000U; i++) {
        for (unsigned int j = 0U; j < 100U; j++) {
            __asm__ volatile ("nop"); /* prevent optimiser from removing the loop */
        }
    }

    /* Ada: --Write_Reg (CTRL_Addr, Word (0));  (commented out) */

    test_camera_sccb_status();
    wb_ov5640_set_image_resolution(100, 100);
    wb_ov5640_debug_print_status_register_detail();

    /* Ada: --Test_Capture_Status_Only;  (commented out) */

    wb_ov5640_wait_for_camera_streaming();

    /* ---------------------------------------------------------------
     * OV5640 register initialisation sequence via SCCB (I2C).
     * Ada: write_i2c (Word (16#XXXX#), Word (16#YY#))
     * C  : wb_ov5640_write_i2c (0xXXXXU, 0xYYU)
     * --------------------------------------------------------------- */

    /* System clock source select */
    wb_ov5640_write_i2c(0x3103U, 0x11U);

    /* Software reset, then enter standby */
    wb_ov5640_write_i2c(0x3008U, 0x82U);
    wb_ov5640_write_i2c(0x3008U, 0x42U);

    /* Output drive strength */
    wb_ov5640_write_i2c(0x3017U, 0xFFU);
    wb_ov5640_write_i2c(0x3018U, 0xFFU);

    /* PLL configuration */
    wb_ov5640_write_i2c(0x3103U, 0x03U);
    wb_ov5640_write_i2c(0x3034U, 0x1AU);
    wb_ov5640_write_i2c(0x3035U, 0x21U);
    wb_ov5640_write_i2c(0x3036U, 0x46U);
    wb_ov5640_write_i2c(0x3037U, 0x13U);
    wb_ov5640_write_i2c(0x3108U, 0x01U);
    wb_ov5640_write_i2c(0x3824U, 0x02U);
    wb_ov5640_write_i2c(0x460CU, 0x22U);
    wb_ov5640_write_i2c(0x4837U, 0x22U);

    /* Format: RGB565, ISP input format */
    wb_ov5640_write_i2c(0x4300U, 0x30U);
    wb_ov5640_write_i2c(0x501FU, 0x00U);

    /* VSYNC polarity */
    wb_ov5640_write_i2c(0x4740U, 0x20U);

    /* AEC / AGC control */
    wb_ov5640_write_i2c(0x3503U, 0x00U);
    wb_ov5640_write_i2c(0x3A00U, 0x38U);

    /* ISP control */
    wb_ov5640_write_i2c(0x5001U, 0xFFU); /* --a3 (comment from Ada) */
    wb_ov5640_write_i2c(0x5003U, 0x08U);

    /* Brightness settings */
    wb_ov5640_write_i2c(0x5587U, 0x20U);
    wb_ov5640_write_i2c(0x5580U, 0x04U);
    wb_ov5640_write_i2c(0x5588U, 0x01U);

    /* AEC stable range */
    wb_ov5640_write_i2c(0x3A0FU, 0x50U);
    wb_ov5640_write_i2c(0x3A10U, 0x48U);
    wb_ov5640_write_i2c(0x3A1BU, 0x50U);
    wb_ov5640_write_i2c(0x3A1EU, 0x48U);
    wb_ov5640_write_i2c(0x3A11U, 0x90U);
    wb_ov5640_write_i2c(0x3A1FU, 0x21U);

    /* Timing: X/Y start */
    wb_ov5640_write_i2c(0x3800U, 0x00U);
    wb_ov5640_write_i2c(0x3801U, 0x08U);
    wb_ov5640_write_i2c(0x3802U, 0x00U);
    wb_ov5640_write_i2c(0x3803U, 0x02U);

    /* Timing: X/Y end */
    wb_ov5640_write_i2c(0x3804U, 0x0AU);
    wb_ov5640_write_i2c(0x3805U, 0x37U);
    wb_ov5640_write_i2c(0x3806U, 0x07U);
    wb_ov5640_write_i2c(0x3807U, 0xA1U);

    /* Output size: 100×100 (0x64 = 100; comment notes 0x32 = 50 for 50×50) */
    wb_ov5640_write_i2c(0x3808U, 0x00U);
    wb_ov5640_write_i2c(0x3809U, 0x64U); /* --32 for 50x50 */
    wb_ov5640_write_i2c(0x380AU, 0x00U);
    wb_ov5640_write_i2c(0x380BU, 0x64U); /* --32 for 50x50 */

    /* Total horizontal / vertical timing */
    wb_ov5640_write_i2c(0x380CU, 0x06U);
    wb_ov5640_write_i2c(0x380DU, 0x14U);
    wb_ov5640_write_i2c(0x380EU, 0x03U);
    wb_ov5640_write_i2c(0x380FU, 0xD8U);

    /* ISP X/Y offset */
    wb_ov5640_write_i2c(0x3810U, 0x00U);
    wb_ov5640_write_i2c(0x3811U, 0x04U);
    wb_ov5640_write_i2c(0x3812U, 0x00U);
    wb_ov5640_write_i2c(0x3813U, 0x02U);

    /* X/Y subsample increment */
    wb_ov5640_write_i2c(0x3814U, 0x31U);
    wb_ov5640_write_i2c(0x3815U, 0x31U);

    /* Flip / mirror / test pattern / MIPI */
    wb_ov5640_write_i2c(0x3820U, 0x47U);
    wb_ov5640_write_i2c(0x3821U, 0x01U);
    wb_ov5640_write_i2c(0x503DU, 0x00U);
    wb_ov5640_write_i2c(0x300EU, 0x58U);

    /* Exit standby — begin streaming */
    wb_ov5640_write_i2c(0x3008U, 0x02U);

    /* ---------------------------------------------------------------
     * Post-initialisation: dump registers, wait for streaming,
     * then capture and print two frames.
     * --------------------------------------------------------------- */
    wb_ov5640_debug_print_registers();
    wb_ov5640_wait_for_camera_streaming();

    /* Ada: --Set_Image_Resolution (50, 50);  (commented out) */

    test_image_capture_and_read();
    test_image_capture_and_read();
    io_print_tensor_q07("Image", captured_image, IMAGE_DIM);

    /* Commented-out multi-capture block from Ada original:
     * wb_ov5640_wait_for_camera_streaming();
     * wb_ov5640_set_image_resolution(50, 50);
     * test_image_capture_and_read();
     * test_image_capture_and_read();
     * test_image_capture_and_read();
     * io_print_tensor_q07("Image", captured_image, ???);
     */

    /* Ada: loop null; end loop; — spin forever */
    for (;;) {}

    return 0; /* unreachable — satisfies the C compiler */
}
