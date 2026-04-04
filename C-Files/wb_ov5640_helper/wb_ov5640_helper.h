/**
 * wb_ov5640_helper.h
 *
 * Wishbone OV5640 camera controller — high-level register helpers.
 * C port of the Ada Wb_Ov5640_Helper package (wb_ov5640_helper.ads / .adb),
 * targeting the NEORV32 RISC-V soft-core and its C software framework.
 *
 * All short functions are declared static inline to honour the Ada
 * pragma Inline directives present in the original package spec.
 *
 * Ada original: Wb_Ov5640_Helper (.ads / .adb)
 */

#ifndef WB_OV5640_HELPER_H
#define WB_OV5640_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include "io_helper.h"            /* word_t, io_read_reg, io_write_reg, io_add_byte_offset */
#include "wb_ov5640_address_map.h"

/* -----------------------------------------------------------------------
 * Image resolution
 * ----------------------------------------------------------------------- */

/**
 * Set image resolution register.
 * Packs width into [15:0] and height into [31:16] of a single 32-bit word.
 *
 * Ada: procedure Set_Image_Resolution
 *        (Image_Width : in Natural; Image_Height : in Natural)
 */
static inline void wb_ov5640_set_image_resolution(unsigned int image_width,
                                                   unsigned int image_height)
{
    word_t resolution = (word_t)image_width;
    resolution |= ((word_t)image_height << 16U);
    io_write_reg(WB_OV5640_IMAGE_RESOLUTION_ADDR, resolution);
}

/* -----------------------------------------------------------------------
 * Image buffer word count
 * ----------------------------------------------------------------------- */

/**
 * Read the MASTER_WORDS_TO_READ register.
 * Returns the number of 32-bit words the master must read for a full image.
 *
 * Ada: function Get_Image_Buffer_Words_To_Read return Natural
 */
static inline unsigned int wb_ov5640_get_image_buffer_words_to_read(void)
{
    return (unsigned int)io_read_reg(WB_OV5640_MASTER_WORDS_TO_READ_ADDR);
}

/* -----------------------------------------------------------------------
 * Capture control
 * ----------------------------------------------------------------------- */

/**
 * Pulse control register bit [0] to order the camera to capture an image.
 * Waits until the camera signals busy before clearing the bit.
 *
 * Ada: procedure Start_Capturing_Image
 */
void wb_ov5640_start_capturing_image(void);

/* -----------------------------------------------------------------------
 * Status queries
 * ----------------------------------------------------------------------- */

/**
 * Return true if the camera is currently busy (STATUS[0] = 1).
 *
 * Ada: function Is_Camera_Busy return Boolean
 */
static inline bool wb_ov5640_is_camera_busy(void)
{
    return (io_read_reg(WB_OV5640_STATUS_ADDR) & 0x1U) != 0U;
}

/**
 * Return true if the camera has finished capturing (STATUS[1] = 1).
 *
 * Ada: function Is_Camera_Done return Boolean
 */
static inline bool wb_ov5640_is_camera_done(void)
{
    return (io_read_reg(WB_OV5640_STATUS_ADDR) & 0x2U) != 0U;
}

/* -----------------------------------------------------------------------
 * Blocking wait helpers
 * ----------------------------------------------------------------------- */

/**
 * Spin until the camera is no longer busy.
 *
 * Ada: procedure Wait_While_Camera_Busy
 */
static inline void wb_ov5640_wait_while_camera_busy(void)
{
    while (wb_ov5640_is_camera_busy()) {}
}

/**
 * Spin until the camera becomes busy (transitions idle → busy).
 *
 * Ada: procedure Wait_While_Camera_Becomes_Busy
 */
static inline void wb_ov5640_wait_while_camera_becomes_busy(void)
{
    while (!wb_ov5640_is_camera_busy()) {}
}

/**
 * Spin until the camera done bit (STATUS[1]) is set.
 *
 * Ada: procedure Wait_While_Camera_Done
 */
static inline void wb_ov5640_wait_while_camera_done(void)
{
    while ((io_read_reg(WB_OV5640_STATUS_ADDR) & 0x2U) == 0U) {}
}

/**
 * Spin until a VSYNC edge is detected (STATUS[7] = 1), indicating the
 * camera has started streaming. Prints an error and returns if the
 * timeout of 10 000 iterations is exceeded.
 *
 * Ada: procedure Wait_For_Camera_Streaming
 */
void wb_ov5640_wait_for_camera_streaming(void);

/* -----------------------------------------------------------------------
 * SCCB / I2C programming
 * ----------------------------------------------------------------------- */

/**
 * Return true if the SCCB auto-programmer has completed successfully.
 * The status register equals 15 (0b01111) when programming is correct;
 * it reads 7 or 31 otherwise.
 *
 * Ada: function Is_Camera_Programmed return Boolean
 */
static inline bool wb_ov5640_is_camera_programmed(void)
{
    return io_read_reg(WB_OV5640_SCCB_PROGRAM_STATUS_REG_ADDR) == 15U;
}

/**
 * Write a value to an OV5640 register via the manual SCCB (I2C) interface.
 * The function signature intentionally mirrors the OV5640 application
 * datasheet, as noted in the original Ada source.
 * Has no effect if the camera has not been successfully programmed.
 *
 * Ada: procedure write_i2c
 *        (OV5640_reg_addres : Word; OV5640_reg_data : Word)
 */
void wb_ov5640_write_i2c(word_t ov5640_reg_address, word_t ov5640_reg_data);

/* -----------------------------------------------------------------------
 * Image buffer access
 * ----------------------------------------------------------------------- */

/**
 * Read a single 32-bit word from the image buffer at word index 'index'.
 * The byte address is IMAGE_BUFFER_BASE + index * 4.
 *
 * Ada: function Read_Word_From_Image_Buffer (Index : Natural) return Word
 */
static inline word_t wb_ov5640_read_word_from_image_buffer(unsigned int index)
{
    uintptr_t addr = io_add_byte_offset(WB_OV5640_IMAGE_BUFFER_BASE_ADDR,
                                        (uint32_t)index * 4U);
    return io_read_reg(addr);
}

/**
 * Read 'count' consecutive 32-bit words from the image buffer into 'dest'.
 * The caller must ensure dest[] has at least 'count' elements.
 *
 * Ada: procedure Read_Words_From_Image_Buffer (Dest : out Word_Array)
 *      (Ada used the array's own range; C requires an explicit count.)
 */
void wb_ov5640_read_words_from_image_buffer(word_t *dest, unsigned int count);

#endif /* WB_OV5640_HELPER_H */
