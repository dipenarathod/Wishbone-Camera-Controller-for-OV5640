VHDL files for the Wishbone Peripheral:
- wb_ov5640.vhd is the main peripheral
- sccb_i2c_wrapper.vhd is a SCCB wrapper for Digikey's I2C implementation. It has the tables showing what OV5640 registers are programmed and with what values.
- ov5640_image_buffer.vhd defines the data type for the image buffer
- i2c_controller.vhd is DigiKey's i2C controller implementation with a few name changes and added comments 
