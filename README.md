# Wishbone Camera Controller for OV5640

A Wishbone-compliant OV5640 camera controller written in VHDL.

For example purposes, this project is designed to work with the [NEORV32 RISC-V Processor](https://github.com/stnolting/neorv32), and the Ada and C libraries are written for the NEORV32. However, this peripheral can be used with any RISCV core with a Wishbone interface. 

The camera used: [Waveshare OV5640 Camera Board (C)](https://www.waveshare.com/wiki/OV5640_Camera_Board_(C)).

## Features
- Supports grayscale image capture from the YUV422 image format
- Can capture 10KB of image data (100x100 grayscale image) - Can be increased/decreased in wb_ov5640_image_buffer.vhd by increasing/decreasing the number of words
- Built-in auto-programmer to program the camera on bootup
- Supports manual programming via code to use custom image capture settings
- < 100ms to capture an image (10 FPS)
- Ada and C support

## Repository Layout
- **RTL** — VHDL source for the camera controller
- **FPGA-Setup** — FPGA project/setup files for supported FPGA boards
- **Ada-Files** — Ada helper library for controlling the camera interface and test files. The tests file folder will also print out an image to the terminal
- **C-Files**— C helper library for controlling the camera interface and test files. The tests file folder will also print out an image to the terminal
  
## Dependencies
- **[Specific NEORV32 Fork](https://github.com/GNAT-Academic-Program/neorv32-setups)** - The Ada HAL as on 28th March 2026 only works with this fork of the NEORV32. Please refer to Part 1 of the video guide for installation instructions
- **[NEORV32-HAL](https://github.com/GNAT-Academic-Program/neorv32-hal)** - Base library required to run any Ada Program on the NEORV32
- **[Input-Output Helper Library](https://github.com/dipenarathod/Input-Output-Helper-Library-for-NEORV32-Ada-Projects)** - Required by the Camera Ada library (an Ada and C version is depending on what you are using)

**The C library works with the March 7th Build of the NEORV32**

## Development Environment
- **Board:** Lattice ECP5U5MG-85F Evaluation Board
- **Tools:** Lattice Diamond, Synplify Pro

## Related Repositories
- **[Central Tutorial Repository](https://github.com/dipenarathod/NEORV32-NGTTDS-YT-Central-Repository)** - Central repository with links to all relevant websites, repositories, and video guides
- **[Wishbone NPU](https://github.com/dipenarathod/Wishbone-NPU)** - Wishbone Peripheral used to accelerate common ML workloads in hardware
- **[Wishbone Interconnect 1 Master 2 Slaves](https://github.com/dipenarathod/Wishbone-Interconnect-1-Master-2-Slaves)** - Wishbone Interconnect to connect 2 Wishbone Peripherals to a Master

## Video Guides
- **[Connecting the Wishbone Camera Controller to the NEORV32](https://www.youtube.com/playlist?list=PLTuulhiizN0K-HTymHKr1Nurv-iq_RdWK)** - Shows you how to connect the NEORV32 to a Wishbone Camera Controller for OV5640
- **[Video Playlist showing how to connect the NPU to the NEORV32 in Lattice Diamond](https://www.youtube.com/playlist?list=PLTuulhiizN0IWdHwq5sg6dwhZwYaWbUX5)** - Refer to Parts 1 and 3 of this playlist to learn how to create a Diamond Project using a TCL Script and how to increase the IMEM and DMEM sizes of the NEORV32

## Acknowledgments

- **[AdaCore](https://www.adacore.com/)** — industry sponsor; project mentor Oliver Henley
- **[NEORV32](https://github.com/stnolting/neorv32)** by Stephan Nolting — the RISC-V soft-core processor
- **[GNAT Academic Program](https://github.com/GNAT-Academic-Program/neorv32-setups)** — NEORV32 + Ada integration
- **[Digikey VHDL I2C Controller](https://forum.digikey.com/t/i2c-master-vhdl/12797)** - Digikey I2C controller used to create the SCCB Programmer Wrapper
- **Penn State University** — Capstone course, instructor/advisor Naseem Ibrahim
