# Wishbone-Camera-Controller-for-OV5640
A Wishbone compliant Camera Controller for the OV5640 Camera written in VHDL.

The controller is designed to work with the NEORV32 RISC-V Processor. Link: https://github.com/stnolting/neorv32
The OV5640 Camera used: Waveshare OV5640 Version C. Link: https://www.waveshare.com/wiki/OV5640_Camera_Board_(C)

The Ada helper library and test File can also be found in this repository.

Requirements:
- NEORV32 Ada HAL. Link: https://github.com/GNAT-Academic-Program/neorv32-hal
- Input Output Library: https://github.com/dipenarathod/neorv32_npu_ecp5u5mg/tree/main/Ada%20Files/input_output_helper

Environment used for testing: ECP5U5MG-85F Evaluation Board, Lattice Diamond, and Synplify Pro
