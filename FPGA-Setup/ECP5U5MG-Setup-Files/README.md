# File Descriptions
- ECP5EVN_NEORV32_MinimalBoot.lpf: Constraint file. Use this file to understand how the camera is wired to the FPGA board
- neorv32_ECP5EVN_BoardTop_MinimalBoot.vhd: Project top-level file. Passes the FPGA physical pin connections to the NEORV32 and the wishbone peripheral
- neorv32_ProcessorTop_MinimalBoot.vhd: NEORV32 Minimal Boot template file modified to instantiate and use the camera controller (has the XBUS enabled)

Please refer to the video guide for connecting an NPU to the NEORV32 to understand how to create a Lattice Diamond Project (video guide linked in main README)
