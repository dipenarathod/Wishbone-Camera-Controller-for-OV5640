In your Ada application program, you can use the Print_Tensor_Q07 function from input_output_helper-debug (https://github.com/dipenarathod/neorv32_npu_ecp5u5mg/tree/main/Ada%20Files/input_output_helper) to print the image buffer to the terminal over UART.

The printed tensor will use int8 Q0.7 numbers.

Paste the tensor from the UART output window into the img_data array in the Python script here to create an image using it.
