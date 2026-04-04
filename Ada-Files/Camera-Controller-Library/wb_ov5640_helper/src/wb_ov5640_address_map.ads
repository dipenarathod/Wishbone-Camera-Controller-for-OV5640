with System;
package Wb_Ov5640_Address_Map is

   --Register addresses

   CTRL_Addr                    : constant System.Address :=
     System'To_Address
       (16#90010000#); --Camera Control register. [0] = 1 from master, capture image
   STATUS_Addr                  : constant System.Address :=
     System'To_Address
       (16#90010004#); --Camera status register. [0] = 1 = busy, [1] = 1 = done
   IMAGE_FORMAT_Addr            : constant System.Address :=
     System'To_Address (16#90010008#); --Image Format address (not used)
   IMAGE_RESOLUTION_Addr        : constant System.Address :=
     System'To_Address
       (16#9001000C#); --[15:0] = Image width [31:16] = Image height
   MASTER_WORDS_TO_READ_Addr    : constant System.Address :=
     System'To_Address
       (16#90010010#); --32-bit words the master has to read to gather the complete image (most likely not used)
   SCCB_PROGRAM_STATUS_REG_Addr : constant System.Address :=
     System'To_Address
       (16#90010014#); --Register to show SCCB programmer status. [0] = start latched.
   --[1] = program started. [2] = wrapper busy. [3] = done. [4] = error
   SCCB_REG_ADDR_Addr : constant System.Address :=
     System'To_Address
       (16#90010018#); --Register address for OV5640 register to be programmed (writtent to over wishbone from client application)
   SCCB_REG_DATA_Addr : constant System.Address :=
     System'To_Address
       (16#9001001C#); --Data for OV5640 register to be programmed (writtent to over wishbone from client application)
   SCCB_USER_CONTROL_Addr : constant System.Address :=
     System'To_Address
       (16#90010020#); --Manual SCCB programmer control register. [0] for start
   SCCB_USER_STATUS_Addr : constant System.Address :=
     System'To_Address
       (16#90010024#); --Manual SCCB programmer status register. [0] = busy. [1] = done. [2] = error
    
   
   IMAGE_BUFFER_BASE_Addr       : constant System.Address :=
     System'To_Address (16#90011000#); --Image Buffer Base Address

end Wb_Ov5640_Address_Map;