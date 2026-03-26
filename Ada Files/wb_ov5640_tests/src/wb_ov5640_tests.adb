with Input_Output_Helper;                   use Input_Output_Helper;
with Input_Output_Helper.Debug;             use Input_Output_Helper.Debug;
with Input_Output_Helper.Utils;             use Input_Output_Helper.Utils;
with Input_Output_Helper.Time_Measurements;
use Input_Output_Helper.Time_Measurements;

with Interfaces;             use Interfaces;
with Ada.Text_IO;            use Ada.Text_IO;
with Uart0;
with Runtime_Support;
with Wb_Ov5640_Helper;       use Wb_Ov5640_Helper;
with Wb_Ov5640_Helper.Debug; use Wb_Ov5640_Helper.Debug;

procedure Wb_Ov5640_Tests is

   Image_Words    : Natural := Tensor_Words (100, False);
   Captured_Image : Word_Array (0 .. Image_Words - 1);
   Start_Cycles   : Unsigned_64;
   End_Cycles     : Unsigned_64;
   Delta_Cycles   : Unsigned_64;
   --Test pass or fail result print
   procedure Print_Result (Name : String; Passed : Boolean) is
   begin
      if Passed then
         Put_Line (Name & " PASS");
      else
         Put_Line (Name & " FAIL");
      end if;
   end Print_Result;

   --Test 1: Test if Camera SCCB programmer is done
   procedure Test_Camera_SCCB_Status is
      Result : Boolean;
   begin
      --  for Count in 1..10 loop
      --     Put_Line (Word'Image(Read_Reg(SCCB_PROGRAM_STATUS_REG_Addr)));
      --  end loop;
      Result := Is_Camera_Programmed;
      Print_Result ("Camera Programmed Test: ", Result);
   end Test_Camera_SCCB_Status;

   procedure Test_Image_Capture_And_Read is
   begin
      --Trigger capture
      Start_Cycles := Read_Cycle;
      Start_Capturing_Image;
      --Wait_While_Camera_Becomes_Busy;
      --clear control once camera has started
      --Write_Reg (CTRL_Addr, Word (0));
      --  for I in 1 .. 100 loop
      --     null;  --Small delay
      --  end loop;
      Wait_While_Camera_Busy;
      End_Cycles := Read_Cycle;

      Print_Time ("Time taken to capture image: ", End_Cycles - Start_Cycles);

      Start_Cycles := Read_Cycle;
      Read_Words_From_Image_Buffer (Captured_Image);
      End_Cycles := Read_Cycle;
      --Delta_Cycles := End_Cycles - Start_Cycles;
      Print_Time ("Time taken to read image: ", End_Cycles - Start_Cycles);
      --Print_Tensor_Q07 ("Image", Captured_Image, 100);
      Put_Line ("Image captured");
   end Test_Image_Capture_And_Read;

begin
   Uart0.Init (19200);
   Put_Line ("Reunning Test Cases----------------");
   -- After programming registers, wait for ISP to stabilize
   for I in 1 .. 1000 loop
      -- Delay ~1 second
      for J in 1 .. 1_000_000 loop
         null;
      end loop;
   end loop;
   --Write_Reg (CTRL_Addr, Word (0));
   Test_Camera_SCCB_Status;
   Set_Image_Resolution (100, 100);
   Print_Status_Register_Detail;
   --Test_Capture_Status_Only;
   Wait_For_Camera_Streaming;
   --Start_And_Read_Captured_Image;
   --loop
   --Test_Image_Capture_And_Read;
   --Print_Tensor_Q07 ("Image", Captured_Image, 100);
   --end loop;
   write_i2c (Word (16#3103#), Word (16#11#));
   write_i2c (Word (16#3008#), Word (16#82#));
   write_i2c (Word (16#3008#), Word (16#42#));


   write_i2c (Word (16#3017#), Word (16#ff#));
   write_i2c (Word (16#3018#), Word (16#ff#));

   write_i2c (Word (16#3103#), Word (16#03#));
   write_i2c (Word (16#3034#), Word (16#1A#));
   write_i2c (Word (16#3035#), Word (16#21#));
   write_i2c (Word (16#3036#), Word (16#46#));
   write_i2c (Word (16#3037#), Word (16#13#));
   write_i2c (Word (16#3108#), Word (16#01#));
   write_i2c (Word (16#3824#), Word (16#02#));
   write_i2c (Word (16#460C#), Word (16#22#));
   write_i2c (Word (16#4837#), Word (16#22#));

   write_i2c (Word (16#4300#), Word (16#30#));
   write_i2c (Word (16#501F#), Word (16#00#));
   write_i2c (Word (16#4740#), Word (16#20#));

   write_i2c (Word (16#3503#), Word (16#00#));
   write_i2c (Word (16#3a00#), Word (16#38#));
   write_i2c (Word (16#5001#), Word (16#ff#)); --a3
   write_i2c (Word (16#5003#), Word (16#08#));

   --Brightness settings
   write_i2c(Word(16#5587#) ,Word(16#20#));
   write_i2c(Word(16#5580#) ,Word(16#04#));
   write_i2c(Word(16#5588#) ,Word(16#01#)); 

   write_i2c (Word (16#3a0f#), Word (16#50#));
   write_i2c (Word (16#3a10#), Word (16#48#));
   write_i2c (Word (16#3a1b#), Word (16#50#));
   write_i2c (Word (16#3a1e#), Word (16#48#));
   write_i2c (Word (16#3a11#), Word (16#90#));
   write_i2c (Word (16#3a1f#), Word (16#21#));

   write_i2c (Word (16#3800#), Word (16#00#));
   write_i2c (Word (16#3801#), Word (16#08#));
   write_i2c (Word (16#3802#), Word (16#00#));
   write_i2c (Word (16#3803#), Word (16#02#));

   write_i2c (Word (16#3804#), Word (16#0a#));
   write_i2c (Word (16#3805#), Word (16#37#));
   write_i2c (Word (16#3806#), Word (16#07#));
   write_i2c (Word (16#3807#), Word (16#a1#));

   write_i2c (Word (16#3808#), Word (16#00#));
   write_i2c (Word (16#3809#), Word (16#64#));  --32 for 50x50
   write_i2c (Word (16#380a#), Word (16#00#));
   write_i2c (Word (16#380b#), Word (16#64#));  --32 for 50x50

   write_i2c (Word (16#380c#), Word (16#06#));
   write_i2c (Word (16#380d#), Word (16#14#));
   write_i2c (Word (16#380e#), Word (16#03#));
   write_i2c (Word (16#380f#), Word (16#D8#));

   write_i2c (Word (16#3810#), Word (16#00#));
   write_i2c (Word (16#3811#), Word (16#04#));
   write_i2c (Word (16#3812#), Word (16#00#));
   write_i2c (Word (16#3813#), Word (16#02#));

   write_i2c (Word (16#3814#), Word (16#31#));
   write_i2c (Word (16#3815#), Word (16#31#));

   write_i2c (Word(16#3820#), Word(16#47#));
   write_i2c (Word(16#3821#), Word(16#01#));
   write_i2c (Word(16#503d#), Word(16#00#));
   write_i2c (Word(16#300e#), Word(16#58#));
   write_i2c (Word(16#3008#), Word(16#02#));


   Print_Registers;
   Wait_For_Camera_Streaming;
   --Set_Image_Resolution (50, 50);
   Test_Image_Capture_And_Read;
   Test_Image_Capture_And_Read;
   Print_Tensor_Q07 ("Image", Captured_Image, 100);

--  Wait_For_Camera_Streaming;
--     Set_Image_Resolution (50, 50);
--     Test_Image_Capture_And_Read;
--     Test_Image_Capture_And_Read;
--     Test_Image_Capture_And_Read;
--     Print_Tensor_Q07 ("Image", Captured_Image, 50);

   loop
      null;
   end loop;

end Wb_Ov5640_Tests;
