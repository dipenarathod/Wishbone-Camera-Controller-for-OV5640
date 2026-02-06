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
      Wait_While_Camera_Becomes_Busy;
      --clear control once camera has started
      Write_Reg (CTRL_Addr, Word (0));
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
      Print_Tensor_Q07 ("Image", Captured_Image, 100);
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
   --Test_Capture_Status_Only;
   --Wait_For_Camera_Streaming;
   --Start_And_Read_Captured_Image;
   --loop
   Test_Image_Capture_And_Read;
   --end loop;
   loop
      null;
   end loop;

end Wb_Ov5640_Tests;
