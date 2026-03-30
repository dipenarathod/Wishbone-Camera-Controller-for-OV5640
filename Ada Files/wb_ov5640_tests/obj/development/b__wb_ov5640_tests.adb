pragma Warnings (Off);
pragma Ada_95;
pragma Source_File_Name (ada_main, Spec_File_Name => "b__wb_ov5640_tests.ads");
pragma Source_File_Name (ada_main, Body_File_Name => "b__wb_ov5640_tests.adb");
pragma Suppress (Overflow_Check);

package body ada_main is

   E05 : Short_Integer; pragma Import (Ada, E05, "ada__text_io_E");
   E09 : Short_Integer; pragma Import (Ada, E09, "input_output_helper_E");
   E15 : Short_Integer; pragma Import (Ada, E15, "input_output_helper__utils_E");
   E13 : Short_Integer; pragma Import (Ada, E13, "input_output_helper__debug_E");
   E37 : Short_Integer; pragma Import (Ada, E37, "riscv__csr_generic_E");
   E53 : Short_Integer; pragma Import (Ada, E53, "interrupts_E");
   E51 : Short_Integer; pragma Import (Ada, E51, "runtime_support_E");
   E40 : Short_Integer; pragma Import (Ada, E40, "uart0_E");
   E30 : Short_Integer; pragma Import (Ada, E30, "input_output_helper__time_measurements_E");
   E57 : Short_Integer; pragma Import (Ada, E57, "wb_ov5640_helper_E");
   E59 : Short_Integer; pragma Import (Ada, E59, "wb_ov5640_helper__debug_E");


   procedure adainit is
   begin
      null;

      Ada.Text_Io'Elab_Body;
      E05 := E05 + 1;
      E09 := E09 + 1;
      E15 := E15 + 1;
      E13 := E13 + 1;
      E37 := E37 + 1;
      Interrupts'Elab_Body;
      E53 := E53 + 1;
      E51 := E51 + 1;
      E40 := E40 + 1;
      E30 := E30 + 1;
      E57 := E57 + 1;
      E59 := E59 + 1;
   end adainit;

   procedure Ada_Main_Program;
   pragma Import (Ada, Ada_Main_Program, "_ada_wb_ov5640_tests");

   procedure main is
      Ensure_Reference : aliased System.Address := Ada_Main_Program_Name'Address;
      pragma Volatile (Ensure_Reference);

   begin
      adainit;
      Ada_Main_Program;
   end;

--  BEGIN Object file/option list
   --   /home/dipen/Downloads/neorv32-halv3/wb_ov5640_tests/obj/development/runtime_support.o
   --   /home/dipen/Downloads/neorv32-halv3/wb_ov5640_tests/obj/development/wb_ov5640_tests.o
   --   -L/home/dipen/Downloads/neorv32-halv3/wb_ov5640_tests/obj/development/
   --   -L/home/dipen/Downloads/neorv32-halv3/wb_ov5640_tests/obj/development/
   --   -L/home/dipen/Downloads/neorv32-halv3/ada_ml_library/lib/
   --   -L/home/dipen/.local/share/alire/builds/bare_runtime_14.0.0_095db6f0/282b01b920f0d5bb2bac604ac6d9e811f26d175144bc99af963e0381e797ee94/adalib/
   --   -L/home/dipen/Downloads/neorv32-halv3/input_output_helper/lib/
   --   -L/home/dipen/Downloads/neorv32-halv3/lib/
   --   -L/home/dipen/Downloads/neorv32-halv3/wb_ov5640_helper/lib/
   --   -static
   --   -lgnat
--  END Object file/option list   

end ada_main;
