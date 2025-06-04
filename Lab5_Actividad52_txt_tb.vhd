-- DUT: DataPath_v3.vhd
    -- R-Type instructions OK
    -- I-Type (LW, SW, ADDI) OK
    -- I-Type (BEQ) OK
    -- J Typy (J target) OK
    
-- Total Delay Max = 20.978ns 
-- Frecuencia de operación = 47.67MHz ()

-- ESTADO: 

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use STD.TEXTIO.ALL;                          -- Package para usar las funciones y tipos para manejar archivos
use IEEE.NUMERIC_STD.ALL;                    -- Permite la conversión de tipos (to_integer())
use IEEE.STD_LOGIC_TEXTIO.ALL;               -- Permite el manejo de señales STD_LOGIC y STD_LOGIC_VECTOR


entity Lab5_Actividad52_txt_tb is
--  Port ( );
end Lab5_Actividad52_txt_tb;

architecture Behavioral of Lab5_Actividad52_txt_tb is
    -- DUT 
    -------------------------------------------------
    component DataPath_v3 is
        -- Solo conexiones externas
        Port ( 
            clk : in std_logic ;
            -- REGISTER FILE
            RegDst_t: out std_logic ;
            RegWrite_t: out std_logic ;
            -- ALU
            ALUSrc_t: out std_logic ;
            ALUOp_t: out std_logic_vector (1 downto 0);
            -- DATA MEMORY
            MemWrite_t: out std_logic;
            MemRead_t: out std_logic ;
            MemtoReg_t: out std_logic ;
            -- PROGRAM COUNTER
       
            -- TESTEO 1
            ResultALU_1: out std_logic_vector (31 downto 0);
            ZeroFlag_ALU1_s : out std_logic ;
            OverflowFlag_ALU1_s: out std_logic ;
            pc_out: out std_logic_vector(31 downto 0);
            -- TESTEO 2
            Instruction2521_t: out std_logic_vector (4 downto 0); -- Rs
            Instruction2016_t: out std_logic_vector (4 downto 0); -- Rt
            Instruction1511_t: out std_logic_vector (4 downto 0); -- Rd
            Instruction150_t: out std_logic_vector (15 downto 0); -- Inmmediate
            Instruction50_t: out std_logic_vector (5 downto 0);   -- Funct
            -- TESTEO 3
            Instruction3126_t: out std_logic_vector (5 downto 0); --Opcode
            ReadData1_t: out std_logic_vector (31 downto 0);      -- Out de RF
            ReadData2_t: out std_logic_vector (31 downto 0);      -- Out de RF
            SignExtended_t : out STD_LOGIC_VECTOR(31 downto 0);
            ControlBits_t: out std_logic_vector (3 downto 0);     -- Código para MIPS ALU (OPERATION) -> ALU CONTROL
            NumB_ALU1_mux_t: out std_logic_vector (31 downto 0);  -- aquí va ReadData2_s o sign_s -> Generado ene l MUX 2 ALUSrc
            ReadData_s_t: out std_logic_vector (31 downto 0);     -- out de DM
            WriteData_s_t: out std_logic_vector (31 downto 0);     -- Generado por el MUX 3 MemtoReg
            -- TESTEO 4
            WriteRegister_mux1_t: out std_logic_vector (4 downto 0); -- aqui va instruction_s(20-16) o instruction_s(15-11)
            -- Implemnentar BEQ sin Control Path
            Branch_t: out std_logic; 
            -- Implementar Jump sin Cntrol Path
            Jump_t: out std_logic ;
            
            -- TESTEO DEL JUMP
            Instruction250_t : out std_logic_vector (25 downto 0);  -- test
            JumpAddressPor4_t: out std_logic_vector (27 downto 0);  -- test
            JumpAddress_t: out std_logic_vector (31 downto 0);      -- test
           
            -- TESTEO DE BEQ
            PC_mux4_t : out std_logic_vector (31 downto 0);
            Select_MUX_4_t: out std_logic;
            
            -- TESTEO MUX 4
            NextInstruction_t : out std_logic_vector (31 downto 0);
            ResultALU_2_t  : out std_logic_vector (31 downto 0);
            PC_in: out std_logic_vector (31 downto 0)
           
        );
    end component;
    ----------------------------------------------------------------
    
    -- Señales para conectar al DUT
    signal clk : std_logic ;
    signal I3126_Opcode_copy: std_logic_vector (5 downto 0);           -- Opcode (SW:43, LW:35) OK
    
    -- TESTEO DE BEQ
    signal Branch: std_logic ;                                         -- Habilitador Branch para BEQ   OK
    signal ZeroFlag_ALU1_s : std_logic ;                               -- OK
  
    -- TESTEO DEL JUMP (Completo)
    signal Jump: std_logic ;                                           -- Habilitador Jump para Jump Target OK
    signal I250_J_Type_Target_copy: std_logic_vector (25 downto 0);    -- Target Address OK
    signal JumpAddressPor4_t: std_logic_vector (27 downto 0);          -- "00" & Target address << 2 OK
    signal JumpAddress_t: std_logic_vector (31 downto 0);              -- JumpAddress definitivo, el que va a PC OK
   
    signal pc_out: std_logic_vector(31 downto 0);                      -- OK
    
    -- SIMULAR MUX 4 (Completo)
    signal Mux4_Select_Mux4: std_logic;                                -- COPIA DE Select_MUX_4_t OK
    signal InA_NextInstruction: std_logic_vector (31 downto 0);        -- COPIA DE NextInstruction_t OK
    signal InB_Offset_I_Type:  std_logic_vector (31 downto 0);         -- COPIA DE ResultALU_2_t OK
    signal Out_Mux4: std_logic_vector (31 downto 0);                   -- COPIA DE PC_mux4_t OK
    
    -- SIMULAR MUX 5 ()
    signal Mux5_Jump: std_logic;                                       -- OK
    signal InA_JumpAddress: std_logic_vector (31 downto 0);            -- OK
    signal InB_Out_Mux4:  std_logic_vector (31 downto 0);              -- Réplica de PC_mux4_t OK
    signal Out_Mux5: std_logic_vector (31 downto 0);                   -- Réplica de PC_in OK
    
    -- SIMULAR IM (Completo)
    signal ReadAddress_IM: std_logic_vector (31 downto 0);             -- Copia exacta de PC_out. OK
    signal I3126_Opcode: std_logic_vector (5 downto 0);                -- Original OK
    signal I2521_Rs: std_logic_vector (4 downto 0);                    -- Original OK
    signal I250_J_Type_Target: std_logic_vector (25 downto 0);         -- Origninal. Target Address OK
    signal I2016_Rt: std_logic_vector (4 downto 0);                    -- Original OK
    signal I1511_Rd: std_logic_vector (4 downto 0);                    -- Original OK
    signal I150_Offset_I_Type: std_logic_vector (15 downto 0);         -- Original. Aquí se almacena el INMMIDIATE OK
    signal I50_funct: std_logic_vector (5 downto 0);                   -- Original OK
    signal SignExtended_I_Type : STD_LOGIC_VECTOR(31 downto 0);        -- Versión extendida de Instruction150_t OK
    
    
    -- SIMULAR MUX 1 (Completo)
    signal Mux1_RegDst: std_logic ;                                    -- OK 0: Writeregister <= Rt (Útil en SW) 1: WriteRegister<=Rd (No ultil con I-Type)
    signal InA_I2016_Rt: std_logic_vector (4 downto 0);                -- OK Copia de I2016_Rt 
    signal InB_I1511_Rd: std_logic_vector (4 downto 0);                -- OK Copia de I1511_Rd. Aqui va el Rd (Ni en LW ni el SW se usa)
    signal Out_MUX1_WriteRegister_RF: std_logic_vector (4 downto 0);   -- OK Copia exacta de WriteRegister_mux1_t con fines de simulación
    
    -- SIMULAR REGISTER FILE (completo)
    signal RegWrite: std_logic ;                                       -- OK 1: Habilitar escritura de RF
    signal WriteRegister_mux1_t: std_logic_vector (4 downto 0);        -- OK aqui va instruction_s(20-16) o instruction_s(15-11)
    signal WriteData_s_t: std_logic_vector (31 downto 0);              -- OK Generado por el MUX 3 MemtoReg
    signal I2521_Rs_copy: std_logic_vector (4 downto 0);               -- OK Aquí va el RS (Almacena la dirección base del vector V. APlica para LW Y SW)
    signal I2016_Rt_copy: std_logic_vector (4 downto 0);               -- OK Es una copia de I2016_Rt . LO hice con fiens de testeo
    signal ReadData1_t: std_logic_vector (31 downto 0);                -- OK Out de RF (se dirige a NUM_A del ALU)
    signal ReadData2_t: std_logic_vector (31 downto 0);                -- OK Out de RF (se dirige a NUM_A del ALU)
    
    -- SIMULAR MUX 2 (Completo)
    signal Mux2_ALUSrc: std_logic ;                                    -- OK 0: NUM_B_ALU<=ReadData2_t (R-Type instructions) 1: NUM_B_ALU<=SignExtended_t (I-Type Instructions)
    signal InA_ReadData2_RF: std_logic_vector (31 downto 0);           -- OK Copia exacta de ReadData2_t
    signal InB_SignExtended_I_Type: std_logic_vector (31 downto 0);    -- OK Copia exacta de SignExtended_I_Type
    signal Out_Mux2_NumB_ALU1: std_logic_vector (31 downto 0);         -- OK Copia de NumB_ALU1
    
    -- SIMULAR ALU (Completo)
    signal ALUOp: std_logic_vector (1 downto 0);                       -- OK Permite elegir si operar con R-Type (10) o I-Type instructions (00)
    signal I50_funct_copy: std_logic_vector (5 downto 0);              -- OK Func en R-Type instructions
    signal ControlBits_t: std_logic_vector (3 downto 0);               -- OK Código para MIPS ALU (OPERATION) -> ALU CONTROL
    signal NumA_ALU1: std_logic_vector (31 downto 0);                  -- OK Copia de ReadData1_copy
    signal NumB_ALU1: std_logic_vector (31 downto 0);                  -- OK 0: Num_B_ALU<=ReadData2, 1:Num_B_ALU <= SignExtended_t. Con esta señal conocemos la decición del MUX 2
    
    signal ResultALU_1: std_logic_vector (31 downto 0);                -- OK
    signal OverflowFlag_ALU1_s: std_logic;                             -- OK
 
    -- TESTEO DM (Completo)
    signal MemWrite: std_logic;                                        -- OK
    signal Address_DM: std_logic_vector(31 downto 0);                  -- OK Una copia exacta de ResultALU_1
    signal WriteData_DM: std_logic_vector(31 downto 0);                -- OK Una copia exacta de signal ReadData2_t
    signal MemRead: std_logic ;                                        -- OK
    signal ReadData_s_t: std_logic_vector (31 downto 0);               -- OK out de DM
    
    -- TESTEO MUX 3
    signal Mux3_MemtoReg: std_logic ;                                  -- OK
    signal InA_ReadData_DM: std_logic_vector (31 downto 0);            -- OK Copia exacta de ReadData_s_t
    signal InB_ResultALU1: std_logic_vector (31 downto 0);             -- OK Copia exacta de ResultALU_1
    signal Out_Mux3_WriteData_RF: std_logic_vector (31 downto 0);      -- OK Copia de WriteData_s_t
   
    -- Clock period
    constant clk_period : time := 21 ns; -- ~ 20.978ns
    
    -- FILE HANDLER OR INDICATOR
    file outputhandle: text;  -- Indicador que apunta al archivo en disco "Lab05_Results2.txt"
    
begin
    -- 1. GENERACIÓN DE UN RELOJ DE 10ns con fines de simulación
    relojAxtix7 :process
    begin
        while true loop
            clk <= '1';
            wait for clk_period / 2;
            clk <= '0';
            wait for clk_period / 2;
        end loop;
    end process;
    
    -- 2. DISPOSITIVO A TESTEAR
    DUT_PATH_v3: DataPath_v3
        port map (
            clk => clk,
            RegDst_t => Mux1_RegDst,
            RegWrite_t => RegWrite,
            ALUSrc_t => Mux2_ALUSrc,
            ALUOp_t => ALUOp,
            MemWrite_t => MemWrite,
            MemRead_t => MemRead,
            MemtoReg_t => Mux3_MemtoReg,
       
            ResultALU_1 => ResultALU_1, 
            ZeroFlag_ALU1_s => ZeroFlag_ALU1_s, 
            OverflowFlag_ALU1_s => OverflowFlag_ALU1_s,
            pc_out => pc_out,
            Instruction2521_t => I2521_Rs,
            Instruction2016_t => I2016_Rt, 
            Instruction1511_t => I1511_Rd,
            Instruction150_t => I150_Offset_I_Type,
            Instruction50_t =>  I50_funct,
            -- TESTE 3
            Instruction3126_t => I3126_Opcode, 
            ReadData1_t => ReadData1_t,
            ReadData2_t =>ReadData2_t,
            SignExtended_t => SignExtended_I_Type,
            ControlBits_t =>ControlBits_t,
            NumB_ALU1_mux_t => NumB_ALU1, 
            ReadData_s_t => ReadData_s_t,
            WriteData_s_t =>WriteData_s_t,
            --TESTEO 4
            WriteRegister_mux1_t =>  WriteRegister_mux1_t,
            -- Implemnentar BEQ sin Control Path
            Branch_t => Branch, 
            -- Implementar Jump sin Cntrol Path
            Jump_t => Jump, 
            -- TESTEO DEL JUMP
            Instruction250_t => I250_J_Type_Target ,
            JumpAddressPor4_t => JumpAddressPor4_t,
            JumpAddress_t =>  JumpAddress_t,
            -- TESTEO DE BEQ
            PC_mux4_t => Out_Mux4,
            Select_MUX_4_t => Mux4_Select_Mux4,
            
            -- ADICIONAL
            NextInstruction_t => InA_NextInstruction,
            ResultALU_2_t => InB_Offset_I_Type,
            PC_in => Out_Mux5
        );
    
    -- DUPLICANDO SEÑALES CON FINES DE TESTEO
    I3126_Opcode_copy <= I3126_Opcode;
    I250_J_Type_Target_copy <= I250_J_Type_Target;
    Mux5_Jump <= Jump;
    InA_JumpAddress <= JumpAddress_t;
    InB_Out_Mux4 <= Out_Mux4;
    ReadAddress_IM <= pc_out;
    InA_I2016_Rt <= I2016_Rt;
    InB_I1511_Rd <= I1511_Rd;
    Out_MUX1_WriteRegister_RF <=  WriteRegister_mux1_t;
    I2521_Rs_copy <= I2521_Rs;
    I2016_Rt_copy <= I2016_Rt;
    InA_ReadData2_RF <= ReadData2_t;
    InB_SignExtended_I_Type <= SignExtended_I_Type;
    Out_Mux2_NumB_ALU1 <= NumB_ALU1;
    I50_funct_copy <= I50_funct;
    NumA_ALU1 <= ReadData1_t;
    Address_DM <= ResultALU_1;
    WriteData_DM <= ReadData2_t;
    InA_ReadData_DM <= ReadData_s_t;
    InB_ResultALU1 <= ResultALU_1;
    Out_Mux3_WriteData_RF <= WriteData_s_t;
    
    -- 3 PROCESO DE ESTIMULACIÓN
    estimulacion: process 
    begin
        wait for 9*clk_period; -- Ejecuta las 9 instrucciones 
        wait for 3*clk_period; -- Ciclos adicionales para validar que entramos en un bucle dentro 
                               -- del label "done".
        wait;
    end process;
    
    -- 5. PROCESO DE ESCRITURA DE RESULTADOS EN EL ARCHIVO Lab05_Results2.txt
   
   Escritura_Txt: process is 
        -- INICIO VARIABLES PARA EL MANJEO DE ARCHIVOS ---
        variable numeroDeLinea : integer := 0;
        variable estado: file_open_status;
        variable buffer2: line;
        -- Variables auxiliares si se requiere procesar valores temporales
        -- FIN VARIABLES PARA EL MANEJO DE ARCHIVOS ---
    begin
        report "Iniciando Testeo de Lab05_Actividad52.vhd" severity note;
    
        -- APERTURA DEL ARCHIVO DE SALIDA
        file_open(estado, outputhandle, "Lab05_Results2.txt", write_mode);
        assert estado = open_ok
            report "No se pudo crear el archivo para escribir los resultados"
            severity failure;
    
        -- ENCABEZADO
        write(buffer2, string'("RESULTADO DE TESTEO DEL DUT Lab5_Actividad52.vhd (MIPS con Control Combinacional de Data Path)"));
        writeline(outputhandle, buffer2);
        writeline(outputhandle, buffer2); -- línea en blanco
        write(buffer2, string'("CICLO | clk | I3126_Opcode_copy | pc_out | I2521_Rs | I2016_Rt | I1511_Rd | I150_Offset_I_Type | SignExtended_I_Type | Out_MUX1_WriteRegister_RF | WriteData_s_t | ResultALU_1 | Address_DM"));
        writeline(outputhandle, buffer2);
        writeline(outputhandle, buffer2); -- línea en blanco
    
        -- BUCLE DE ESCRITURA POR CADA CICLO DE RELOJ
        for ciclo in 1 to 11 loop
            wait until rising_edge(clk);
        
            write(buffer2, string'(integer'image(ciclo) & "    "));
            write(buffer2, std_logic'image(clk));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(unsigned(I3126_Opcode_copy))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(unsigned(pc_out))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(unsigned(I2521_Rs))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(unsigned(I2016_Rt))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(unsigned(I1511_Rd))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(signed(I150_Offset_I_Type))));  -- probablemente sea offset con signo
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(signed(SignExtended_I_Type))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(unsigned(Out_MUX1_WriteRegister_RF))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(signed(WriteData_s_t))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(signed(ResultALU_1))));
            write(buffer2, string'("    "));
            write(buffer2, integer'image(to_integer(unsigned(Address_DM))));
        
            writeline(outputhandle, buffer2);
        end loop;
    
        file_close(outputhandle);
    
        report "Verificación exitosa!" severity note;
        wait;
    end process;
    
end Behavioral;