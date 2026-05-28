library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
library work;
use work.my_types_pkg.all;


entity acc_image_filter is
    Generic (
        C_S_AXI_DATA_WIDTH : integer := 32;
        C_S_AXI_ADDR_WIDTH : integer := 9;   -- jer ima 86 32bitnih podataka
        MAX_IMAGE_WIDTH : integer := 1024;
        MAX_IMAGE_HEIGHT : integer := 512;  
        G_MAX_RADIUS    : integer := 4
    );
    Port ( reset : in STD_LOGIC;
           clk : in STD_LOGIC;
           		
           -------- AXI4-Lite interface -------
	       --  AXI4-Lite Write address channel
		   s_axi_lite_awaddr  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		   -- protection type (priviledge and security of transaction)
		   s_axi_lite_awprot  : in std_logic_vector(2 downto 0);
		   s_axi_lite_awvalid : in std_logic;
		   s_axi_lite_awready : out std_logic;
		
		   --  AXI4-Lite Write data channel
		   s_axi_lite_wdata  : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		   s_axi_lite_wstrb  : in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
		   s_axi_lite_wvalid : in std_logic;
		   s_axi_lite_wready : out std_logic;
		
		   --  AXI4-Lite Write response channel
		   s_axi_lite_bresp  : out std_logic_vector(1 downto 0);
		   s_axi_lite_bvalid : out std_logic;
		   s_axi_lite_bready : in std_logic;
		
		   --  AXI4-Lite Read address related signals
		   s_axi_lite_araddr  : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		   -- protection type (priviledge and security of transaction)
		   s_axi_lite_arprot  : in std_logic_vector(2 downto 0);
		   s_axi_lite_arvalid : in std_logic;
		   s_axi_lite_arready : out std_logic;
		
		   --  AXI4-Lite Read data related signals
		   s_axi_lite_rdata  : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		   s_axi_lite_rvalid : out std_logic;
		   s_axi_lite_rready : in std_logic;
		   s_axi_lite_rresp  : out std_logic_vector(1 downto 0);

           -- Input AXI Stream interface
           s_axis_tdata : in STD_LOGIC_VECTOR (7 downto 0);
           s_axis_tvalid : in STD_LOGIC;
           s_axis_tready : out STD_LOGIC;
           s_axis_tlast : in STD_LOGIC;
           
           -- Output AXI Stream interface
           m_axis_tdata : out STD_LOGIC_VECTOR (15 downto 0);
           m_axis_tvalid : out STD_LOGIC;
           m_axis_tready : in STD_LOGIC;
           m_axis_tlast : out STD_LOGIC);
end acc_image_filter;

architecture Behavioral of acc_image_filter is
    
    -- AXI4-Lite register addresses
    constant REG_CTRL_ADDR  :       std_logic_vector(6 downto 0) := "0000000";
    constant REG_RADIUS_ADDR  :     std_logic_vector(6 downto 0) := "0000001";
    constant REG_IMG_W_ADDR :       std_logic_vector(6 downto 0) := "0000010";
    constant REG_IMG_H_ADDR :       std_logic_vector(6 downto 0) := "0000011";
    constant REG_COEFF_SCALE_ADDR : std_logic_vector(6 downto 0) := "0000100";
    constant REG_COEFF_BASE_ADDR :  std_logic_vector(6 downto 0) := "0000101";
    -- When addressing 32-bit registers, 2 LSB of address are not used
    -- since each register occupies 4 byte addresses.
    constant ADDR_LSB   : natural := (C_S_AXI_DATA_WIDTH/32) + 1;

    -- Internal registers accessed via AXI4-Lite interface
    signal reg_ctrl : std_logic_vector(15 downto 0);
    signal reg_radius : std_logic_vector(15 downto 0);
	signal reg_img_w : std_logic_vector(15 downto 0);
	signal reg_img_h : std_logic_vector(15 downto 0);
	signal reg_coeff_scale : std_logic_vector(15 downto 0);
	signal coeff_array : coeff_array_type (0 to 80);
	
	-- Helper values
	signal reg_ctrl_for_processing : std_logic_vector(15 downto 0);  --uvedeno da se ne bi mogle koristiti neke random vrednosti uvedene ko zna odakle
    signal reg_radius_for_processing : std_logic_vector(15 downto 0);
	signal reg_img_w_for_processing : std_logic_vector(15 downto 0);
	signal reg_img_h_for_processing : std_logic_vector(15 downto 0);
	signal reg_coeff_scale_for_processing : std_logic_vector(15 downto 0);
	signal coeff_array_for_processing : coeff_array_type (0 to 80);
	-- Internal signals used for processing
	
    --Counters
    signal pixel_width_counter  : natural range 0 to MAX_IMAGE_WIDTH; 
    signal pixel_height_counter : natural range 0 to MAX_IMAGE_HEIGHT; 
    
	
    -- AXI4-Lite internal signals
    signal axi_awready : std_logic;
    signal axi_wready  : std_logic;
    signal axi_awaddr  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    
    signal axi_bvalid  : std_logic;
    
    signal axi_arready : std_logic;
    signal axi_rvalid  : std_logic;
    signal axi_araddr  : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);

    signal reg_waddr : std_logic_vector(C_S_AXI_ADDR_WIDTH-ADDR_LSB-1 downto 0);
    signal reg_raddr : std_logic_vector(C_S_AXI_ADDR_WIDTH-ADDR_LSB-1 downto 0);
    
    signal axi_write_ready : std_logic;
    signal axi_read_ready : std_logic;
    
    -- AXI4-Lite state machines
    type fsm_read_state_type is  (ReadAddress,  ReadData);
    type fsm_write_state_type is (WriteAddress, WriteData, WriteStalled);
    
    signal fsm_axi_read_state : fsm_read_state_type;
    signal fsm_axi_write_state : fsm_write_state_type;
    
    -- AXI Stream state machine
    constant PIPELINE_DEPTH : natural := 6;  --1) upis u glavnu jedinicu, 2)upis u ALU, 3) mnozenje sa koe, 4) sabiranje reda, 5) sabiranje svih redova, 6) mnozenje sa scalling_coef sto je i konacan rezultat
    signal tvalid_fifo : std_logic_vector(PIPELINE_DEPTH-1 downto 0);
    signal tlast_fifo  : std_logic_vector(PIPELINE_DEPTH-1 downto 0);
    
    signal reg_input   : std_logic_vector(7 downto 0);
    
    signal buff_tdata  : std_logic_vector(7 downto 0);
    signal buff_tvalid : std_logic;
    signal buff_tlast  : std_logic;
    signal buff_flag   : std_logic;
    
    
    --state for our component
    signal curr_state : State_t := St_Idle; --pocetno stanje
    -- Other signals needed for realisation
    signal pixels_from_BRAM : std_logic_vector (2 * G_MAX_RADIUS*8 -1 downto 0);
    signal ALU_valid: std_logic;
    signal ALU_result: std_logic_vector(15 downto 0);
    
begin
    BRAM: entity work.RAM_line_buffer (Behavioral) generic map (G_MAX_IMG_WIDTH => MAX_IMAGE_WIDTH, G_MAX_RADIUS => G_MAX_RADIUS)
                                 port map(clk => clk, reset => reset, enable => '1', curr_state => curr_state, pixel_in =>s_axis_tdata,
                                 tvalid => tvalid_fifo(PIPELINE_DEPTH-1), m_axis_tready => m_axis_tready, buff_flag => buff_flag, buff_tdata => buff_tdata,
                                 reg_img_w_for_processing => reg_img_w_for_processing, pixel_buf => pixels_from_BRAM, s_axis_tvalid => s_axis_tvalid);
                                 
    ALU: entity work.ALU_filter (Behavioral) generic map (MAX_RADIUS => G_MAX_RADIUS, PIPELINE_DEPTH => PIPELINE_DEPTH-1)--ovo ovako mora - 1
                                 port map (clk => clk, reset => reset, enable => '1', curr_state => curr_state, reg_radius_for_processing => reg_radius_for_processing,
                                 reg_img_w_for_processing => reg_img_w_for_processing, reg_img_h_for_processing => reg_img_h_for_processing,
                                 coeffs => coeff_array_for_processing, coeff_scale => reg_coeff_scale_for_processing, mode => reg_ctrl_for_processing(0),
                                 bypass => reg_ctrl_for_processing(1), pixel_new => reg_input, pixel_buf => pixels_from_BRAM, result_data => ALU_result,
                                 tvalid => tvalid_fifo(PIPELINE_DEPTH-1), m_axis_tready => m_axis_tready, pixel_width_counter_filter => pixel_width_counter,
                                 pixel_height_counter_filter => pixel_height_counter, result_valid => ALU_valid); 
        

    -- AXI4-Lite write registers
    process (clk) is
    variable x :integer := 0;
    begin
        if (rising_edge(clk)) then
            if (reset = '1') then
                reg_ctrl <= (others=>'0');
                reg_radius <= (others=>'0');
	            reg_img_w <= (others=>'0');
	            reg_img_h <= (others=>'0');
	            reg_coeff_scale <= (others=>'0');
	            for i in coeff_array'range loop
                    coeff_array(i) <= (others => '0');
                end loop;
            else
                if (axi_write_ready = '1') then
                    if (s_axi_lite_wstrb(0) = '1') then
                        case (reg_waddr) is
                            when REG_CTRL_ADDR  =>  reg_ctrl(7 downto 0) <= s_axi_lite_wdata(7 downto 0);
                            when REG_RADIUS_ADDR => reg_radius (7 downto 0) <= s_axi_lite_wdata(7 downto 0);
                            when REG_IMG_W_ADDR => reg_img_w (7 downto 0) <= s_axi_lite_wdata(7 downto 0);
                            when REG_IMG_H_ADDR => reg_img_h (7 downto 0) <= s_axi_lite_wdata(7 downto 0);
                            when REG_COEFF_SCALE_ADDR => reg_coeff_scale (7 downto 0) <= s_axi_lite_wdata(7 downto 0);
                            when others => 
                                x := to_integer (unsigned(reg_waddr)) - to_integer(unsigned(REG_COEFF_BASE_ADDR));
                                if(x < 81) then
                                    coeff_array(x)(7 downto 0) <= s_axi_lite_wdata(7 downto 0);
                                end if;  
                        end case;
                    end if;
                    
                    if (s_axi_lite_wstrb(1) = '1') then
                        case (reg_waddr) is
                            when REG_CTRL_ADDR  =>  reg_ctrl(15 downto 8) <= s_axi_lite_wdata(15 downto 8);
                            when REG_RADIUS_ADDR => reg_radius (15 downto 8) <= s_axi_lite_wdata(15 downto 8);
                            when REG_IMG_W_ADDR => reg_img_w (15 downto 8) <= s_axi_lite_wdata(15 downto 8);
                            when REG_IMG_H_ADDR => reg_img_h (15 downto 8) <= s_axi_lite_wdata(15 downto 8);
                            when REG_COEFF_SCALE_ADDR => reg_coeff_scale (15 downto 8) <= s_axi_lite_wdata(15 downto 8);
                            when others =>  
                                x := to_integer (unsigned(reg_waddr)) - to_integer(unsigned(REG_COEFF_BASE_ADDR));
                                if(x < 81) then
                                    coeff_array(x)(15 downto 8) <= s_axi_lite_wdata(15 downto 8);
                                end if;  
                        end case;
                    end if;                    
                end if;
            end if;
        end if;
    end process;
    
    -- AXI4-Lite read registers
    s_axi_lite_rdata(15 downto 0) <= reg_ctrl when (reg_raddr = REG_CTRL_ADDR ) else
                                     reg_radius  when (reg_raddr = REG_RADIUS_ADDR ) else
                                     reg_img_w when (reg_raddr = REG_IMG_W_ADDR) else
                                     reg_img_h when (reg_raddr = REG_IMG_H_ADDR) else
                                     reg_coeff_scale when (reg_raddr = REG_COEFF_SCALE_ADDR) else
                                     (others => '0') when (to_integer(unsigned(reg_raddr)) > to_integer(unsigned(REG_COEFF_BASE_ADDR)) + 80) else
                                     coeff_array(to_integer(unsigned(reg_raddr)) - to_integer(unsigned(REG_COEFF_BASE_ADDR)));
    s_axi_lite_rdata(C_S_AXI_DATA_WIDTH-1 downto 16) <= (others => '0');                          
    
    -- Set default value of read and write response to OKAY
    s_axi_lite_bresp <= "00";
    s_axi_lite_rresp <= "00";
    
    -- AXI4-Lite read state machine
    process (clk) is
    begin
        if (rising_edge(clk)) then
            if (reset = '1') then
                axi_arready <= '0';
                axi_rvalid  <= '0';
                fsm_axi_read_state <= ReadAddress;
            else
                case (fsm_axi_read_state) is
                    when ReadAddress =>
                        axi_arready <= '1';
                        if (axi_arready = '1' and s_axi_lite_arvalid = '1') then
                            axi_araddr <= s_axi_lite_araddr;
                            axi_arready <= '0';
                            axi_rvalid <= '1';
                            fsm_axi_read_state <= ReadData;
                        end if;
                    when ReadData =>
                        
                        if (s_axi_lite_rready = '1' and axi_rvalid = '1') then
                            axi_rvalid <= '0';
                            axi_arready <= '1';
                            fsm_axi_read_state <= ReadAddress;
                        end if;
                end case;
            end if;
        end if;
    end process;
    
    s_axi_lite_arready <= axi_arready;
    s_axi_lite_rvalid <= axi_rvalid;
    
    reg_raddr <= s_axi_lite_araddr(C_S_AXI_ADDR_WIDTH-1 downto ADDR_LSB) when (s_axi_lite_arvalid = '1') else
                        axi_araddr(C_S_AXI_ADDR_WIDTH-1 downto ADDR_LSB);
    
    -- AXI4-Lite write state machine
    process (clk) is
    begin
        if (rising_edge(clk)) then
            if (reset = '1') then
                axi_awready <= '0';
                axi_wready  <= '0';
                axi_bvalid  <= '0';
                fsm_axi_write_state <= WriteAddress;
            else
                case (fsm_axi_write_state) is                                              
                    when WriteAddress =>
                        axi_awready <= '1';
                        axi_wready <= '1';
                    
                        if (axi_awready = '1' and s_axi_lite_awvalid = '1') then
                            axi_awaddr <= s_axi_lite_awaddr;
                            if (axi_wready = '1' and s_axi_lite_wvalid = '1') then
                                axi_bvalid <= '1';
                                if (s_axi_lite_bready = '0') then
                                    axi_awready <= '0';
                                    axi_wready <= '0';
                                    fsm_axi_write_state <= WriteStalled;
                                end if;
                            else
                                axi_awready <= '0';
                                fsm_axi_write_state <= WriteData;
                                if (s_axi_lite_bready = '1' and axi_bvalid = '1') then
                                    axi_bvalid <= '0';
                                end if;
                            end if;
                        else
                            if (s_axi_lite_bready = '1' and axi_bvalid = '1') then
                                axi_bvalid <= '0';
                            end if;
                        end if;
                        
                    when WriteData =>
                        if (axi_wready = '1' and s_axi_lite_wvalid = '1') then
                            axi_bvalid <= '1';
                            if (s_axi_lite_bready = '0') then
                                axi_awready <= '0';
                                axi_wready <= '0';
                                fsm_axi_write_state <= WriteStalled;
                            else
                                axi_awready <= '1';
                                axi_wready <= '1';
                                fsm_axi_write_state <= WriteAddress;
                            end if;
                        else
                            if (s_axi_lite_bready = '1' and axi_bvalid = '1') then
                                axi_bvalid <= '0';
                            end if;
                        end if;
                        
                    when WriteStalled =>
                        if (s_axi_lite_bready = '1' and axi_bvalid = '1') then
                            axi_bvalid <= '0';
                            axi_awready <= '1';
                            axi_wready <= '1';
                            fsm_axi_write_state <= WriteAddress;
                        end if;
                        
                    when others =>
                        axi_awready <= '0';
                        axi_wready <= '0';
                        axi_bvalid <= '0';
                        fsm_axi_write_state <= WriteAddress;
                end case;
            end if;
        end if;
     end process;
     
    s_axi_lite_awready <= axi_awready;
    s_axi_lite_wready  <= axi_wready;
    s_axi_lite_bvalid <= axi_bvalid;
    
    axi_write_ready <= '1' when ((fsm_axi_write_state = WriteAddress and s_axi_lite_awvalid = '1' and s_axi_lite_wvalid = '1') or
                                 (fsm_axi_write_state = WriteData and s_axi_lite_wvalid = '1')) else '0';
    
    reg_waddr <= s_axi_lite_awaddr(C_S_AXI_ADDR_WIDTH-1 downto ADDR_LSB) when (s_axi_lite_awvalid = '1') else axi_awaddr(C_S_AXI_ADDR_WIDTH-1 downto ADDR_LSB);
    

    FILTERING_IMAGE : process (clk) is
    begin
        if (rising_edge(clk)) then
            if (reset = '1') then
                tvalid_fifo <= (others => '0');
                tlast_fifo  <= (others => '0');
                
                reg_input   <= (others => '0');
                
                reg_ctrl_for_processing <=(others => '0');
                reg_radius_for_processing <=(others => '0');
                reg_img_w_for_processing <=(others => '0');
                reg_img_h_for_processing <=(others => '0');
                reg_coeff_scale_for_processing <=(others => '0');
                coeff_array_for_processing <= (others => (others => '0'));
                 
                
                buff_tdata  <= (others => '0');
                buff_tvalid <= '0';
                buff_tlast  <= '0';
                buff_flag   <= '0';

                pixel_width_counter <= 0;
                pixel_height_counter <= 0;
                curr_state <= St_Idle;
                
            else
                case curr_state is
                when St_Idle =>
                    if (s_axis_tvalid = '1') then      
                        curr_state <= St_Processing;
                        
                        reg_ctrl_for_processing <= reg_ctrl;
                        reg_radius_for_processing <= reg_radius;
                        reg_img_w_for_processing <= reg_img_w;
                        reg_img_h_for_processing <= reg_img_h;
                        reg_coeff_scale_for_processing <= reg_coeff_scale;
                        coeff_array_for_processing <= coeff_array;
                        
                        reg_input      <= s_axis_tdata;
                        tvalid_fifo(0) <= s_axis_tvalid;
                        tlast_fifo(0)  <= s_axis_tlast;
                        pixel_width_counter <= 1; 
                        
                        
                    end if;
                when St_Processing => 
                    if (tvalid_fifo(PIPELINE_DEPTH-1) = '1' and m_axis_tready = '1' and tlast_fifo(PIPELINE_DEPTH-1) = '1') then
                        curr_state <= St_Done;
                    end if;   
                    if (tvalid_fifo(PIPELINE_DEPTH-1) = '0' or m_axis_tready = '1') then
                        if (buff_flag = '1') then
                            reg_input      <= buff_tdata;
                            tvalid_fifo(0) <= buff_tvalid;
                            tlast_fifo(0)  <= buff_tlast;
                            buff_flag <= '0';                       
                        else
                            reg_input      <= s_axis_tdata;
                            tvalid_fifo(0) <= s_axis_tvalid;
                            tlast_fifo(0)  <= s_axis_tlast;
                        end if;
                        -- 2nd pipeline stage
                        tvalid_fifo(1) <= tvalid_fifo(0);
                        tlast_fifo(1)  <= tlast_fifo(0);
                        -- 3rd pipeline stage
                        tvalid_fifo(2) <= tvalid_fifo(1);
                        tlast_fifo(2)  <= tlast_fifo(1);
                        -- 4th pipeline stage
                        tvalid_fifo(3) <= tvalid_fifo(2);
                        tlast_fifo(3)  <= tlast_fifo(2);
                        -- 5th pipeline stage
                        tvalid_fifo(4) <= tvalid_fifo(3);
                        tlast_fifo(4)  <= tlast_fifo(3);
                        -- 6th pipeline stage
                        tvalid_fifo(5) <= tvalid_fifo(4);
                        tlast_fifo(5)  <= tlast_fifo(4);
                        
                        --regulsanje brojaca
                        if (pixel_width_counter < to_integer(unsigned(reg_img_w_for_processing) - 1)) then 
                            pixel_width_counter <= pixel_width_counter + 1;
                        else
                            pixel_width_counter <= 0; 
                            if (pixel_height_counter < to_integer(unsigned(reg_img_h_for_processing) - 1 ))then
                                pixel_height_counter <= pixel_height_counter + 1;
                            else
                                pixel_height_counter <= 0;
                            end if;
                        end if;    
                    else
                        if (buff_flag = '0') then
                            buff_tdata  <= s_axis_tdata;
                            buff_tvalid <= s_axis_tvalid;
                            buff_tlast  <= s_axis_tlast;
                            buff_flag <= '1';
                        end if;    
                    end if;
                when St_Done => --ovo je bezbedno jer popunjavanje registara traje bar 86 taktnih signala
                    --resetovanje svih registara, memorija itd.
                    reg_ctrl_for_processing <=(others => '0');
                    reg_radius_for_processing <=(others => '0');
                    reg_img_w_for_processing <=(others => '0');
                    reg_img_h_for_processing <=(others => '0');
                    reg_coeff_scale_for_processing <=(others => '0');
                    coeff_array_for_processing <= (others => (others => '0'));
            
                    curr_state <= St_Idle;
                    pixel_width_counter <= 0; 
                    pixel_height_counter <= 0;
                when others =>           
                end case;       
            end if;
        end if;
    end process;
    
    m_axis_tvalid <= tvalid_fifo(PIPELINE_DEPTH-1) and ALU_valid;
    m_axis_tlast  <= tlast_fifo(PIPELINE_DEPTH-1);
    m_axis_tdata  <= ALU_result;
    
    process (clk) is --ne moze da prima ako salje ili ako master nije ready
    begin
        if (rising_edge(clk)) then
            if (reset = '1') then
                s_axis_tready <= '0';
            else
                if (tvalid_fifo(PIPELINE_DEPTH-1) = '0' or m_axis_tready = '1') then
                    s_axis_tready <= '1';
                else
                    s_axis_tready <= '0';
                end if;
            end if;
        end if;
    end process;
end Behavioral;
