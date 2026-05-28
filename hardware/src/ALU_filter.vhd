library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.my_types_pkg.all;

entity ALU_filter is
    generic (
        MAX_RADIUS : integer := 4;
        PIPELINE_DEPTH : natural := 6 - 1 --ovo -1 jer je prvi stage napolju
    );

    port (
        clk            : in std_logic;
        reset          : in std_logic;
        enable         : in std_logic;

        -- Konfiguracija iz AXI-Lite
        reg_radius_for_processing : in std_logic_vector(15 downto 0);
        reg_img_w_for_processing : in std_logic_vector(15 downto 0);
	    reg_img_h_for_processing : in std_logic_vector(15 downto 0);
	    
        coeffs         : in coeff_array_type(0 to 80);
        coeff_scale    : in std_logic_vector(15 downto 0);
        mode           : in std_logic;  -- 0 = uint8, 1 = signed 9.7 fixed point
        bypass         : in std_logic;  -- 0 => nema bypass, 1 => bypass
        
        pixel_new      : in std_logic_vector(7 downto 0); -- novi piksel koji dolazi iz AXI-Stream-a
        pixel_buf      : in std_logic_vector(2 * MAX_RADIUS * 8 - 1 downto 0); -- stari redovi iz RAM_line_buffer
                    
        result_data    : out std_logic_vector(15 downto 0); --ovo ce biti ispljunuto
        result_valid   : out std_logic; --ovo je neophodno
        
        --ostali signali koji dolaze iz glavne komponente
        tvalid         : in std_logic;
        m_axis_tready   : in std_logic;
        curr_state     : in State_t;
        pixel_width_counter_filter  : in natural; 
        pixel_height_counter_filter : in natural
    );
end entity ALU_filter;

architecture behavioral of ALU_filter is

    constant MAX_K : integer := 2 * MAX_RADIUS + 1;  -- maksimalna velicina kernela = 9
    constant NUM_ROWS : integer := MAX_K;  
    constant NUM_COLS : integer := MAX_K;
    
    type shift_row_t is array (0 to NUM_COLS - 1) of unsigned (7 downto 0);
    type shift_reg_t is array (0 to NUM_ROWS - 1) of shift_row_t;

    type coeff_row_t is array (0 to NUM_COLS - 1) of signed (15 downto 0);
    type coeff_matrix_t is array (0 to NUM_ROWS - 1) of coeff_row_t;

    subtype product_t is signed(24 downto 0); -- Q9.15
    type product_row_t is array (0 to NUM_COLS - 1) of product_t;
    type product_matrix_t is array (0 to NUM_ROWS - 1) of product_row_t;

    subtype accum_t is signed(31 downto 0);

    -- signali kroz pipeline
    signal shift_reg    : shift_reg_t;
    signal coeff_matrix : coeff_matrix_t;

    -- pipeline faza 3: mnozenje
    signal products     : product_matrix_t;

    -- pipeline faza 4: parcijalne sume po redovima
    type row_sum_t is array (0 to NUM_ROWS - 1) of accum_t;
    signal row_sums     : row_sum_t;

    -- pipeline faza 5: finalna suma
    signal total_sum    : accum_t;

    -- pipeline faza 6: formatiranje izlaza
    signal out_data     : std_logic_vector(15 downto 0);
    
    type  bypass_pipeline is array (natural range <>) of std_logic_vector(7 downto 0);
    signal bypass_pipe : bypass_pipeline(0 to PIPELINE_DEPTH-2); -- -2 zato sto stajem kod predposlednjeg, a poslednji je samo out
    
    signal alu_valid_pipe: std_logic_vector(PIPELINE_DEPTH-1 downto 0); --validnost podatka zavisi od piksela koji smo ucitali
    signal pixel_width_counter  : natural;
    signal pixel_height_counter : natural;
    
begin
    gen_unpack_rows : for row in 0 to NUM_ROWS - 1 generate
        gen_unpack_cols : for col in 0 to NUM_COLS - 1 generate
            coeff_matrix(row)(col) <= signed(coeffs(row * NUM_COLS + col));
        end generate gen_unpack_cols;
    end generate gen_unpack_rows;
    
    
    
    proc_shift_reg : process(clk)
    begin
        if (rising_edge(clk)) then
            if reset = '1' then
                for r in 0 to NUM_ROWS - 1 loop
                    for c in 0 to NUM_COLS - 1 loop
                        shift_reg(r)(c) <= (others => '0');
                    end loop;                  
                end loop;
            else
                case (curr_state) is
                when St_Processing =>  
                    if (tvalid = '0' or m_axis_tready = '1') then  
                        --2. stage pipeline-a
                        for r in 1 to NUM_ROWS - 1 loop
                            if r < 2*to_integer(unsigned(reg_radius_for_processing)) + 1 then
                                shift_reg(r)(0) <= unsigned(pixel_buf(r*8 - 1 downto (r-1)*8));
                            else
                                shift_reg(r)(0) <= (others => '0');
                            end if;
                         end loop; 
                            
                        shift_reg (0)(0) <= unsigned(pixel_new);
                        for r in 0 to NUM_ROWS - 1 loop
                            for c in NUM_COLS - 1 downto 1 loop
                                shift_reg(r)(c) <= shift_reg(r)(c - 1);
                            end loop;
                        end loop;
                    end if;
                 when others => 
                 end case;                     
            end if;
        end if;
    end process proc_shift_reg;
    
    
    
    PIXEL_COUNTER_LOGIC: process(clk) --ovim obezbedjujem da radim takodje sa  odgovarajucim vrednostima pixel_counter-a
    begin
        if (rising_edge(clk)) then
            case (curr_state) is
            when St_Processing =>
                if (tvalid = '0' or m_axis_tready = '1') then
                    pixel_width_counter <= pixel_width_counter_filter;
                    pixel_height_counter <= pixel_height_counter_filter;
                end if;
            when others =>
                pixel_width_counter <= pixel_width_counter_filter;
                pixel_height_counter <= pixel_height_counter_filter;
            end case;
        end if;   
    end process PIXEL_COUNTER_LOGIC;
    
    ALU_VALID_LOGIC: process (clk)
    variable v_image_w : natural;
    variable v_image_h : natural;
    variable v_filter_radius : natural ;
    begin
        if (rising_edge(clk)) then
            if reset = '1' then
                alu_valid_pipe <= (others => '0');
            else
                v_image_w := to_integer(unsigned(reg_img_w_for_processing));
                v_image_h := to_integer(unsigned(reg_img_h_for_processing));
                v_filter_radius := to_integer(unsigned(reg_radius_for_processing));
                case (curr_state) is
                when St_Processing =>
                    if (tvalid = '0' or m_axis_tready = '1') then 
                    --1. stage je efektivno upisivanje brojaca
                    --2. stage pipeline-a
                        if ((pixel_height_counter < 2*v_filter_radius) or (pixel_width_counter < 2*v_filter_radius)) then
                            alu_valid_pipe(0)<= '0';
                        else
                            alu_valid_pipe(0)<= '1';   
                        end if;
                    --3. stage pipeline-a
                        alu_valid_pipe(1) <= alu_valid_pipe(0);
                    --4. stage pipeline-a
                        alu_valid_pipe(2) <= alu_valid_pipe(1);
                    --5. stage pipeline-a
                        alu_valid_pipe(3) <= alu_valid_pipe(2);
                    --6. stage pipeline-a
                        alu_valid_pipe(4) <= alu_valid_pipe(3);                                               
                    end if;
                when St_Done =>
                    alu_valid_pipe <= (others => '0');
                when others =>
                end case;               
            end if;
        end if;            
    end process ALU_VALID_LOGIC;
    
    result_valid <= alu_valid_pipe(PIPELINE_DEPTH -1);
    
    Calculation : process(clk)
    variable v_radius: integer;
    variable v_sum : accum_t;
    variable v_total : accum_t;
    variable v_scaled_result: signed(47 downto 0);
 
    begin
        
        if (rising_edge(clk)) then
            v_radius := to_integer(unsigned(reg_radius_for_processing));
            if reset = '1' then
                for r in 0 to NUM_ROWS - 1 loop
					for c in 0 to NUM_COLS - 1 loop
						products(r)(c) <= (others => '0');
					end loop;
				end loop;
				    
				for r in 0 to NUM_ROWS - 1 loop
					row_sums(r) <= (others => '0');
				end loop;
					
				total_sum <= (others => '0');
					
				out_data <= (others => '0');               
            else
                case (curr_state) is
                when St_Processing =>  
                    if (tvalid = '0' or m_axis_tready = '1') then
                        --2. stage pipeline-a je upis u ALU
                        if v_radius = 0 then
                            bypass_pipe(0) <= pixel_new;
                        else
                            bypass_pipe(0) <= std_logic_vector(shift_reg(v_radius)(v_radius-1));
                        end if;
                        --3. stage pipeline-a
                        for r in 0 to NUM_ROWS - 1 loop
                            for c in 0 to NUM_COLS - 1 loop
                                if(v_radius*2 - r >= 0 and v_radius*2 - c >= 0) then
                                    products(r)(c) <= resize(signed('0' & shift_reg(r)(c)) * coeff_matrix(v_radius*2 - r)(v_radius*2 - c), 25);
                                else
                                    products(r)(c) <= (others => '0');
                                end if;    
                            end loop;
                        end loop;
                            
                        bypass_pipe(1)<=bypass_pipe(0);
                            
                        --4. stage pipeline-a
                        for r in 0 to NUM_ROWS - 1 loop
                           row_sums(r) <= ((resize(products(r)(0), 32) +
                                          resize(products(r)(1), 32)) +
                                          (resize(products(r)(2), 32) +
                                          resize(products(r)(3), 32))) +
                                          ((resize(products(r)(4), 32) +
                                          resize(products(r)(5), 32)) +
                                          (resize(products(r)(6), 32) +
                                          resize(products(r)(7), 32))) +
                                          resize(products(r)(8), 32); 
                        end loop;
                            
                        bypass_pipe(2)<=bypass_pipe(1);
                        --5. stage pipeline-a
                        total_sum <= ((row_sums(0) + row_sums(1)) + (row_sums(2) + row_sums(3))) + ((row_sums(4) + row_sums(5)) + (row_sums(6) + row_sums(7))) + row_sums(8);
                            
                        bypass_pipe(3)<=bypass_pipe(2);
                            
                        --6. stage pipeline-a
                            
                        v_scaled_result := resize(total_sum * signed('0' & coeff_scale), 48);
                            
                        if(bypass = '0') then
                            if (mode = '0') then 
                               out_data (15 downto 8) <= (others => '0');
                               -- klipovanje:
                               if v_scaled_result(47) = '1' then
                                   out_data(7 downto 0) <= (others => '0');    -- ako je negativan klipuje se na 0
                               elsif v_scaled_result(46 downto 35) /= "000000000000" then
                                   out_data(7 downto 0) <= (others => '1');    -- ako je veci od 255 klipuje se na 255
                               else
                                   out_data (7 downto 0) <= std_logic_vector(v_scaled_result(34 downto 27));
                               end if;
                            else
                               -- out_data <= std_logic_vector(v_scaled_result(35 downto 20));   -- ovo je ok i radi wrap ali ja zelim clip   
                               -- klipovanje:
                               if v_scaled_result(47 downto 35) = "0000000000000" or v_scaled_result(47 downto 35) = "1111111111111" then
                                   out_data <= std_logic_vector(v_scaled_result(35 downto 20));
                               elsif v_scaled_result(47) = '0' then
                                   out_data <= "0111111111111111"; 
                               else
                                   out_data <= "1000000000000000"; 
                               end if;
                            end if;
                        else 
                            if (mode = '0') then 
                               out_data (15 downto 8) <= (others => '0');
                               out_data (7 downto 0) <= bypass_pipe(3);
                            else
                               out_data(15) <= '0';           
                               out_data (14 downto 7) <= bypass_pipe(3);
                               out_data (6 downto 0) <= (others => '0'); 
                            end if; 
                        end if;   
                    end if;
                when others =>
                end case;
            end if;
        end if;
    end process Calculation;
    
    result_data <= out_data;
    
end architecture behavioral;