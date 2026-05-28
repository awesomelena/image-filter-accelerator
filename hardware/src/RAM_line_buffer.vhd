library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
library work;
use work.RAM_definitions_PK.all;
use work.my_types_pkg.all;

entity RAM_line_buffer is
    generic (
        G_MAX_IMG_WIDTH : integer := 1024;
        G_MAX_RADIUS    : integer := 4
    );
    port (
        clk             : in  std_logic;
        reset           : in  std_logic;
        enable          : in  std_logic;

        curr_state      : in State_t;

        pixel_in        : in  std_logic_vector(7 downto 0);  -- prima s_axis_tdata

        tvalid          : in std_logic; --tvalid(PIPELINE-1)
        m_axis_tready   : in std_logic;
        
        s_axis_tvalid   : in std_logic;
        
        buff_flag       : in std_logic;
        buff_tdata      : in std_logic_vector(7 downto 0); 
        reg_img_w_for_processing : in std_logic_vector(15 downto 0);

        pixel_buf       : out std_logic_vector(2 * G_MAX_RADIUS * 8 - 1 downto 0)
    );
end RAM_line_buffer;

architecture Behavioral of RAM_line_buffer is

    constant WORD_WIDTH : integer := 2 * G_MAX_RADIUS * 8;  -- 64 bita
    constant RAM_DEPTH  : integer := G_MAX_IMG_WIDTH;         -- 512

    type ram_type is array (0 to RAM_DEPTH - 1) of std_logic_vector(WORD_WIDTH - 1 downto 0);
    signal ram_data : ram_type := (others => (others => '0'));
    
    attribute ram_style : string;
    attribute ram_style of ram_data : signal is "block";

    -- adresa za citanje ili pisanje
    signal read_addr : unsigned(clogb2(G_MAX_IMG_WIDTH)-1 downto 0);
    
    -- bram izlaz
    signal bram_rd_data : std_logic_vector(WORD_WIDTH - 1 downto 0);
    
    signal wb_pixel   : std_logic_vector(7 downto 0);
    signal wb_addr    : unsigned(clogb2(G_MAX_IMG_WIDTH)-1 downto 0);
    signal wb_valid   : std_logic;
    signal wb_rd_data : std_logic_vector(WORD_WIDTH - 1 downto 0);
    
    -- novi podatak za upis (shift left + insert)
    signal new_word : std_logic_vector(WORD_WIDTH - 1 downto 0);
    
begin
    BRAM_READ : process(clk)
    begin
        if rising_edge(clk) then
            if enable = '1' then
            
                case curr_state is
                when St_Processing =>
                    if (tvalid = '0' or m_axis_tready = '1') then 
                        bram_rd_data <= ram_data(to_integer(read_addr)); 
                    end if;
                when others => 
                    bram_rd_data <= ram_data(to_integer(read_addr));
                end case;
            end if;
        end if;
    end process BRAM_READ;

    -- pipeline stage 1: pamti ulazne podatke dok cekamo bram
    WRITE_BACK : process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                wb_pixel   <= (others => '0');
                wb_addr    <= (others => '0');
                wb_valid   <= '0';
                wb_rd_data <= (others => '0');
            elsif enable = '1' then
                case curr_state is
                
                
                when St_Idle => 
                    if (s_axis_tvalid = '1') then 
                        wb_pixel <= pixel_in;
                        wb_addr    <= read_addr;
                        wb_valid   <= '1';
                        wb_rd_data <= bram_rd_data;
                    end if;
                    
                    
                when St_Processing =>
                    if (tvalid = '0' or m_axis_tready = '1') then
                        if (buff_flag = '1') then
                            wb_pixel <= buff_tdata;
                        else
                            wb_pixel <= pixel_in;
                        end if;
                        wb_addr    <= read_addr;
                        wb_valid   <= '1';
                    else
                        wb_valid <= '0';
                    end if;
                when St_Done =>
                    wb_valid <= '0';
                when others =>
                    wb_valid <= '0';
                end case;
            end if;
        end if;
    end process WRITE_BACK;

    -- siftovanje i ubacivanje novog piksela
    new_word(WORD_WIDTH - 1 downto 8) <= bram_rd_data(WORD_WIDTH - 9 downto 0);
    new_word(7 downto 0) <= wb_pixel;

    BRAM_WRITE : process(clk)
    begin
        if rising_edge(clk) then
            if enable = '1' then
                if wb_valid = '1' then
                    ram_data(to_integer(wb_addr)) <= new_word;
                end if;
            end if;
        end if;
    end process BRAM_WRITE;

    ADDR_COUNTER : process(clk)
    begin
        if rising_edge(clk) then
            if reset = '1' then
                read_addr <= (others => '0');
            elsif enable = '1' then
                case curr_state is
                
                
                when St_Idle =>
                    if (s_axis_tvalid = '1') then 
                        read_addr <= read_addr + 1;    
                    end if;
                
                when St_Processing =>
                    if (tvalid = '0' or m_axis_tready = '1') then
                        if (read_addr < unsigned(reg_img_w_for_processing(clogb2(G_MAX_IMG_WIDTH)-1 downto 0)) - 1) then
                            read_addr <= read_addr + 1;
                        else
                            read_addr <= (others => '0');
                        end if;
                    end if;
                when St_Done =>
                    read_addr <= (others => '0');
                when others =>
                end case;
            end if;
        end if;
    end process ADDR_COUNTER;

    pixel_buf <= bram_rd_data;

end Behavioral;