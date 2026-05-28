library ieee;
use ieee.std_logic_1164.all;

package my_types_pkg is

    type coeff_array_type is array (natural range <>) of std_logic_vector(15 downto 0);
    type State_t is(St_Idle, St_Processing, St_Done, St_Wait);

end package my_types_pkg;

package body my_types_pkg is 
end package body my_types_pkg;