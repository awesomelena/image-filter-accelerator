library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package RAM_definitions_PK is
    impure function clogb2 (depth: in natural) return integer;
end RAM_definitions_PK;

package body RAM_definitions_PK is
    --  The following function calculates the address width based on specified RAM depth
    impure function clogb2( depth : natural) return integer is
        variable temp    : integer := depth;
        variable ret_val : integer := 0;
    begin
        while temp > 1 loop
            ret_val := ret_val + 1;
            temp    := temp / 2;
        end loop;
        return ret_val;
    end function;
end package body RAM_definitions_PK;