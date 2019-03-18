library IEEE;
use IEEE.std_logic_1164.all;

package tbus_common is
	-- The version number
	constant version : integer := 4;
	-- The total number of bus channels available
	constant channels : integer := 32;
	-- The number of bits reserved for the number of channels in host interface
	constant channel_bits : integer := 10;
	-- The number of channels for which we have reserved pins
	constant channel_max : integer := 32;

	-- Utility functions

	function if_all(s: STD_LOGIC_VECTOR; v: STD_LOGIC) return STD_LOGIC;
	function if_any(s: STD_LOGIC_VECTOR; v: STD_LOGIC) return STD_LOGIC;

end package;

package body tbus_common is

	-- Utility functions

	function if_all(s: STD_LOGIC_VECTOR; v: STD_LOGIC) return STD_LOGIC is
	begin
		for i in 0 to s'LENGTH-1 loop
			if s(i) /= v then
				return '0';
			end if;
		end loop;
		return '1';
	end function;

	function if_any(s: STD_LOGIC_VECTOR; v: STD_LOGIC) return STD_LOGIC is
	begin
		for i in 0 to s'LENGTH-1 loop
			if s(i) = v then
				return '1';
			end if;
		end loop;
		return '0';
	end function;

end package body;
