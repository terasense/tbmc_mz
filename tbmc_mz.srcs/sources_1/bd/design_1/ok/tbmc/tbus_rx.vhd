library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.tbus_common.all;

-- The T-BUS receiver module
entity bus_receiver is
	generic (N : integer);
	port (
		-- Receiver addressing
		signal rx_this            : in integer range 0 to N-1;
		signal rx_select          : in integer range 0 to N-1;
		signal rx_active          : in integer range 0 to N-1;

		-- Clock
		signal clk                : in STD_LOGIC;
		signal bus_clk_en         : in STD_LOGIC;

		-- Bus signals
		signal bus_input          : in STD_LOGIC;

		-- Control triggers
		signal ctl_reset          : in STD_LOGIC;
		signal ctl_srq            : in STD_LOGIC;
		signal ctl_start          : in STD_LOGIC;

		-- Control signals
		signal rx_wait            : in STD_LOGIC;
		signal rx_stream          : in STD_LOGIC;
	 	signal rx_skip            : in STD_LOGIC_VECTOR(15 downto 0);
		signal rx_length1         : in STD_LOGIC_VECTOR(10 downto 0);

		-- Signals from the transmitter
		signal bus_bit_latch      : in BIT;
		signal bus_byte_completed : in BIT;

		-- Output signals
		signal st_hdr_done        : out STD_LOGIC;
		signal st_pause           : out STD_LOGIC;
		signal st_data_rdy        : out STD_LOGIC;
		signal st_completed       : out STD_LOGIC;
		signal st_underrun        : out STD_LOGIC;

		-- Receiver buffer interface
		signal buff_read          : STD_LOGIC;
		signal buff_raddr         : in UNSIGNED(9 downto 0);
		signal buff_out           : out STD_LOGIC_VECTOR(15 downto 0)
	);
end bus_receiver;

architecture RTL of bus_receiver is
	-- Response buffer 8x2048/16x1024 dual port
	COMPONENT bus_mem
		PORT (
			clka  : IN STD_LOGIC;
			wea   : IN STD_LOGIC_VECTOR(0 downto 0);
			addra : IN STD_LOGIC_VECTOR(10 downto 0);
			dina  : IN STD_LOGIC_VECTOR(7 downto 0);
			clkb  : IN STD_LOGIC;
			addrb : IN STD_LOGIC_VECTOR(9 downto 0);
			doutb : OUT STD_LOGIC_VECTOR(15 downto 0)
		);
	END COMPONENT;

	signal int_hdr_done  : STD_LOGIC := '0';
	signal int_pause     : STD_LOGIC := '0';
	signal int_data_rdy  : STD_LOGIC := '0';
	signal int_completed : STD_LOGIC := '0';
	signal int_underrun  : STD_LOGIC := '0';

	-- Receiver buffer registers
	signal buff_in      : STD_LOGIC_VECTOR(7 downto 0);
	signal buff_out_int : STD_LOGIC_VECTOR(15 downto 0);
	signal buff_waddr   : UNSIGNED(10 downto 0);
	signal buff_write   : STD_LOGIC;
	signal buff_write_en: STD_LOGIC_VECTOR(0 downto 0) := "0";

	alias chunk_waddr is buff_waddr(buff_waddr'HIGH-1 downto 0);
	alias chunk_raddr is buff_raddr(buff_raddr'HIGH-1 downto 0);

	signal rx_out_select    : integer range 0 to N-1 := 0;
begin

-- Response buffer
rxBuffer : bus_mem port map (
		clka=>clk, addra=>std_logic_vector(buff_waddr), dina=>buff_in, wea=>buff_write_en,
		clkb=>clk, addrb=>std_logic_vector(buff_raddr), doutb=>buff_out_int
	);

buff_out <= buff_out_int when rx_this = rx_out_select else (others => 'Z');

st_hdr_done  <= int_hdr_done  when rx_this <= rx_active else '1';
st_pause     <= int_pause     when rx_this <= rx_active else '0';
st_data_rdy  <= int_data_rdy  when rx_this <= rx_active else '1';
st_completed <= int_completed when rx_this <= rx_active else '1';
st_underrun  <= int_underrun  when rx_this <= rx_active else '0';

bus_receiver: process (clk, rx_this, rx_active)
	variable skip_cnt  : integer range 0 to 65535;
	variable hdr_cnt   : integer range 0 to 7;
	variable shift_reg : UNSIGNED(7 downto 0);
	variable byte_cnt  : integer range 0 to 2047;
	variable waiting   : STD_LOGIC;
	variable buff_fill : integer range 0 to 2;
begin
	if rising_edge(clk) and rx_this <= rx_active then

		-- reset write enable signal so it will last for single clock cycle
		buff_write_en <= "0";

		rx_out_select <= rx_select;

		if bus_clk_en = '1' then

			-- handle tx start
			if ctl_start = '1' then
				hdr_cnt := 0;
				skip_cnt := TO_INTEGER(UNSIGNED(rx_skip));
				buff_waddr <= (others => '0');
				buff_fill := 0;
				buff_write <= '0';
				shift_reg := (others => '0');
				byte_cnt := TO_INTEGER(UNSIGNED(rx_length1));
				waiting := rx_wait;
			end if;

			-- handle byte receiving on rising edge of the clock
			if bus_bit_latch = '1' then
				shift_reg := ROTATE_RIGHT(shift_reg, 1);
				shift_reg(7) := bus_input;
			end if;

			-- handle byte completed
			if bus_byte_completed = '1' then
				if waiting = '1' and TO_INTEGER(shift_reg) /= 0 then
					waiting := '0';
				end if;
				if skip_cnt /= 0 then
					skip_cnt := skip_cnt - 1;
				elsif waiting = '0' and int_completed = '0' then
					if hdr_cnt /= 7 then
						hdr_cnt := hdr_cnt + 1;
					else
						int_hdr_done <= '1';
					end if;
					buff_in <= std_logic_vector(shift_reg);
					buff_write <= '1';
					buff_write_en <= "1";
				end if;
			end if;

			-- the byte is written to buffer
			if buff_write = '1' then
				buff_write <= '0';
				if rx_stream = '0' then
					if byte_cnt = 0 then
						-- stop transmission
						int_data_rdy <= '1';
						int_completed <= '1';
					else
						buff_waddr <= buff_waddr + 1;						
						byte_cnt := byte_cnt - 1;
					end if;
				else
					-- streaming flow control
					if if_all(std_logic_vector(chunk_waddr), '1') = '1' then
						buff_fill := buff_fill + 1;
						int_data_rdy <= '1';
						if buff_fill = 2 then
							int_pause <= '1';
						end if;
					end if;			
					buff_waddr <= buff_waddr + 1;
				end if;
			end if;

		end if; -- bus_clk_en = '1'

		-- read flow control
		if rx_stream = '1' and buff_read = '1' and rx_this = rx_select then
			if if_all(std_logic_vector(chunk_raddr), '1') = '1' then
				if buff_fill = 0 then
					int_underrun <= '1';
				else
					if buff_fill = 2 then
						int_pause <= '0';
					end if;
					buff_fill := buff_fill - 1;
					if buff_fill = 0 then
						int_data_rdy <= '0';
					end if;
				end if;
			end if;
		end if;

		if bus_clk_en = '1' then
			-- receiver reset
			if ctl_reset = '1' or ctl_srq = '1' or ctl_start = '1' then
				int_hdr_done  <= '0';
				int_data_rdy  <= '0';
				int_pause     <= '0';
				int_completed <= '0';
				int_underrun  <= '0';
			end if;
		end if;
	end if;
end process;

end RTL;
