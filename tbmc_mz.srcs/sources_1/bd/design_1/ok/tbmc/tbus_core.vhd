
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.tbus_common.all;

-- The multichannel T-BUS controller
entity TBus_core is
	-- The number of channels
	generic (
		N : integer := channels;
		MAX_CHANS : integer := channel_max;
		CLK_MHZ : integer := 48
	);
	port (
	-- TBUS signals
		sdi  : in  STD_LOGIC_VECTOR(0 to MAX_CHANS-1);
		sdo  : out STD_LOGIC;
		sclk : out STD_LOGIC;
		srq  : out STD_LOGIC;

	-- Sync outputs
		fsync   : out STD_LOGIC;
		nfsync  : out STD_LOGIC;
		fsync2  : out STD_LOGIC;
		fsync2d : out STD_LOGIC;

	-- Board LEDs
		led : out STD_LOGIC_VECTOR(7 downto 0);

	-- Control ports -----------------------------------------
	-- Reset
		nrst : in STD_LOGIC;

	-- Host clock
		clk : in STD_LOGIC;
		bus_clk_en : in STD_LOGIC;

	-- Trigger bit has active '1' level during single clock cycle:
	--   bit 0 bus reset
	--   bit 1 bus srq
	--   bit 2 send command
	--   bit 8 stop tranceiver
	--   bit 12 start auto mode
		bus_trigger : in STD_LOGIC_VECTOR(15 downto 0);

	-- Global configuration register
		bus_cfg_ctl : in STD_LOGIC_VECTOR(15 downto 0);

	-- Bus frequency control register
	--   bits 0..7  master clock (48MHz) divider for tbus clock
	--      the tbus clock freq. will be 48/2(N+1) MHz
	--   bits 8..15 the byte transmittion interval (usec)
		bus_freq_ctl : in STD_LOGIC_VECTOR(15 downto 0);

	-- Extended frequency control register
	--   bits 8..15 the byte transmittion interval in turbo mode (usec)
		bus_freqx_ctl : in STD_LOGIC_VECTOR(15 downto 0);

	-- Bus reset control register.
	--   bits 0..7  bus reset duration (usec)
	--   bits 8..15 clock hold time on hard reset (usec)
		bus_rst_ctl : in STD_LOGIC_VECTOR(15 downto 0);

	-- Command control register
		bus_cmd_ctl : in STD_LOGIC_VECTOR(15 downto 0);

	-- Receiver control register
		bus_rx_ctl : in STD_LOGIC_VECTOR(15 downto 0);

	-- Skip bytes on receive
		bus_rx_skip : in STD_LOGIC_VECTOR(15 downto 0);

	-- Version register
		bus_version : out STD_LOGIC_VECTOR(15 downto 0);

	-- Bus status register
		bus_status : out STD_LOGIC_VECTOR(15 downto 0);

	-- Bus input state is output to this port
		bus_input_state : out STD_LOGIC_VECTOR(MAX_CHANS-1 downto 0);

	-- Command buffer access ports
		cmd_buff_write : in STD_LOGIC;
		cmd_buff_in    : in STD_LOGIC_VECTOR(15 downto 0);

	-- Receiver buffer access ports
		rx_buff_read  : in  STD_LOGIC;
		rx_pipe_data  : out STD_LOGIC_VECTOR(15 downto 0)
	);
end TBus_core;

architecture abstract of TBus_core is

	signal ctl_trigger : STD_LOGIC_VECTOR(15 downto 0) := (others => '0');

	alias ctl_reset is ctl_trigger(0);
	alias ctl_srq   is ctl_trigger(1);
	alias ctl_start is ctl_trigger(2);
	alias ctl_stop  is ctl_trigger(8);
	alias ctl_auto  is ctl_trigger(12);

	-- The number of active chunnels-1
	alias cfg_channels1 is bus_cfg_ctl(channel_bits-1 downto 0);
	alias cfg_use_leds  is bus_cfg_ctl(15);
 
	alias clk_div      is bus_freq_ctl(7 downto 0);
	alias clk_interval is bus_freq_ctl(15 downto 8);

	alias clk_intervalx is bus_freqx_ctl(15 downto 8);

	alias rst_time is bus_rst_ctl(7 downto 0);
	alias rst_hold is bus_rst_ctl(15 downto 8);

	-- command length in bytes minus 1
	alias cmd_length1 is bus_cmd_ctl(8 downto 0);
	-- turbo mode
	alias cmd_turbo   is bus_cmd_ctl(10);
	-- 16 bit mode
	alias cmd_16_bit  is bus_cmd_ctl(11);
	-- loop transmission (for self testing)
	alias cmd_loop    is bus_cmd_ctl(15);

	-- response length in bytes minus 1
	alias rx_length1  is bus_rx_ctl(10 downto 0);
	-- wait non zero response
	alias rx_wait     is bus_rx_ctl(13);
	-- continuous receive
	alias rx_stream   is bus_rx_ctl(14);
	-- connect receiver to transmitter internally (for self testing)
	alias rx_loop     is bus_rx_ctl(15);

	-- The firmware version
	alias v_version  is bus_version(15 downto channel_bits);
	-- The total number chunnels-1
	alias v_channels is bus_version(channel_bits-1 downto 0);

	signal int_status : STD_LOGIC_VECTOR(15 downto 0) := (others => '0');
	
	alias st_ready    is int_status(0);
	alias st_reset    is int_status(1);
	alias st_hold     is int_status(2);
	alias st_active   is int_status(3);
	alias st_tx_done  is int_status(4);
	alias st_stopping is int_status(6);
	alias st_error    is int_status(7);
	alias st_tx       is int_status(6 downto 1);

	alias st_data_rdy     is int_status(8);
	alias st_completed    is int_status(9);
	alias st_pause        is int_status(10);
	alias st_rd_unaligned is int_status(11);
	alias st_hdr_done     is int_status(12);
	alias st_underrun     is int_status(15);
	alias st_rx           is int_status(15 downto 8);

	-- Internal state LEDs signals
	signal st_led         : STD_LOGIC_VECTOR(7 downto 0);

	-- Bus command buffer registers
	signal cmd_buff_out   : STD_LOGIC_VECTOR(15 downto 0);
	signal cmd_buff_addr  : UNSIGNED(7 downto 0);

	-- Bus buffer registers
	signal bus_clk        : STD_LOGIC := '0';
	signal bus_srq        : STD_LOGIC := '0';
	signal bus_output     : STD_LOGIC := '0';
	signal bus_input      : STD_LOGIC_VECTOR(MAX_CHANS-1 downto 0);
	signal bus_ready      : STD_LOGIC := '0';

	signal fsync_out      : STD_LOGIC := '0';
	signal fsync2_out     : STD_LOGIC := '0';

	-- The following signals are driven by the transmitter and used by receivers
	signal bus_bit_latch      : BIT;
	signal bus_byte_completed : BIT;

	-- Receiver status flags
	signal rx_hdr_done        : STD_LOGIC_VECTOR(0 to N-1);
	signal rx_pause           : STD_LOGIC_VECTOR(0 to N-1);
	signal rx_data_rdy        : STD_LOGIC_VECTOR(0 to N-1);
	signal rx_completed       : STD_LOGIC_VECTOR(0 to N-1);
	signal rx_underrun        : STD_LOGIC_VECTOR(0 to N-1);

	-- Receiver select
	signal rx_select          : integer range 0 to N;

	-- Receiver buffer registers
	signal rx_buff_out   : STD_LOGIC_VECTOR(15 downto 0);
	signal rx_buff_raddr : UNSIGNED(9 downto 0) := (others => '0');

	alias rx_chunk_raddr is rx_buff_raddr(rx_buff_raddr'HIGH-1 downto 0);

	signal rx_buff_ready  : STD_LOGIC := '0';
	signal rx_buff_out_en : STD_LOGIC := '0';
	signal rx_reset       : STD_LOGIC;

	-- Auto mode run state
	type RUN_STATE_T is (s_Idle, s_Srq, s_WaitReady, s_WaitBuff, s_ReadData);
	signal run_state      : RUN_STATE_T := s_Idle;
	signal run_state_next : RUN_STATE_T := s_Idle;

	-- Internal triggers
	signal trg_srq      : STD_LOGIC := '0';
	signal run_srq      : STD_LOGIC := '0';
	signal trg_start    : STD_LOGIC := '0';
	signal run_read     : STD_LOGIC := '0';

	-- Command memory 16x256 single port 
	COMPONENT cmd_mem
		PORT (
			a   : IN STD_LOGIC_VECTOR(7 downto 0);
			d   : IN STD_LOGIC_VECTOR(15 downto 0);
			clk : IN STD_LOGIC;
			we  : IN STD_LOGIC;
			spo : OUT STD_LOGIC_VECTOR(15 downto 0)
		);
	END COMPONENT;

begin

v_channels <= std_logic_vector(TO_UNSIGNED(N-1,     v_channels'LENGTH));
v_version  <= std_logic_vector(TO_UNSIGNED(version, v_version'LENGTH));

-- Bus signals
sclk      <= bus_clk;
srq       <= bus_srq;
sdo       <= bus_output;

-- Sync signals
fsync   <= fsync_out;
nfsync  <= not fsync_out;
fsync2  <= fsync2_out;
fsync2d <= fsync2_out;

rx_reset <= ctl_reset or ctl_auto or trg_start;

bus_gen : for i in 0 to N-1 generate
	bus_input(i) <=
		'0'    when i > TO_INTEGER(UNSIGNED(cfg_channels1)) else
		sdi(i) when rx_loop = '0' else
		bus_output;
end generate;

-- Internal triggers
trg_srq   <= ctl_srq   or run_srq;
trg_start <= ctl_start or run_read;

-- State flags

st_rd_unaligned <= if_any(std_logic_vector(rx_chunk_raddr), '1');

st_pause     <= if_any(rx_pause, '1');
st_hdr_done  <= if_all(rx_hdr_done, '1');
st_data_rdy  <= if_all(rx_data_rdy, '1');
st_completed <= if_all(rx_completed, '1');
st_underrun  <= if_any(rx_underrun, '1');

-- wire LEDs
st_led(0) <= st_ready;
st_led(1) <= st_active;
st_led(2) <= st_tx_done;
st_led(3) <= st_stopping;
st_led(4) <= st_pause;
st_led(5) <= st_completed;
st_led(6) <= st_underrun;
st_led(7) <= st_error;

-- active LED level is 0
led <= not st_led when cfg_use_leds = '1' else  (others => '1');

-- Receiver buffer pipe
rx_pipe_data <= rx_buff_out when rx_stream = '1' or rx_buff_out_en = '1' else (others => '0');

-- Command buffer
cmdBuffer : cmd_mem port map (
		a=>std_logic_vector(cmd_buff_addr), d=>cmd_buff_in, clk=>clk, we=>cmd_buff_write, spo=>cmd_buff_out
	);

-- Instantiate receivers
rx_gen : for i in 0 to N-1 generate
	rx_n : entity work.bus_receiver(RTL)
		generic map (N=>N)
		port map (
			rx_this      => i,
			rx_select    => rx_select,
			rx_active    => TO_INTEGER(UNSIGNED(cfg_channels1)),
			clk          => clk,
			bus_clk_en   => bus_clk_en,
			bus_input    => bus_input(i),
			ctl_reset    => rx_reset,
			ctl_srq      => trg_srq,
			ctl_start    => trg_start,
			rx_wait      => rx_wait,
			rx_stream    => rx_stream,
			rx_skip      => bus_rx_skip,
			rx_length1   =>rx_length1,
			bus_bit_latch=> bus_bit_latch,
			bus_byte_completed => bus_byte_completed,
			st_hdr_done  => rx_hdr_done(i),
			st_pause     => rx_pause(i),
			st_data_rdy  => rx_data_rdy(i),
			st_completed => rx_completed(i),
			st_underrun  => rx_underrun(i),
			buff_read    => rx_buff_read,
			buff_raddr   => rx_buff_raddr,
			buff_out     => rx_buff_out
		);
end generate;

-- Status latch
status_latch: process (clk)
begin
	if rising_edge(clk) then
		if nrst = '0' then
			bus_status <= (others => '0');
			bus_input_state <= (others => '0');
		else
			bus_status <= int_status;
			bus_input_state <= bus_input;
		end if;
	end if;
end process;

-- Trigger latch
trigger_latch: process (clk)
begin
	if rising_edge(clk) then
		if nrst = '0' then
			-- reset all triggers
			ctl_trigger <= (others => '0');
			-- set reset trigger
			ctl_reset <= '1';
		else
			if bus_clk_en = '1' then
				ctl_trigger <= bus_trigger;
			else
				ctl_trigger <= ctl_trigger or bus_trigger;
			end if;
		end if;
	end if;
end process;

-- Ready status conditioner (the bus_input is asynchronous)
ready_latch: process (clk, bus_clk_en)
begin
	if rising_edge(clk) and bus_clk_en = '1' then
		if bus_srq = '1' then
			-- Use double latching to avoid metastability issues
			bus_ready <= if_all(bus_input, '0');
			st_ready <= bus_ready;
		else
			bus_ready <= '0';
			st_ready <= '0';
		end if;
	end if;
end process;

-- Auto mode controller
auto_sequencer: process (clk, bus_clk_en)
begin
	if rising_edge(clk) and bus_clk_en = '1' then
		if ctl_reset = '1' then
			-- handle reset
			run_state_next <= s_Idle;
		else
			-- initiate state transitions
			if run_state = s_Idle and ctl_auto = '1' then
				run_state_next <= s_Srq;
			end if;
			if run_state = s_Srq and st_ready = '0' then
				run_state_next <= s_WaitReady;
			end if;
			if run_state = s_WaitReady and st_ready = '1' then
				if rx_buff_ready = '1' then
					run_state_next <= s_WaitBuff;
				else
					run_state_next <= s_ReadData;
				end if;
			end if;
			if run_state = s_WaitBuff and rx_buff_ready = '0' then
				run_state_next <= s_ReadData;
			end if;
			if run_state = s_ReadData and rx_buff_ready = '1' then
				run_state_next <= s_Srq;
			end if;
		end if;
	end if;
end process;

auto_controller: process (clk, bus_clk_en)
begin
	if rising_edge(clk) and bus_clk_en = '1' then
		-- defaults
		run_srq  <= '0';
		run_read <= '0';
		if ctl_reset = '1' then
			-- handle reset
			fsync_out  <= '0';
			fsync2_out <= '0';
			run_state <= s_Idle;
		else
			-- generate internal triggers on state transitions
			if run_state /= run_state_next then
				if run_state_next = s_Srq then
					fsync_out <= '1';
					run_srq <= '1';
				elsif run_state_next = s_ReadData then
					run_read <= '1';
				end if;
				if run_state = s_WaitReady then
					fsync_out <= '0';
					fsync2_out <= not fsync2_out;
				end if;
			end if;
			-- update state
			run_state <= run_state_next;
		end if;
	end if;
end process;

-- The following process represents receiver buffers as one continuous buffer to the host
buff_keeper: process (clk)
begin
	if rising_edge(clk) then
		if bus_clk_en = '1' then
			if rx_reset = '1' then
				-- reset receiver state
				rx_select <= 0;
				rx_buff_raddr <= (others => '0');
				rx_buff_ready <= '0';
			elsif st_completed = '1' then
				-- host may read data buffer from this point
				rx_buff_ready <= '1';
			end if;
		end if;
		-- handle buffer read
		if rx_buff_read = '1' then
			if rx_stream = '0' then
				if rx_buff_ready = '1' then
					-- read requested length from each buffer sequentially
					if TO_INTEGER(rx_buff_raddr) = TO_INTEGER(SHIFT_RIGHT(UNSIGNED(rx_length1), 1)) then
						if rx_select = TO_INTEGER(UNSIGNED(cfg_channels1)) then
							-- will output 0s on further host requests
							rx_buff_ready <= '0';
						end if;
						rx_buff_raddr <= (others => '0');
						rx_select <= rx_select + 1;
					else
						rx_buff_raddr <= rx_buff_raddr + 1;
					end if;
				end if;
			else
				-- stream mode
				-- read the whole chunk (half of the buffer) from each buffer sequentially then proceed with the next chunk
				if if_all(std_logic_vector(rx_chunk_raddr), '1') = '1' then
					if rx_select = TO_INTEGER(UNSIGNED(cfg_channels1)) then
						rx_select <= 0;
						rx_buff_raddr <= rx_buff_raddr + 1;					
					else
						rx_select <= rx_select + 1;
						rx_chunk_raddr <= (others => '0');
					end if;
				else
					rx_buff_raddr <= rx_buff_raddr + 1;
				end if;
			end if;
		end if;
		-- enable read of the buffer
		rx_buff_out_en <= rx_buff_ready;
	end if;
end process;

-- Bus transmitter implementation
bus_keeper: process (clk)
	variable mhz_clk_div   : integer range 0 to CLK_MHZ-1;
	variable bus_clk_div   : integer range 0 to 255;
	variable rst_delay     : integer range 0 to 255;
	variable tx_delay      : integer range 0 to 255;
	variable shift_reg     : UNSIGNED(7 downto 0);
	variable next_byte     : integer range 0 to 1;
	variable bit_cnt       : integer range 0 to 8;
	variable byte_cnt      : integer range 0 to 511;
	variable byte_tx       : BIT;
	variable next_clk      : STD_LOGIC;

	procedure initialize is
	begin
		-- clear all but bus ready bit and reset everything
		st_tx         <= (others => '0');
		cmd_buff_addr <= (others => '0');
		bus_byte_completed <= '0';
		bus_bit_latch <= '0';
		bus_output  <= '0';
		bus_srq     <= '0';
		mhz_clk_div := 0;
		bus_clk_div := 0;
		rst_delay   := 0;
		tx_delay    := 0;
		shift_reg   := (others => '0');
		next_byte   := 0;
		bit_cnt     := 0;
		byte_cnt    := 0;
		byte_tx     := '0';
	end procedure initialize;

	procedure tx_reset is
	begin
		byte_cnt := TO_INTEGER(UNSIGNED(cmd_length1));
		cmd_buff_addr <= (others => '0');
		next_byte := 0;
	end procedure tx_reset;

	procedure tx_start is
	begin
		-- clear all but error and bus ready bits
		st_tx <= (others => '0');
		-- initialize transceiver state
		tx_reset;
		next_clk := '1';
		bus_clk_div := 0;
		tx_delay := 0;
		-- set tranceiver active flag
		st_active <= '1';
	end procedure tx_start;

	procedure start_byte is
	begin
		if st_tx_done = '1' then
			shift_reg := (others => '0');
			next_byte := next_byte + 1;
		else
			if next_byte = 0 then
				shift_reg := UNSIGNED(cmd_buff_out(7 downto 0));
			else
				shift_reg := UNSIGNED(cmd_buff_out(15 downto 8));
				cmd_buff_addr <= cmd_buff_addr + 1;
			end if;
			next_byte := next_byte + 1;
			if byte_cnt /= 0 then
				byte_cnt := byte_cnt - 1;
			else
				if cmd_loop = '1' then
					-- restart transmission
					tx_reset;
				else
					-- transmit zeros further
					st_tx_done <= '1';
				end if;
			end if;
		end if;
		bit_cnt := 8;
		byte_tx := '1';
	end procedure start_byte;

	impure function get_tx_delay return integer is
	begin
		if cmd_turbo = '1' and st_hdr_done = '1' then
			return TO_INTEGER(UNSIGNED(clk_intervalx));
		else
			return TO_INTEGER(UNSIGNED(clk_interval));
		end if;
	end function get_tx_delay;

begin
	if rising_edge(clk) then

		-- increment command buffer address on buffer write
		if cmd_buff_write = '1' then
			if st_active = '0' then
				cmd_buff_addr <= cmd_buff_addr + 1;
			else
				st_error <= '1';
			end if;
		end if;

		if bus_clk_en = '1' then

			-- checking state consistency
			if byte_tx = '1' and st_active = '0' then
				st_error <= '1';
			end if;

			-- set default values
			bus_bit_latch <= '0';
			bus_byte_completed <= '0';
			next_clk := bus_clk;

			-- divide host clock on 48 and handle delays
			if mhz_clk_div = 0 then
				if rst_delay /= 0 then
					rst_delay := rst_delay - 1;
				end if;
				mhz_clk_div := CLK_MHZ-1;
			else
				mhz_clk_div := mhz_clk_div - 1;
			end if;

			-- handle tx start
			if trg_start = '1' then
				-- check not yet started and the clock is high
				if st_active = '1' or bus_clk /= '1' then
					st_error <= '1';
				end if;
				tx_start;
			end if;

			if run_read = '1' then
				-- check there are no unread data and acquisition completed when we start reading
				if rx_buff_ready = '1' or st_ready = '0' then
					st_error <= '1';
				end if;
			end if;

			-- generate bus clock and data
			if st_active = '1' then
				if st_completed = '1' then
					-- check we are not transmitting the byte and the clock is high
					if byte_tx = '1' or bus_clk /= '1' then
						st_error <= '1';
					end if;
					st_active <= '0';
					cmd_buff_addr <= (others => '0');
				else
					if byte_tx = '0' then
						if tx_delay = 0 and st_pause = '0' then
							start_byte;
						end if;
					else -- byte_tx = '1'
						if bus_clk_div = 0 then
							if bus_clk = '1' then -- falling edge of the bus clock
								bus_output <= shift_reg(0);
								shift_reg := SHIFT_RIGHT(shift_reg, 1);
								bit_cnt := bit_cnt - 1;
							else -- raising edge of the bus clock
								if bit_cnt = 0 then
									byte_tx := '0';
									if cmd_16_bit = '1' and next_byte = 1 then
										tx_delay := 0;
									else
										tx_delay := get_tx_delay;
										if tx_delay = 0 then
											st_error <= '1';
										end if;
									end if;
									bus_byte_completed <= '1';
								end if;
							end if;
							next_clk := not bus_clk;
						end if;
					end if;
				end if;
			end if;

			-- handle stop
			if byte_tx = '1' or next_byte = 1 then
				-- delay stop till byte tx completion
				if ctl_stop = '1' then
					st_stopping <= '1';
				end if;
			else
				-- can stop right now
				if ctl_stop = '1' or st_stopping = '1' then
					-- clear all but error and bus ready bits
					st_tx <= (others => '0');
					cmd_buff_addr <= (others => '0');
				end if;
			end if;

			-- complete bus reset/srq
			if rst_delay = 0 then
				if st_reset = '1' then
					st_reset <= '0';
					bus_srq <= '1';
					if st_hold = '1' then
						rst_delay := TO_INTEGER(UNSIGNED(rst_hold));
						if rst_delay = 0 then
							st_error <= '1';
						end if;
					end if;
				elsif st_hold = '1' then
					st_hold <= '0';
					next_clk := '1';
				end if;
			end if;

			-- handle bus reset/sqr
			if ctl_reset = '1' or trg_srq = '1' then
				initialize;
				if ctl_reset = '1' then
					-- hard reset
					st_hold <= '1';
					st_error <= '0';
					next_clk := '0';
				else
					-- srq
					next_clk := '1';
				end if;
				st_reset <= '1';
				rst_delay := TO_INTEGER(UNSIGNED(rst_time));
				if rst_delay = 0 then
					st_error <= '1';
				end if;
			end if;

			-- maintain bus clock
			bus_clk <= next_clk;
			if bus_clk_div = 0 then
				bus_clk_div := TO_INTEGER(UNSIGNED(clk_div));
				if tx_delay /= 0 then
					tx_delay := tx_delay - 1;
				end if;	
			else
				bus_clk_div := bus_clk_div - 1;
			end if;

			-- set bit latch flag
			if byte_tx = '1' and bus_clk_div = 0 and next_clk = '0' then
				bus_bit_latch <= '1';
			end if;

		end if; -- bus_clk_en = '1'
	end if; -- rising_edge(clk)
end process;

end abstract;
