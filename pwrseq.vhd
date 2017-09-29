library IEEE;
use IEEE.numeric_std.ALL;
use IEEE.STD_LOGIC_1164.ALL;

entity pwrseq is
	generic (rail_size : integer);
	Port (EN : out STD_LOGIC_VECTOR (rail_size-1 downto 0) := (others=>'0');
			PGOOD_A : in STD_LOGIC_VECTOR (rail_size-1 downto 0);
			SYSEN_A : in STD_LOGIC;
			SYSGOOD: out STD_LOGIC := '0';
			SCL: inout STD_LOGIC;
			SDA: inout STD_LOGIC;
			CLK_IN: in STD_LOGIC);
end pwrseq;

architecture Behavioral of pwrseq is
	--Input output buffers and synchronizers
	--constant rail_size : integer := EN'LENGTH;
	signal EN_BUF : STD_LOGIC_VECTOR (rail_size-1 downto 0) := (others => '0');
	signal PGOOD : STD_LOGIC_VECTOR (rail_size-1 downto 0) := (others => '0');
	signal PGOOD_S : STD_LOGIC_VECTOR (rail_size-1 downto 0) := (others => '0');
	signal SYSEN : STD_LOGIC := '0';
	signal SYSEN_S : STD_LOGIC := '0';
	signal SYSGOOD_BUF : STD_LOGIC := '0';
	
	--Sequencer State Machine	
	type state_type is (idleon,shifton,waitpgood,waiten,idleoff,shiftoff,waitoff);
	signal state : state_type := idleoff;
	signal ERR : STD_LOGIC := '0';
	signal err_msg : STD_LOGIC_VECTOR (15 downto 0) := (others => '1');
	constant all_on : STD_LOGIC_VECTOR (rail_size-1 downto 0) := (others=>'1');
	constant all_off : STD_LOGIC_VECTOR (rail_size-1 downto 0) := (others =>'0');
	
	--Clocks and timers
	constant counter_size : integer := 20;
	signal T_COUNT : UNSIGNED (counter_size-1 downto 0);
	--t_max_wait is max delay from Enable assert to Pgood assert (200 ms assumption 4.125MHz clk)
	constant t_max_wait : UNSIGNED (counter_size-1 downto 0) := TO_UNSIGNED(825000,counter_size);
	--t_delay is delay from Pgood assert to next Enable assert (1ms assumption 4.125MHz clk)
	constant t_delay : UNSIGNED (counter_size-1 downto 0) := TO_UNSIGNED(4125,counter_size);
	
	--i2c signals	
	signal i2c_read_req : STD_LOGIC;
	signal i2c_data_to_master : STD_LOGIC_VECTOR(7 downto 0) := "00000000";
	signal i2c_data_from_master : STD_LOGIC_VECTOR(7 downto 0);
	signal i2c_data_valid : STD_LOGIC;
	signal i2c_rst : STD_LOGIC := '0';
	signal i2c_reg_cur : UNSIGNED(7 downto 0) := "00000000";
	constant i2c_addr : STD_LOGIC_VECTOR (6 downto 0) := "0110001";
	constant i2c_pg_reg_addr1 : UNSIGNED (7 downto 0) := "00000001";
	constant i2c_pg_reg_addr2 : UNSIGNED (7 downto 0) := i2c_pg_reg_addr1 + 1;
	constant i2c_status_reg_addr : UNSIGNED (7 downto 0) := i2c_pg_reg_addr2 + 1;
	
	
begin


--I2C device
I2C_SLAVE: entity work.I2C_slave generic map(i2c_addr) port map(
	SCL, SDA, CLK_IN, i2c_rst,
	i2c_read_req,i2c_data_to_master,
	i2c_data_valid,i2c_data_from_master);
	
--Handle I2C
		--2 8-bit registers with PGOOD state on error
	process (CLK_IN)
	begin
		if (rising_edge(CLK_IN)) then
			--return high byte with any memory address, loop on any consecutive reads
			if i2c_data_valid = '1' then
				i2c_reg_cur <= unsigned(i2c_data_from_master);
			elsif i2c_read_req = '1' then
				i2c_reg_cur <= i2c_reg_cur + 1;
			end if;
			
			case i2c_reg_cur is
				when i2c_pg_reg_addr1 =>
					i2c_data_to_master <= err_msg(15 downto 8);
				when i2c_pg_reg_addr2 =>
					i2c_data_to_master <= err_msg(7 downto 0);
				when i2c_status_reg_addr =>
					i2c_data_to_master <= "000000" & SYSEN & SYSGOOD_BUF;
				when others =>
					i2c_data_to_master <= "00000000";
			end case;
		end if;
	end process;

-- Power Sequencer state machine
	process (CLK_IN)
	begin
		if rising_edge(CLK_IN) then
		
	
		-- Increase counter	
			T_COUNT <= T_COUNT + 1;
		
		-- Synchronize Asynchronous inputs to clock
			PGOOD_S <= PGOOD_A;
			PGOOD <= PGOOD_S;
			SYSEN_S <= SYSEN_A;
			SYSEN <= SYSEN_S;
		-- Decide next state
			case state is
				when idleoff=>
					--Only leave idle off if system enable active and no error
					if ERR = '1' then
						state <= idleoff;
					elsif SYSEN = '1' then
						state <= shifton;
					else
						state <= idleoff;
					end if;
				when shifton=>
					-- enable next power rail, reset counter, wait for pgood
					EN_BUF(rail_size-1 downto 1) <= EN_BUF(rail_size-2 downto 0);
					EN_BUF(0) <= '1';
					T_COUNT <= (others=>'0');
					state <= waitpgood;
				when waitpgood=>
					-- Wait for enabled power rail's PGOOD, after time with no pgood, error
					if T_COUNT > t_max_wait then
						ERR <= '1';
						err_msg <= (others =>'0');
						err_msg(rail_size-1 downto 0) <= EN_BUF and PGOOD;
						state <= shiftoff;

					elsif (EN_BUF and PGOOD) = all_on then
						state <=idleon;

					elsif ((EN_BUF and PGOOD) = EN_BUF) then
						T_COUNT <= (others => '0');
						state <= waiten;
					else
						state <= waitpgood;
					end if;
				-- delay between last pgood and next enable
				when waiten=>
					if T_COUNT > t_delay then
						T_COUNT <= (others => '0');
						state <= shifton;
						
					else
						state <= waiten;
					end if;
					
				-- stay in idle on unless power rail goes down (error) or system enable removed
				when idleon=>
					SYSGOOD_BUF <= '1';
					if (not(PGOOD = all_on)) then
						ERR <= '1';
						err_msg <= (others =>'0');
						err_msg(rail_size-1 downto 0) <= PGOOD;
					end if;
					if ((SYSEN = '0') or (ERR = '1')) then
						SYSGOOD_BUF <= '0';
						state <= shiftoff;
						
					else
						state <= idleon;
					end if;
				-- Turn off enable for next power rail
				when shiftoff=>
					EN_BUF(rail_size-2 downto 0) <= EN_BUF(rail_size-1 downto 1);
					EN_BUF(rail_size-1) <= '0';
					if(EN_BUF=all_off) then
						state <= idleoff;
					
					else
						T_COUNT <= (others => '0');
						state <= waitoff;
					end if;
				-- in controlled shutdown, delay between disabling power rails
				when waitoff=>
					if ERR = '1' then
						state <= shiftoff;
						--LED_BUF <= "10";
					elsif T_COUNT > t_delay then
						state <= shiftoff;
						--LED_BUF <= "10";
					else
						state <= waitoff;
					end if;	
			end case;
		end if;
	end process;


-- Output enable buffer to pins
	EN <= not(EN_BUF);
	i2c_rst <= '0';
	SYSGOOD <=SYSGOOD_BUF;
	
	
end Behavioral;

