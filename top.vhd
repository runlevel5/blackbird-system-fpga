library IEEE;
use IEEE.numeric_std.ALL;
use IEEE.STD_LOGIC_1164.ALL;
library machxo2;
use machxo2.all;

entity FPGA_TOP is
	Port (	SYSEN : in STD_LOGIC;
			SYSGOOD: out STD_LOGIC := '0';
			DEBUG_IN: in STD_LOGIC;
			
			--DD1 temp fix for VCS overcurrent bug
			SEQ_CONT: in STD_LOGIC;

			
			--Enable outputs
			VDDA_EN: out STD_LOGIC := '0';
			VDDB_EN: out STD_LOGIC := '0';
			VCSA_EN: out STD_LOGIC := '0';
			VCSB_EN: out STD_LOGIC := '0';
			VDNA_EN: out STD_LOGIC := '0';
			VDNB_EN: out STD_LOGIC := '0';
			VIOA_EN: out STD_LOGIC := '0';
			VIOB_EN: out STD_LOGIC := '0';
			VPPAB_EN: out STD_LOGIC := '0';
			VPPCD_EN: out STD_LOGIC := '0';
			VDDRAB_EN: out STD_LOGIC := '0';
			VTTAB_EN: out STD_LOGIC := '0';
			VDDRCD_EN: out STD_LOGIC := '0';
			VTTCD_EN: out STD_LOGIC := '0';
			AVDD_EN: out STD_LOGIC := '0';
			MISCIO_EN: out STD_LOGIC := '0';
			ATX_EN: out STD_LOGIC := '1';
			
			--Power Good inputs
			VDDA_PG: in STD_LOGIC;
			VDDB_PG: in STD_LOGIC;
			VCSA_PG: in STD_LOGIC;
			VCSB_PG: in STD_LOGIC;
			VDNA_PG: in STD_LOGIC;
			VDNB_PG: in STD_LOGIC;
			VIOA_PG: in STD_LOGIC;
			VIOB_PG: in STD_LOGIC;
			VPPAB_PG: in STD_LOGIC;
			VPPCD_PG: in STD_LOGIC;
			VDDRAB_PG: in STD_LOGIC;
			VDDRCD_PG: in STD_LOGIC;
			AVDD_PG: in STD_LOGIC;
			MISCIO_PG: in STD_LOGIC;
			ATX_PG: in STD_LOGIC;
			BMC_VR_PG: in STD_LOGIC;
			
			--i2c
			SCL: inout STD_LOGIC;
			SDA: inout STD_LOGIC;
			
			--CLK_IN: in STD_LOGIC;
			
			--Second CPU Present Detection
			CPUB_PRESENT_N: in STD_LOGIC;
			CPUB_CLK_OEA: out STD_LOGIC;
			CPUB_CLK_OEB: out STD_LOGIC;
			
			-- Resets
			LPC_RST: out STD_LOGIC := '0';
			BMC_SOFTWARE_PG: in STD_LOGIC;
			BMC_RST: out STD_LOGIC := '0';
			FAN_RST: out STD_LOGIC := '0';
			USBHUB_RST: out STD_LOGIC := '0';
			CPU_STBY_RST: out STD_LOGIC := '0';
			
			-- Reserved for Future Use
			DUAL_5V_CTRL: out STD_LOGIC := '0';
			--DUAL_5V_CTRL: in STD_LOGIC;
			WINDOW_OPEN_N: out STD_LOGIC := '0'
			);
			
						
end FPGA_TOP;

architecture Behavioral of FPGA_TOP is
	--FUTURE update version
	constant cpld_version : STD_LOGIC_VECTOR (7 downto 0) := "00000101";
	constant rail_size : integer := 15;
	signal EN_BUF : STD_LOGIC_VECTOR(rail_size-1 downto 0) := (others => '0');
	signal PG_BUF : STD_LOGIC_VECTOR(rail_size-1 downto 0);
	signal SYSGOOD_BUF : STD_LOGIC;
--	signal CLK_DIV : STD_LOGIC;
--	signal CLK_REG : UNSIGNED(2 downto 0) := "000";
	signal CLK_IN: STD_LOGIC;
	signal stdby_sed: STD_LOGIC;
	signal SYSEN_BUF: STD_LOGIC;
--	signal DEBUG_BUF: STD_LOGIC;
	constant railarray_0 : STD_LOGIC_VECTOR(rail_size-1 downto 0) := (others =>'0');
	constant railarray_1 : STD_LOGIC_VECTOR(rail_size-1 downto 0) := (others =>'1');
	
	-- synchronizing signals
	signal PG_S1 : STD_LOGIC_VECTOR(rail_size-1 downto 0) := (others => '0');
	signal PG_S2 : STD_LOGIC_VECTOR(rail_size-1 downto 0) := (others => '0');
	signal SYSEN_S1: STD_LOGIC := '0';
	signal SYSEN_S2: STD_LOGIC := '0';
	signal SEQ_S1: STD_LOGIC := '1';
	signal SEQ_S2: STD_LOGIC := '1';
	
	-- Timer (Watchdog and Delay) signals
	signal DELAY_DONE : STD_LOGIC_VECTOR(rail_size-1 downto 0) := (others => '0');
	signal W_COUNT : UNSIGNED(23 downto 0) := (others=> '0');
	signal D_COUNT : UNSIGNED(16 downto 0) := (others=> '0');
	-- at 4.16MHz, W_COUNT(23) being one means approximately 100ms have passed, good for checking watchdog between EN and PG
	-- D_COUNT(16) being one means approximately 15ms have passed, good enough for delay betwen One rail and the next
	signal WAIT_ERR : STD_LOGIC := '0';
	signal OPERATION_ERR : STD_LOGIC := '0';
	signal ERR_FOUND :STD_LOGIC := '0';
--	signal FIRST_DELAY : STD_LOGIC := '1';
	signal CLEAR_ERR : STD_LOGIC := '0';
	
		--i2c signals	
	signal i2c_read_req : STD_LOGIC;
	signal i2c_data_to_master : STD_LOGIC_VECTOR(7 downto 0) := "00000000";
	signal i2c_data_from_master : STD_LOGIC_VECTOR(7 downto 0);
	signal i2c_data_valid : STD_LOGIC;
	signal i2c_rst : STD_LOGIC := '0';
	signal i2c_reg_cur : UNSIGNED(7 downto 0) := "00000000";
	constant i2c_addr : STD_LOGIC_VECTOR (6 downto 0) := "0110001";
	constant i2c_clr_err_addr : UNSIGNED (7 downto 0) := "00000011";
	constant i2c_pg_reg_addr1 : UNSIGNED (7 downto 0) := "00000101";
	constant i2c_pg_reg_addr2 : UNSIGNED (7 downto 0) := i2c_pg_reg_addr1 + 1;
	constant i2c_status_reg_addr : UNSIGNED (7 downto 0) := i2c_pg_reg_addr2 + 1;
	constant i2c_version_reg_addr: UNSIGNED (7 downto 0) := "00000000";
	signal i2c_pg_reg : STD_LOGIC_VECTOR(15 downto 0) := (others=>'0');
	signal i2c_clr_err : STD_LOGIC := '0';
	
	component OSCH
	-- synthesis translate_off
		generic (NOM_FREQ: string := "4.16");
	-- synthesis translate_on
		PORT ( STDBY : in STD_LOGIC;
				OSC: out STD_LOGIC;
				SEDSTDBY: out STD_LOGIC);
	end component;
	attribute NOM_FREQ : string;
	attribute NOM_FREQ of OSC1 : label is "4.16";
	
	
	
begin
 OSC1: OSCH
	-- synthesis translate_off
	generic map( NOM_FREQ => "4.16")
	-- synthesis translate_on
	port map ( STDBY => '0', OSC=>CLK_IN, SEDSTDBY => stdby_sed);
	
	--debug
	--CLEAR_ERR <= DUAL_5V_CTRL;
	
	--Divide input 33MHz clock down to 4.125MHz
--	process (CLK_IN)
--	begin
--		if (rising_edge(CLK_IN)) then
--			CLK_REG <= CLK_REG + 1;
--		end if;
--	end process;
--	CLK_DIV <= STD_LOGIC(CLK_REG(2));
	
	-- Power Sequencer Instance
-- SEQ1: entity work.pwrseq 
	-- generic map(rail_size)
	-- port map(
		-- EN => EN_BUF,
		-- PGOOD_A => PG_BUF,
		-- SYSEN_A => SYSEN_BUF,
		-- SYSGOOD => SYSGOOD_BUF,
		-- SCL => SCL,
		-- SDA => SDA,
		-- CLK_IN => CLK_DIV
		-- CLK_IN => CLK_IN
	-- );
	
	--I2C device
I2C_SLAVE: entity work.I2C_slave generic map(i2c_addr) port map(
	SCL, SDA, CLK_IN, i2c_rst,
	i2c_read_req,i2c_data_to_master,
	i2c_data_valid,i2c_data_from_master);
	
	i2c_rst <= '0';
	
--Handle I2C
		--2 8-bit registers with PGOOD state on error
	process (CLK_IN)
	begin
		if (rising_edge(CLK_IN)) then
			i2c_clr_err <= '0';
			if i2c_data_valid = '1' then
				-- data from master is register to be read
				i2c_reg_cur <= unsigned(i2c_data_from_master);
				
				--pulse clear err signal if i2c master reads register 0x03
				if(unsigned(i2c_data_from_master) = i2c_clr_err_addr) then
					i2c_clr_err <= '1';
				end if;
			elsif i2c_read_req = '1' then
				i2c_reg_cur <= i2c_reg_cur + 1;
			end if;
			
			case i2c_reg_cur is
				when i2c_clr_err_addr =>
					i2c_data_to_master <= "11111111";
				when i2c_pg_reg_addr1 =>
					i2c_data_to_master <= i2c_pg_reg(15 downto 8);
				when i2c_pg_reg_addr2 =>
					i2c_data_to_master <= i2c_pg_reg(7 downto 0);
				when i2c_status_reg_addr =>
					--FUTURE add CPU1 Present detect
					i2c_data_to_master <= "000" & WAIT_ERR & OPERATION_ERR & ERR_FOUND & SYSEN_BUF & SYSGOOD_BUF;
				when i2c_version_reg_addr =>
					i2c_data_to_master <= cpld_version;
				when others =>
					i2c_data_to_master <= "00000000";
			end case;
		end if;
	end process;
	
	process (CLK_IN)
	begin
		if (rising_edge(CLK_IN)) then
			PG_S1 <= PG_BUF;
			PG_S2 <= PG_S1;
			SYSEN_S1 <= SYSEN_BUF;
			SYSEN_S2 <= SYSEN_S1;
			SEQ_S1 <= SEQ_CONT;
			SEQ_S2 <= SEQ_S1;
			if(CLEAR_ERR = '1') then
				WAIT_ERR <= '0';
				OPERATION_ERR <= '0';
				ERR_FOUND <= '0';
				W_COUNT <= (others => '0');
				D_COUNT <= (others => '0');
			elsif(SYSEN_S2 = '0' or ERR_FOUND = '1') then
				W_COUNT <= (others => '0');
				D_COUNT <= (others => '0');
				DELAY_DONE <= (others => '0');
			elsif (PG_S2(0) = '1' and EN_BUF(0) = '1' and DELAY_DONE(0) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(0) <= '1';
				end if;
			elsif (PG_S2(1) = '1' and EN_BUF(1) = '1' and DELAY_DONE(1) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(1) <= '1';
				end if;
			elsif (PG_S2(2) = '1' and EN_BUF(2) = '1' and DELAY_DONE(2) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(2) <= '1';
				end if;
			elsif (PG_S2(3) = '1' and EN_BUF(3) = '1' and DELAY_DONE(3) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(3) <= '1';
				end if;
			elsif (PG_S2(4) = '1' and EN_BUF(4) = '1' and DELAY_DONE(4) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(4) <= '1';
				end if;
			elsif (PG_S2(5) = '1' and EN_BUF(5) = '1' and DELAY_DONE(5) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(5) <= '1';
				end if;
			elsif (PG_S2(6) = '1' and EN_BUF(6) = '1' and DELAY_DONE(6) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(6) <= '1';
				end if;
			elsif (PG_S2(7) = '1' and EN_BUF(7) = '1' and DELAY_DONE(7) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(7) <= '1';
				end if;
			elsif (PG_S2(8) = '1' and EN_BUF(8) = '1' and DELAY_DONE(8) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(8) <= '1';
				end if;
			elsif (PG_S2(9) = '1' and EN_BUF(9) = '1' and DELAY_DONE(9) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(9) <= '1';
				end if;
			elsif (PG_S2(10) = '1' and EN_BUF(10) = '1' and DELAY_DONE(10) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(10) <= '1';
				end if;
			elsif (PG_S2(11) = '1' and EN_BUF(11) = '1' and DELAY_DONE(11) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(11) <= '1';
				end if;
			elsif (PG_S2(12) = '1' and EN_BUF(12) = '1' and DELAY_DONE(12) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(12) <= '1';
				end if;
			elsif (PG_S2(13) = '1' and EN_BUF(13) = '1' and DELAY_DONE(13) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(13) <= '1';
				end if;
			elsif (PG_S2(14) = '1' and EN_BUF(14) = '1' and DELAY_DONE(14) = '0') then
				W_COUNT <= (others => '0');
				D_COUNT <= D_COUNT+1;
				if (D_COUNT(16) = '1') then
					D_COUNT <= (others => '0');
					DELAY_DONE(14) <= '1';
				end if;
				
			-- Error Checks
			-- Check time between Enables going high and PGOODs arriving. Error out after 100ms
			elsif (PG_S2(0) = '0' and EN_BUF(0) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if (W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(1) = '0' and EN_BUF(1) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if (W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(2) = '0' and EN_BUF(2) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(3) = '0' and EN_BUF(3) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(4) = '0' and EN_BUF(4) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(5) = '0' and EN_BUF(5) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(6) = '0' and EN_BUF(6) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(7) = '0' and EN_BUF(7) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(8) = '0' and EN_BUF(8) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(9) = '0' and EN_BUF(9) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(10) = '0' and EN_BUF(10) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(11) = '0' and EN_BUF(11) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(12) = '0' and EN_BUF(12) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(13) = '0' and EN_BUF(13) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			elsif (PG_S2(14) = '0' and EN_BUF(14) = '1') then
				
				W_COUNT <= W_COUNT+1;
				if ( W_COUNT(23) = '1') then
					W_COUNT <= (others => '0');
					WAIT_ERR <= '1';
				end if;
			end if;
			
			if(not (DELAY_DONE and not PG_S2) /= railarray_1) then
				OPERATION_ERR <= '1';
			end if;
			
			if ((WAIT_ERR or OPERATION_ERR) = '1' and CLEAR_ERR = '0') then
				ERR_FOUND <= '1';
			else
				i2c_pg_reg(14 downto 0) <= PG_S2(14 downto 0);
			end if;
		end if;
	end process;
	
	
	--Assign Ports to Enables
	ATX_EN <= not EN_BUF(0);
	MISCIO_EN <= EN_BUF(1);
	VDNA_EN <= EN_BUF(2);
	VDNB_EN <= EN_BUF(3) and not CPUB_PRESENT_N;
	AVDD_EN <= EN_BUF(4);
	VIOA_EN <= EN_BUF(5);
	VIOB_EN <= EN_BUF(6) and not CPUB_PRESENT_N;
	VDDA_EN <= EN_BUF(7);
	VDDB_EN <= EN_BUF(8) and not CPUB_PRESENT_N;
	VCSA_EN <= EN_BUF(9);
	VCSB_EN <= EN_BUF(10) and not CPUB_PRESENT_N;
	VPPAB_EN <= EN_BUF(11);
	VPPCD_EN <= EN_BUF(12) and not CPUB_PRESENT_N;
	VDDRAB_EN <= EN_BUF(13);
	VTTAB_EN <= EN_BUF(13);
	VDDRCD_EN <= EN_BUF(14) and not CPUB_PRESENT_N;
	VTTCD_EN <= EN_BUF(14) and not CPUB_PRESENT_N;
	
	-- Assign Ports to PGood buffer
	PG_BUF(0) <= ATX_PG;
	PG_BUF(1) <= MISCIO_PG;
	PG_BUF(2) <= VDNA_PG;
	PG_BUF(3) <= VDNB_PG or (CPUB_PRESENT_N and EN_BUF(3));
	PG_BUF(4) <= AVDD_PG;
	PG_BUF(5) <= VIOA_PG;
	PG_BUF(6) <= VIOB_PG or (CPUB_PRESENT_N and EN_BUF(6));
	PG_BUF(7) <= VDDA_PG;
	PG_BUF(8) <= VDDB_PG or (CPUB_PRESENT_N and EN_BUF(8));
	PG_BUF(9) <= VCSA_PG;
	PG_BUF(10) <= VCSB_PG or (CPUB_PRESENT_N and EN_BUF(10));
	PG_BUF(11) <= VPPAB_PG;
	PG_BUF(12) <= VPPCD_PG or (CPUB_PRESENT_N and EN_BUF(12));
	PG_BUF(13) <= VDDRAB_PG;
	PG_BUF(14) <= VDDRCD_PG or (CPUB_PRESENT_N and EN_BUF(14));
	
	--Enable outputs
	 -- Shut everything off if ann error has occurred
	 -- Otherwise, if system enable is up, then enable short delay is done after previous rail
	 -- Otherwise, disable after next rail goes down
	EN_BUF(0) <= (SYSEN_S2 or PG_S2(1)) and not ERR_FOUND;
	EN_BUF(1) <= ((SYSEN_S2 and DELAY_DONE(0)) or PG_S2(2)) and not ERR_FOUND;
	EN_BUF(2) <= ((SYSEN_S2 and DELAY_DONE(1)) or PG_S2(3)) and not ERR_FOUND;
	EN_BUF(3) <= ((SYSEN_S2 and DELAY_DONE(2)) or PG_S2(4)) and not ERR_FOUND;
	EN_BUF(4) <= ((SYSEN_S2 and DELAY_DONE(3)) or PG_S2(5)) and not ERR_FOUND;
	EN_BUF(5) <= ((SYSEN_S2 and DELAY_DONE(4)) or PG_S2(6)) and not ERR_FOUND;
	EN_BUF(6) <= ((SYSEN_S2 and DELAY_DONE(5)) or PG_S2(7)) and not ERR_FOUND;
	EN_BUF(7) <= ((SYSEN_S2 and DELAY_DONE(6)) or PG_S2(8)) and not ERR_FOUND;
	EN_BUF(8) <= ((SYSEN_S2 and DELAY_DONE(7)) or PG_S2(9)) and not ERR_FOUND;
	EN_BUF(9) <= ((not SEQ_S2 and SYSEN_S2 and DELAY_DONE(8)) or PG_S2(10)) and not ERR_FOUND;
	EN_BUF(10) <= ((SYSEN_S2 and DELAY_DONE(9)) or PG_S2(11)) and not ERR_FOUND;
	EN_BUF(11) <= ((SYSEN_S2 and DELAY_DONE(10)) or PG_S2(12)) and not ERR_FOUND;
	EN_BUF(12) <= ((SYSEN_S2 and DELAY_DONE(11)) or PG_S2(13)) and not ERR_FOUND;
	EN_BUF(13) <= ((SYSEN_S2 and DELAY_DONE(12)) or PG_S2(14)) and not ERR_FOUND;
	EN_BUF(14) <= (SYSEN_S2 and DELAY_DONE(13)) and not ERR_FOUND;
	
	--ERR state reset
	CLEAR_ERR <= i2c_clr_err;

	
	-- CPUB clk enables
	CPUB_CLK_OEA <= not CPUB_PRESENT_N;
	CPUB_CLK_OEB <= not CPUB_PRESENT_N;
	
	-- System PWRGOOD
	SYSGOOD_BUF <= DELAY_DONE(14);
	SYSGOOD <= SYSGOOD_BUF and BMC_SOFTWARE_PG;
	LPC_RST <= SYSGOOD_BUF;
	
	-- CPU Reset
	CPU_STBY_RST <= EN_BUF(0);
	
	-- BMC RESETs
	BMC_RST <= BMC_VR_PG;
	USBHUB_RST <= SYSGOOD_BUF and BMC_SOFTWARE_PG;
	FAN_RST <= not BMC_VR_PG;
	
	-- DEBUG_IN override allows non-BMC control of CPLD
	SYSEN_BUF <= SYSEN or not DEBUG_IN;
	--SYSEN_BUF <= not DEBUG_IN;
	

	
	
end Behavioral;

