// Copyright © 2017 Raptor Engineering, LLC
// Copyright © 2017, International Business Machines Corp.
// All Rights Reserved
//
// See LICENSE file for licensing details

module FPGA_TOP
	(
		// General I/O
		input wire SYSEN,
		output wire SYSGOOD,
		input wire DEBUG_IN,

		// DD1 temp fix for VCS overcurrent bug
		input wire SEQ_CONT,

		// Enable outputs
		output wire VDDA_EN,
		output wire VDDB_EN,
		output wire VCSA_EN,
		output wire VCSB_EN,
		output wire VDNA_EN,
		output wire VDNB_EN,
		output wire VIOA_EN,
		output wire VIOB_EN,
		output wire VPPAB_EN,
		output wire VPPCD_EN,
		output wire VDDRAB_EN,
		output wire VTTAB_EN,
		output wire VDDRCD_EN,
		output wire VTTCD_EN,
		output wire AVDD_EN,
		output wire MISCIO_EN,
		output wire ATX_EN,

		// Power Good inputs
		input wire VDDA_PG,
		input wire VDDB_PG,
		input wire VCSA_PG,
		input wire VCSB_PG,
		input wire VDNA_PG,
		input wire VDNB_PG,
		input wire VIOA_PG,
		input wire VIOB_PG,
		input wire VPPAB_PG,
		input wire VPPCD_PG,
		input wire VDDRAB_PG,
		input wire VDDRCD_PG,
		input wire AVDD_PG,
		input wire MISCIO_PG,
		input wire ATX_PG,
		input wire BMC_VR_PG,

		// I2C
		inout wire SCL,
		inout wire SDA,

		// Second CPU Present Detection
		input wire CPUB_PRESENT_N,
		output wire CPUB_CLK_OEA,
		output wire CPUB_CLK_OEB,

		// Resets
		output wire LPC_RST,
		input wire BMC_SOFTWARE_PG,
		output wire BMC_RST,
		output wire FAN_RST,
		output wire USBHUB_RST,
		output wire CPU_STBY_RST,

		// Reserved for Future Use
		output wire DUAL_5V_CTRL,
		output wire WINDOW_OPEN_N
	);

	// FUTURE update version
	parameter cpld_version = 8'b00000110;
	parameter rail_size = 15;
	wire [rail_size - 1:0] EN_BUF = 1'b0;
	wire [rail_size - 1:0] PG_BUF;
	wire SYSGOOD_BUF;
	wire CLK_IN;
	wire stdby_sed;
	wire SYSEN_BUF;
	parameter railarray_0 = 1'b0;
	parameter railarray_1 = 1'b1; 	// synchronizing signals
	reg [rail_size - 1:0] PG_S1 = 1'b0;
	reg [rail_size - 1:0] PG_S2 = 1'b0;
	reg SYSEN_S1 = 1'b0;
	reg SYSEN_S2 = 1'b0;
	reg SEQ_S1 = 1'b1;
	reg SEQ_S2 = 1'b1;		// Timer (Watchdog and Delay) signals
	reg [rail_size - 1:0] DELAY_DONE = 1'b0;
	reg [23:0] W_COUNT = 1'b0;
	reg [16:0] D_COUNT = 1'b0; 	// at 4.16MHz, W_COUNT(23) being one means approximately 100ms have passed, good for checking watchdog between EN and PG
					// D_COUNT(16) being one means approximately 15ms have passed, good enough for delay betwen One rail and the next
	reg WAIT_ERR = 1'b0;
	reg OPERATION_ERR = 1'b0;
	reg ERR_FOUND = 1'b0;  //	signal FIRST_DELAY : STD_LOGIC := '1';
	wire CLEAR_ERR = 1'b0;  //i2c signals	
	wire i2c_read_req;
	reg [7:0] i2c_data_to_master = 8'b00000000;
	wire [7:0] i2c_data_from_master;
	wire i2c_data_valid;
	wire i2c_rst = 1'b0;
	reg [7:0] i2c_reg_cur = 8'b00000000;
	parameter i2c_addr = 7'b0110001;
	parameter i2c_clr_err_addr = 8'b00000011;
	parameter i2c_pg_reg_addr1 = 8'b00000101;
	parameter i2c_pg_reg_addr2 = i2c_pg_reg_addr1 + 1;
	parameter i2c_status_reg_addr = i2c_pg_reg_addr2 + 1;
	parameter i2c_version_reg_addr = 8'b00000000;
	reg [15:0] i2c_pg_reg = 1'b0;
	reg i2c_clr_err = 1'b0;

	OSCH OSC1(
		.STDBY(1'b0),
		.OSC(CLK_IN),
		.SEDSTDBY(stdby_sed)
	);

	//debug
	//CLEAR_ERR <= DUAL_5V_CTRL;
	//Divide input 33MHz clock down to 4.125MHz
	//	process (CLK_IN)
	//	begin
	//		if (rising_edge(CLK_IN)) then
	//			CLK_REG <= CLK_REG + 1;
	//		end if;
	//	end process;
	//	CLK_DIV <= STD_LOGIC(CLK_REG(2));
	// Power Sequencer Instance
	// SEQ1: entity work.pwrseq 
	// generic map(rail_size)
	// port map(
	// EN => EN_BUF,
	// PGOOD_A => PG_BUF,
	// SYSEN_A => SYSEN_BUF,
	// SYSGOOD => SYSGOOD_BUF,
	// SCL => SCL,
	// SDA => SDA,
	// CLK_IN => CLK_DIV
	// CLK_IN => CLK_IN
	// );
	//I2C device
	I2C_slave #(
	i2c_addr)
	I2C_SLAVE(
	SCL,
	SDA,
	CLK_IN,
	i2c_rst,
	i2c_read_req,
	i2c_data_to_master,
	i2c_data_valid,
	i2c_data_from_master);
	
	assign i2c_rst = 1'b0;
	//Handle I2C
	//2 8-bit registers with PGOOD state on error
	always @(posedge CLK_IN) begin
		i2c_clr_err <= 1'b0;

		if (i2c_data_valid == 1'b1) begin
			// data from master is register to be read
			i2c_reg_cur <= i2c_data_from_master;
	
			//pulse clear err signal if i2c master reads register 0x03
			if (((i2c_data_from_master) == i2c_clr_err_addr)) begin
				i2c_clr_err <= 1'b1;
			end
		end
		else if (i2c_read_req == 1'b1) begin
			i2c_reg_cur <= i2c_reg_cur + 1;
		end
		case(i2c_reg_cur)
			i2c_clr_err_addr : begin
				i2c_data_to_master <= 8'b11111111;
			end
			i2c_pg_reg_addr1 : begin
				i2c_data_to_master <= i2c_pg_reg[15:8];
			end
			i2c_pg_reg_addr2 : begin
				i2c_data_to_master <= i2c_pg_reg[7:0];
			end
			i2c_status_reg_addr : begin
				//FUTURE add CPU1 Present detect
				i2c_data_to_master <= {3'b000,WAIT_ERR,OPERATION_ERR,ERR_FOUND,SYSEN_BUF,SYSGOOD_BUF};
			end
			i2c_version_reg_addr : begin
				i2c_data_to_master <= cpld_version;
			end
			default : begin
				i2c_data_to_master <= 8'b00000000;
			end
		endcase
	end
	
	always @(posedge CLK_IN) begin
		PG_S1 <= PG_BUF;
		PG_S2 <= PG_S1;
		SYSEN_S1 <= SYSEN_BUF;
		SYSEN_S2 <= SYSEN_S1;
		SEQ_S1 <= SEQ_CONT;
		SEQ_S2 <= SEQ_S1;
		if ((CLEAR_ERR == 1'b1)) begin
			WAIT_ERR <= 1'b0;
			OPERATION_ERR <= 1'b0;
			ERR_FOUND <= 1'b0;
			W_COUNT <= {24{1'b0}};
			D_COUNT <= {17{1'b0}};
		end
		else if ((SYSEN_S2 == 1'b0 || ERR_FOUND == 1'b1)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= {17{1'b0}};
			DELAY_DONE <= {(((rail_size - 1))-((0))+1){1'b0}};
		end
		else if ((PG_S2[0] == 1'b1 && EN_BUF[0] == 1'b1 && DELAY_DONE[0] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[0] <= 1'b1;
			end
		end
		else if ((PG_S2[1] == 1'b1 && EN_BUF[1] == 1'b1 && DELAY_DONE[1] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[1] <= 1'b1;
			end
		end
		else if ((PG_S2[2] == 1'b1 && EN_BUF[2] == 1'b1 && DELAY_DONE[2] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[2] <= 1'b1;
			end
		end
		else if ((PG_S2[3] == 1'b1 && EN_BUF[3] == 1'b1 && DELAY_DONE[3] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[3] <= 1'b1;
			end
		end
		else if ((PG_S2[4] == 1'b1 && EN_BUF[4] == 1'b1 && DELAY_DONE[4] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[4] <= 1'b1;
			end
		end
		else if ((PG_S2[5] == 1'b1 && EN_BUF[5] == 1'b1 && DELAY_DONE[5] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[5] <= 1'b1;
			end
		end
		else if ((PG_S2[6] == 1'b1 && EN_BUF[6] == 1'b1 && DELAY_DONE[6] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[6] <= 1'b1;
			end
		end
		else if ((PG_S2[7] == 1'b1 && EN_BUF[7] == 1'b1 && DELAY_DONE[7] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[7] <= 1'b1;
			end
		end
		else if ((PG_S2[8] == 1'b1 && EN_BUF[8] == 1'b1 && DELAY_DONE[8] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[8] <= 1'b1;
			end
		end
		else if ((PG_S2[9] == 1'b1 && EN_BUF[9] == 1'b1 && DELAY_DONE[9] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[9] <= 1'b1;
			end
		end
		else if ((PG_S2[10] == 1'b1 && EN_BUF[10] == 1'b1 && DELAY_DONE[10] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[10] <= 1'b1;
			end
		end
		else if ((PG_S2[11] == 1'b1 && EN_BUF[11] == 1'b1 && DELAY_DONE[11] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[11] <= 1'b1;
			end
		end
		else if ((PG_S2[12] == 1'b1 && EN_BUF[12] == 1'b1 && DELAY_DONE[12] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[12] <= 1'b1;
			end
		end
		else if ((PG_S2[13] == 1'b1 && EN_BUF[13] == 1'b1 && DELAY_DONE[13] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[13] <= 1'b1;
			end
		end
		else if ((PG_S2[14] == 1'b1 && EN_BUF[14] == 1'b1 && DELAY_DONE[14] == 1'b0)) begin
			W_COUNT <= {24{1'b0}};
			D_COUNT <= D_COUNT + 1;
			if ((D_COUNT[16] == 1'b1)) begin
				D_COUNT <= {17{1'b0}};
				DELAY_DONE[14] <= 1'b1;
			end
		end

		// Error Checks
		// Check time between Enables going high and PGOODs arriving. Error out after 100ms
		else if ((PG_S2[0] == 1'b0 && EN_BUF[0] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[1] == 1'b0 && EN_BUF[1] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[2] == 1'b0 && EN_BUF[2] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[3] == 1'b0 && EN_BUF[3] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[4] == 1'b0 && EN_BUF[4] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[5] == 1'b0 && EN_BUF[5] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[6] == 1'b0 && EN_BUF[6] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[7] == 1'b0 && EN_BUF[7] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[8] == 1'b0 && EN_BUF[8] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[9] == 1'b0 && EN_BUF[9] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[10] == 1'b0 && EN_BUF[10] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[11] == 1'b0 && EN_BUF[11] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[12] == 1'b0 && EN_BUF[12] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[13] == 1'b0 && EN_BUF[13] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		else if ((PG_S2[14] == 1'b0 && EN_BUF[14] == 1'b1)) begin
			W_COUNT <= W_COUNT + 1;
			if ((W_COUNT[23] == 1'b1)) begin
				W_COUNT <= {24{1'b0}};
				WAIT_ERR <= 1'b1;
			end
		end
		if ((( ~(DELAY_DONE & ~PG_S2)) != railarray_1)) begin
			OPERATION_ERR <= 1'b1;
		end
		if (((WAIT_ERR | OPERATION_ERR) == 1'b1 && CLEAR_ERR == 1'b0)) begin
			ERR_FOUND <= 1'b1;
		end else begin
			i2c_pg_reg[14:0] <= PG_S2[14:0];
		end
	end
	
	// Assign Ports to Enables
	assign ATX_EN = ~EN_BUF[0];
	assign MISCIO_EN = EN_BUF[1];
	assign VDNA_EN = EN_BUF[2];
	assign VDNB_EN = EN_BUF[3] & ~CPUB_PRESENT_N;
	assign AVDD_EN = EN_BUF[4];
	assign VIOA_EN = EN_BUF[5];
	assign VIOB_EN = EN_BUF[6] & ~CPUB_PRESENT_N;
	assign VDDA_EN = EN_BUF[7];
	assign VDDB_EN = EN_BUF[8] & ~CPUB_PRESENT_N;
	assign VCSA_EN = EN_BUF[9];
	assign VCSB_EN = EN_BUF[10] & ~CPUB_PRESENT_N;
	assign VPPAB_EN = EN_BUF[11];
	assign VPPCD_EN = EN_BUF[12] & ~CPUB_PRESENT_N;
	assign VDDRAB_EN = EN_BUF[13];
	assign VTTAB_EN = EN_BUF[13];
	assign VDDRCD_EN = EN_BUF[14] & ~CPUB_PRESENT_N;
	assign VTTCD_EN = EN_BUF[14] & ~CPUB_PRESENT_N;

	// Assign Ports to PGood buffer
	assign PG_BUF[0] = ATX_PG;
	assign PG_BUF[1] = MISCIO_PG;
	assign PG_BUF[2] = VDNA_PG;
	assign PG_BUF[3] = VDNB_PG | (CPUB_PRESENT_N & EN_BUF[3]);
	assign PG_BUF[4] = AVDD_PG;
	assign PG_BUF[5] = VIOA_PG;
	assign PG_BUF[6] = VIOB_PG | (CPUB_PRESENT_N & EN_BUF[6]);
	assign PG_BUF[7] = VDDA_PG;
	assign PG_BUF[8] = VDDB_PG | (CPUB_PRESENT_N & EN_BUF[8]);
	assign PG_BUF[9] = VCSA_PG;
	assign PG_BUF[10] = VCSB_PG | (CPUB_PRESENT_N & EN_BUF[10]);
	assign PG_BUF[11] = VPPAB_PG;
	assign PG_BUF[12] = VPPCD_PG | (CPUB_PRESENT_N & EN_BUF[12]);
	assign PG_BUF[13] = VDDRAB_PG;
	assign PG_BUF[14] = VDDRCD_PG | (CPUB_PRESENT_N & EN_BUF[14]);

	// Enable outputs
	// Shut everything off if ann error has occurred
	// Otherwise, if system enable is up, then enable short delay is done after previous rail
	// Otherwise, disable after next rail goes down
	assign EN_BUF[0] = (SYSEN_S2 | PG_S2[1]) & ~ERR_FOUND;
	assign EN_BUF[1] = ((SYSEN_S2 & DELAY_DONE[0]) | PG_S2[2]) & ~ERR_FOUND;
	assign EN_BUF[2] = ((SYSEN_S2 & DELAY_DONE[1]) | PG_S2[3]) & ~ERR_FOUND;
	assign EN_BUF[3] = ((SYSEN_S2 & DELAY_DONE[2]) | PG_S2[4]) & ~ERR_FOUND;
	assign EN_BUF[4] = ((SYSEN_S2 & DELAY_DONE[ + 1]) | PG_S2[ + 1]) & ~ERR_FOUND;
	assign EN_BUF[5] = ((SYSEN_S2 & DELAY_DONE[4]) | PG_S2[6]) & ~ERR_FOUND;
	assign EN_BUF[6] = ((SYSEN_S2 & DELAY_DONE[5]) | PG_S2[7]) & ~ERR_FOUND;
	assign EN_BUF[7] = ((SYSEN_S2 & DELAY_DONE[6]) | PG_S2[8]) & ~ERR_FOUND;
	assign EN_BUF[8] = ((SYSEN_S2 & DELAY_DONE[7]) | PG_S2[9]) & ~ERR_FOUND;
	assign EN_BUF[9] = (( ~SEQ_S2 & SYSEN_S2 & DELAY_DONE[8]) | PG_S2[10]) & ~ERR_FOUND;
	assign EN_BUF[10] = ((SYSEN_S2 & DELAY_DONE[9]) | PG_S2[11]) & ~ERR_FOUND;
	assign EN_BUF[11] = ((SYSEN_S2 & DELAY_DONE[10]) | PG_S2[12]) & ~ERR_FOUND;
	assign EN_BUF[12] = ((SYSEN_S2 & DELAY_DONE[11]) | PG_S2[13]) & ~ERR_FOUND;
	assign EN_BUF[13] = ((SYSEN_S2 & DELAY_DONE[12]) | PG_S2[14]) & ~ERR_FOUND;
	assign EN_BUF[14] = (SYSEN_S2 & DELAY_DONE[13]) & ~ERR_FOUND;

	//ERR state reset
	assign CLEAR_ERR = i2c_clr_err;

	// CPUB clk enables
	assign CPUB_CLK_OEA = ~CPUB_PRESENT_N;
	assign CPUB_CLK_OEB = ~CPUB_PRESENT_N;

	// System PWRGOOD
	assign SYSGOOD_BUF = DELAY_DONE[14];
	assign SYSGOOD = SYSGOOD_BUF & BMC_SOFTWARE_PG;
	assign LPC_RST = SYSGOOD_BUF;

	// CPU Reset
	assign CPU_STBY_RST = EN_BUF[0];

	// BMC RESETs
	assign BMC_RST = BMC_VR_PG;
	assign USBHUB_RST = SYSGOOD_BUF & BMC_SOFTWARE_PG;
	assign FAN_RST = ~BMC_VR_PG;

	// DEBUG_IN override allows non-BMC control of CPLD
	assign SYSEN_BUF = SYSEN | ~DEBUG_IN;
	// assign SYSEN_BUF = ~DEBUG_IN;
	
endmodule