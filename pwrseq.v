// Copyright © 2017 Raptor Engineering, LLC
// Copyright © 2017, International Business Machines Corp.
// All Rights Reserved
//
// See LICENSE file for licensing details

module pwrseq(
		output wire [rail_size - 1:0] EN,
		input wire [rail_size - 1:0] PGOOD_A,
		input wire SYSEN_A,
		output wire SYSGOOD,
		inout wire SCL,
		inout wire SDA,
		input wire CLK_IN
	);

	parameter [31:0] rail_size;

	// Input output buffers and synchronizers
	reg [rail_size - 1:0] EN_BUF = 1'b0;
	reg [rail_size - 1:0] PGOOD = 1'b0;
	reg [rail_size - 1:0] PGOOD_S = 1'b0;
	reg SYSEN = 1'b0;
	reg SYSEN_S = 1'b0;
	reg SYSGOOD_BUF = 1'b0;

	// Sequencer State Machine
	parameter [2:0] idleon = 0;
	parameter [2:0] shifton = 1;
	parameter [2:0] waitpgood = 2;
	parameter [2:0] waiten = 3;
	parameter [2:0] idleoff = 4;
	parameter [2:0] shiftoff = 5;
	parameter [2:0] waitoff = 6;

	reg [2:0] state = idleoff;
	reg ERR = 1'b0;
	reg [15:0] err_msg = 1'b1;
	parameter all_on = 1'b1;
	parameter all_off = 1'b0;

	// Clocks and timers
	parameter counter_size = 20;
	reg [counter_size - 1:0] T_COUNT;

	// t_max_wait is max delay from Enable assert to Pgood assert (200 ms assumption 4.125MHz clk)
	parameter t_max_wait = 825000;

	// t_delay is delay from Pgood assert to next Enable assert (1ms assumption 4.125MHz clk)
	parameter t_delay = 4125;

	//I2C signals
	wire i2c_read_req;
	reg [7:0] i2c_data_to_master = 8'b00000000;
	wire [7:0] i2c_data_from_master;
	wire i2c_data_valid;
	wire i2c_rst = 1'b0;
	reg [7:0] i2c_reg_cur = 8'b00000000;
	parameter i2c_addr = 7'b0110001;
	parameter i2c_pg_reg_addr1 = 8'b00000001;
	parameter i2c_pg_reg_addr2 = i2c_pg_reg_addr1 + 1;
	parameter i2c_status_reg_addr = i2c_pg_reg_addr2 + 1;
	
	I2C_slave #(i2c_addr)
	I2C_SLAVE(
		SCL,
		SDA,
		CLK_IN,
		i2c_rst,
		i2c_read_req,
		i2c_data_to_master,
		i2c_data_valid,
		i2c_data_from_master
	);
	
	// Handle I2C
	// 2 8-bit registers with PGOOD state on error
	always @(posedge CLK_IN) begin
		//return high byte with any memory address, loop on any consecutive reads
		if (i2c_data_valid == 1'b1) begin
			i2c_reg_cur <= i2c_data_from_master;
		end
		else if (i2c_read_req == 1'b1) begin
			i2c_reg_cur <= i2c_reg_cur + 1;
		end
		case(i2c_reg_cur)
			i2c_pg_reg_addr1 : begin
				i2c_data_to_master <= err_msg[15:8];
			end
			i2c_pg_reg_addr2 : begin
				i2c_data_to_master <= err_msg[7:0];
			end
			i2c_status_reg_addr : begin
				i2c_data_to_master <= {6'b000000,SYSEN,SYSGOOD_BUF};
			end
			default : begin
				i2c_data_to_master <= 8'b00000000;
			end
		endcase
		end
	
	// Power Sequencer state machine
	always @(posedge CLK_IN) begin
		// Increase counter
		T_COUNT <= T_COUNT + 1;

		// Synchronize Asynchronous inputs to clock
		PGOOD_S <= PGOOD_A;
		PGOOD <= PGOOD_S;
		SYSEN_S <= SYSEN_A;
		SYSEN <= SYSEN_S;

		// Decide next state
		case(state)
			idleoff : begin
				//Only leave idle off if system enable active and no error
				if (ERR == 1'b1) begin
					state <= idleoff;
				end
				else if (SYSEN == 1'b1) begin
					state <= shifton;
				end else begin
					state <= idleoff;
				end
			end
			shifton : begin
				// enable next power rail, reset counter, wait for pgood
				EN_BUF[rail_size - 1:1] <= EN_BUF[rail_size - 2:0];
				EN_BUF[0] <= 1'b1;
				T_COUNT <= {(((counter_size - 1))-((0))+1){1'b0}};
				state <= waitpgood;
				end
				waitpgood : begin
				// Wait for enabled power rail's PGOOD, after time with no pgood, error
				if (T_COUNT > t_max_wait) begin
					ERR <= 1'b1;
					err_msg <= {16{1'b0}};
					err_msg[rail_size - 1:0] <= EN_BUF & PGOOD;
					state <= shiftoff;
				end
				else if ((EN_BUF & PGOOD) == all_on) begin
					state <= idleon;
				end
				else if (((EN_BUF & PGOOD) == EN_BUF)) begin
					T_COUNT <= {(((counter_size - 1))-((0))+1){1'b0}};
					state <= waiten;
				end else begin
					state <= waitpgood;
				end
			end
			waiten : begin
				// delay between last pgood and next enable
				if (T_COUNT > t_delay) begin
					T_COUNT <= {(((counter_size - 1))-((0))+1){1'b0}};
					state <= shifton;
				end else begin
					state <= waiten;
				end
			end
			idleon : begin
				// stay in idle on unless power rail goes down (error) or system enable removed
				SYSGOOD_BUF <= 1'b1;
				if ((!(PGOOD == all_on))) begin
					ERR <= 1'b1;
					err_msg <= {16{1'b0}};
					err_msg[rail_size - 1:0] <= PGOOD;
				end
				if (((SYSEN == 1'b0) || (ERR == 1'b1))) begin
					SYSGOOD_BUF <= 1'b0;
					state <= shiftoff;
				end else begin
					state <= idleon;
				end
			end
			shiftoff : begin
				// Turn off enable for next power rail
				EN_BUF[rail_size - 2:0] <= EN_BUF[rail_size - 1:1];
				EN_BUF[rail_size - 1] <= 1'b0;
				if ((EN_BUF == all_off)) begin
					state <= idleoff;
				end else begin
					T_COUNT <= {(((counter_size - 1))-((0))+1){1'b0}};
					state <= waitoff;
				end
			end
			waitoff : begin
				// in controlled shutdown, delay between disabling power rails
				if (ERR == 1'b1) begin
					state <= shiftoff;
					//LED_BUF <= "10";
				end
				else if (T_COUNT > t_delay) begin
					state <= shiftoff;
					//LED_BUF <= "10";
				end else begin
					state <= waitoff;
				end
			end
		endcase
	end
	
	// Output enable buffer to pins
	assign EN = ~(EN_BUF);
	assign i2c_rst = 1'b0;
	assign SYSGOOD = SYSGOOD_BUF;
	
endmodule