// Copyright © 2014-2016 Peter Samarin
// Copyright © 2017-2018 Raptor Engineering, LLC
// All Rights Reserved
//
// See I2C_SLAVE_LICENSE file for licensing details

module i2c_slave(
		input wire scl_in,
		output wire scl_out,
		output wire scl_direction,

		input wire sda_in,
		output wire sda_out,
		output wire sda_direction,

		input wire clk,
		input wire rst,

		// User interface
		output wire read_req,
		input wire [7:0] data_to_master,
		output wire data_valid,
		output wire [7:0] data_from_master,
		output wire [7:0] write_cycle_count
	);

	parameter [6:0] SLAVE_ADDR = 0;


	//----------------------------------------------------------
	parameter [2:0]
	i2c_idle = 0,
	i2c_get_address_and_cmd = 1,
	i2c_answer_ack_start = 2,
	i2c_write = 3,
	i2c_read = 4,
	i2c_read_ack_start = 5,
	i2c_read_ack_got_rising = 6,
	i2c_read_stop = 7;
	// I2C state management
	reg [2:0] state_reg = i2c_idle;
	reg cmd_reg = 1'b0;
	reg [31:0] bits_processed_reg = 0;
	reg continue_reg = 1'b0; 	// Helpers to figure out next state
	reg start_reg = 1'b0;
	reg stop_reg = 1'b0;
	reg scl_rising_reg = 1'b0;
	reg scl_falling_reg = 1'b0; 	// Address and data received from master
	reg [6:0] addr_reg = 1'b0;
	reg [7:0] data_reg = 1'b0; 	// Delayed SCL (by 1 clock cycle, and by 2 clock cycles)
	reg [7:0] wr_cyc_count_reg = 8'b00000000;
	reg scl_reg = 1'b1;
	reg scl_prev_reg = 1'b1; 	// Slave writes on scl
	wire scl_wen_reg = 1'b0;
	wire scl_o_reg = 1'b0; 		// Delayed SDA (1 clock cycle, and 2 clock cycles)
	reg sda_reg = 1'b1;
	reg sda_prev_reg = 1'b1; 	// Slave writes on sda
	reg sda_wen_reg = 1'b0;
	reg sda_o_reg = 1'b0; 		// User interface
	reg data_valid_reg = 1'b0;
	reg read_req_reg = 1'b0;
	reg [7:0] data_to_master_reg = 1'b0;
	
	always @(posedge clk) begin
		// Delay SCL by 1 and 2 clock cycles
		scl_reg <= scl_in;
		scl_prev_reg <= scl_reg;

		// Delay SDA by 1 and 2 clock cycles
		sda_reg <= sda_in;
		sda_prev_reg <= sda_reg;

		// Detect rising and falling SCL
		scl_rising_reg <= 1'b0;
		if (scl_prev_reg == 1'b0 && scl_reg == 1'b1) begin
			scl_rising_reg <= 1'b1;
		end
			scl_falling_reg <= 1'b0;
		if (scl_prev_reg == 1'b1 && scl_reg == 1'b0) begin
			scl_falling_reg <= 1'b1;
		end

		// Detect I2C START condition
		start_reg <= 1'b0;
		stop_reg <= 1'b0;
		if (scl_reg == 1'b1 && scl_prev_reg == 1'b1 && sda_prev_reg == 1'b1 && sda_reg == 1'b0) begin
			start_reg <= 1'b1;
			stop_reg <= 1'b0;
		end

		// Detect I2C STOP condition
		if (scl_prev_reg == 1'b1 && scl_reg == 1'b1 && sda_prev_reg == 1'b0 && sda_reg == 1'b1) begin
			start_reg <= 1'b0;
			stop_reg <= 1'b1;
		end
	end
	
	//--------------------------------------------------------
	// I2C state machine
	//--------------------------------------------------------
	always @(posedge clk) begin
		// Default assignments
		sda_o_reg <= 1'b0;
		sda_wen_reg <= 1'b0;

		// User interface
		data_valid_reg <= 1'b0;
		read_req_reg <= 1'b0;

		case(state_reg)
			i2c_idle : begin
				if (start_reg == 1'b1) begin
					state_reg <= i2c_get_address_and_cmd;
					bits_processed_reg <= 0;
					wr_cyc_count_reg <= 0;
				end
			end
			i2c_get_address_and_cmd : begin
				if (scl_rising_reg == 1'b1) begin
					if (bits_processed_reg < 7) begin
						bits_processed_reg <= bits_processed_reg + 1;
						addr_reg[6 - bits_processed_reg] <= sda_reg;
					end
					else if (bits_processed_reg == 7) begin
						bits_processed_reg <= bits_processed_reg + 1;
						cmd_reg <= sda_reg;
					end
				end
				if (bits_processed_reg == 8 && scl_falling_reg == 1'b1) begin
					bits_processed_reg <= 0;
					if (addr_reg == SLAVE_ADDR) begin
						// check req address
						state_reg <= i2c_answer_ack_start;
						if (cmd_reg == 1'b1) begin
							// issue read request 
							read_req_reg <= 1'b1;
							data_to_master_reg <= data_to_master;
						end
					end else begin
						state_reg <= i2c_idle;
					end
				end
			end
			i2c_answer_ack_start : begin
				//--------------------------------------------------
				// I2C acknowledge to master
				//--------------------------------------------------
				sda_wen_reg <= 1'b1;
				sda_o_reg <= 1'b0;
				if (scl_falling_reg == 1'b1) begin
					if (cmd_reg == 1'b0) begin
						state_reg <= i2c_write;
					end else begin
						state_reg <= i2c_read;
					end
				end
			end
			i2c_write : begin
				//--------------------------------------------------
				// WRITE
				//--------------------------------------------------
				if (scl_rising_reg == 1'b1) begin
					if (bits_processed_reg <= 7) begin
						data_reg[7 - bits_processed_reg] <= sda_reg;
						bits_processed_reg <= bits_processed_reg + 1;
					end
					if (bits_processed_reg == 7) begin
						data_valid_reg <= 1'b1;
						wr_cyc_count_reg <= wr_cyc_count_reg + 1;
					end
				end
				if (scl_falling_reg == 1'b1 && bits_processed_reg == 8) begin
					state_reg <= i2c_answer_ack_start;
					bits_processed_reg <= 0;
				end
			end
			i2c_read : begin
				//--------------------------------------------------
				// READ: send data to master
				//--------------------------------------------------
				sda_wen_reg <= 1'b1;
				sda_o_reg <= data_to_master_reg[7 - bits_processed_reg];
				if (scl_falling_reg == 1'b1) begin
					if (bits_processed_reg < 7) begin
						bits_processed_reg <= bits_processed_reg + 1;
					end
					else if (bits_processed_reg == 7) begin
						state_reg <= i2c_read_ack_start;
						bits_processed_reg <= 0;
					end
				end
			end
			i2c_read_ack_start : begin
				//--------------------------------------------------
				// I2C read master acknowledge
				//--------------------------------------------------
				if (scl_rising_reg == 1'b1) begin
					state_reg <= i2c_read_ack_got_rising;
					if (sda_reg == 1'b1) begin
						// nack = stop read
						continue_reg <= 1'b0;
					end else begin
						// ack = continue read
						continue_reg <= 1'b1;
						read_req_reg <= 1'b1;

						// request reg byte
						data_to_master_reg <= data_to_master;
					end
				end
			end
			i2c_read_ack_got_rising : begin
				// Wait for START or STOP to get out of this state
				if (scl_falling_reg == 1'b1) begin
					if (continue_reg == 1'b1) begin
						if (cmd_reg == 1'b0) begin
							state_reg <= i2c_write;
						end else begin
							state_reg <= i2c_read;
						end
					end else begin
						state_reg <= i2c_read_stop;
					end
				end
			end
			i2c_read_stop : begin
				// Wait for START or STOP to get out of this state
			end
			default : begin
			end
		endcase
		//------------------------------------------------------
		// Reset counter and state on start/stop
		//------------------------------------------------------
		if (start_reg == 1'b1) begin
			state_reg <= i2c_get_address_and_cmd;
			bits_processed_reg <= 0;
			wr_cyc_count_reg <= 0;
		end
		if (stop_reg == 1'b1) begin
			state_reg <= i2c_idle;
			bits_processed_reg <= 0;
			wr_cyc_count_reg <= 0;
		end
		if (rst == 1'b1) begin
			state_reg <= i2c_idle;
		end
	end
	
	//--------------------------------------------------------
	// I2C interface
	//--------------------------------------------------------
	assign sda_out = (sda_o_reg & sda_wen_reg);
	assign sda_direction = sda_wen_reg;
	assign scl_out = (scl_o_reg & scl_wen_reg);
	assign scl_direction = scl_wen_reg;
	//--------------------------------------------------------
	// User interface
	//--------------------------------------------------------
	// Master writes
	assign data_valid = data_valid_reg;
	assign data_from_master = data_reg;
	assign write_cycle_count = wr_cyc_count_reg;
	// Master reads
	assign read_req = read_req_reg;
	
endmodule