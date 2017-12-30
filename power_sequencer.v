// Copyright © 2017, International Business Machines Corp.
// Copyright © 2017 Raptor Engineering, LLC
// All Rights Reserved
//
// See LICENSE file for licensing details

module pwrseq(
		output wire [rail_size - 1:0] en,
		input wire [rail_size - 1:0] pgood_a,
		input wire sysen_a,
		output wire sysgood,

		input wire scl_in,
		output wire scl_out,
		output wire scl_direction,

		input wire sda_in,
		output wire sda_out,
		output wire sda_direction,

		input wire clk_in
	);

	parameter [31:0] rail_size = 15;

	// Input output buffers and synchronizers
	reg [rail_size - 1:0] en_buf = 1'b0;
	reg [rail_size - 1:0] pgood = 1'b0;
	reg [rail_size - 1:0] pgood_s = 1'b0;
	reg sysen = 1'b0;
	reg sysen_s = 1'b0;
	reg sysgood_buf = 1'b0;

	// Sequencer State Machine
	parameter [2:0] idleon = 0;
	parameter [2:0] shifton = 1;
	parameter [2:0] waitpgood = 2;
	parameter [2:0] waiten = 3;
	parameter [2:0] idleoff = 4;
	parameter [2:0] shiftoff = 5;
	parameter [2:0] waitoff = 6;

	reg [2:0] state = idleoff;
	reg err = 1'b0;
	reg [15:0] err_msg = 1'b1;
	parameter all_on = 1'b1;
	parameter all_off = 1'b0;

	// Clocks and timers
	parameter counter_size = 20;
	reg [counter_size - 1:0] t_count;

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
	
	i2c_slave #(
		.SLAVE_ADDR(i2c_addr)
	)
	i2c_slave_instance(
		.scl_in(scl_in),
		.scl_out(scl_out),
		.scl_direction(scl_direction),

		.sda_in(sda_in),
		.sda_out(sda_out),
		.sda_direction(sda_direction),

		.clk(clk_in),
		.rst(i2c_rst),
		.read_req(i2c_read_req),
		.data_to_master(i2c_data_to_master),
		.data_valid(i2c_data_valid),
		.data_from_master(i2c_data_from_master)
	);
	
	// Handle I2C
	// 2 8-bit registers with pgood state on error
	always @(posedge clk_in) begin
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
				i2c_data_to_master <= {6'b000000, sysen, sysgood_buf};
			end
			default : begin
				i2c_data_to_master <= 8'b00000000;
			end
		endcase
		end
	
	// Power Sequencer state machine
	always @(posedge clk_in) begin
		// Increase counter
		t_count <= t_count + 1;

		// Synchronize Asynchronous inputs to clock
		pgood_s <= pgood_a;
		pgood <= pgood_s;
		sysen_s <= sysen_a;
		sysen <= sysen_s;

		// Decide next state
		case(state)
			idleoff : begin
				//Only leave idle off if system enable active and no error
				if (err == 1'b1) begin
					state <= idleoff;
				end
				else if (sysen == 1'b1) begin
					state <= shifton;
				end else begin
					state <= idleoff;
				end
			end
			shifton : begin
				// enable next power rail, reset counter, wait for pgood
				en_buf[rail_size - 1:1] <= en_buf[rail_size - 2:0];
				en_buf[0] <= 1'b1;
				t_count <= {(((counter_size - 1))-((0))+1){1'b0}};
				state <= waitpgood;
				end
				waitpgood : begin
				// Wait for enabled power rail's pgood, after time with no pgood, error
				if (t_count > t_max_wait) begin
					err <= 1'b1;
					err_msg <= {16{1'b0}};
					err_msg[rail_size - 1:0] <= en_buf & pgood;
					state <= shiftoff;
				end
				else if ((en_buf & pgood) == all_on) begin
					state <= idleon;
				end
				else if (((en_buf & pgood) == en_buf)) begin
					t_count <= {(((counter_size - 1))-((0))+1){1'b0}};
					state <= waiten;
				end else begin
					state <= waitpgood;
				end
			end
			waiten : begin
				// delay between last pgood and next enable
				if (t_count > t_delay) begin
					t_count <= {(((counter_size - 1))-((0))+1){1'b0}};
					state <= shifton;
				end else begin
					state <= waiten;
				end
			end
			idleon : begin
				// stay in idle on unless power rail goes down (error) or system enable removed
				sysgood_buf <= 1'b1;
				if ((!(pgood == all_on))) begin
					err <= 1'b1;
					err_msg <= {16{1'b0}};
					err_msg[rail_size - 1:0] <= pgood;
				end
				if (((sysen == 1'b0) || (err == 1'b1))) begin
					sysgood_buf <= 1'b0;
					state <= shiftoff;
				end else begin
					state <= idleon;
				end
			end
			shiftoff : begin
				// Turn off enable for next power rail
				en_buf[rail_size - 2:0] <= en_buf[rail_size - 1:1];
				en_buf[rail_size - 1] <= 1'b0;
				if ((en_buf == all_off)) begin
					state <= idleoff;
				end else begin
					t_count <= {(((counter_size - 1))-((0))+1){1'b0}};
					state <= waitoff;
				end
			end
			waitoff : begin
				// in controlled shutdown, delay between disabling power rails
				if (err == 1'b1) begin
					state <= shiftoff;
					//LED_BUF <= "10";
				end
				else if (t_count > t_delay) begin
					state <= shiftoff;
					//LED_BUF <= "10";
				end else begin
					state <= waitoff;
				end
			end
		endcase
	end
	
	// Output enable buffer to pins
	assign en = ~(en_buf);
	assign i2c_rst = 1'b0;
	assign sysgood = sysgood_buf;
	
endmodule