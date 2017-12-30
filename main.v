// Copyright © 2017, International Business Machines Corp.
// Copyright © 2017 Raptor Engineering, LLC
// All Rights Reserved
//
// See LICENSE file for licensing details

module system_fpga_top
	(
		// LPC clock
		input wire lpc_clock,

		// General I/O
		input wire sysen,
		output wire sysgood,
		input wire debug_in,

		// DD1 temp fix for VCS overcurrent bug
		input wire seq_cont,

		// Enable outputs
		output wire vdda_en,
		output wire vddb_en,
		output wire vcsa_en,
		output wire vcsb_en,
		output wire vdna_en,
		output wire vdnb_en,
		output wire vioa_en,
		output wire viob_en,
		output wire vppab_en,
		output wire vppcd_en,
		output wire vddrab_en,
		output wire vttab_en,
		output wire vddrcd_en,
		output wire vttcd_en,
		output wire avdd_en,
		output wire miscio_en,
		output wire atx_en,

		// Power Good inputs
		input wire vdda_pg,
		input wire vddb_pg,
		input wire vcsa_pg,
		input wire vcsb_pg,
		input wire vdna_pg,
		input wire vdnb_pg,
		input wire vioa_pg,
		input wire viob_pg,
		input wire vppab_pg,
		input wire vppcd_pg,
		input wire vddrab_pg,
		input wire vddrcd_pg,
		input wire avdd_pg,
		input wire miscio_pg,
		input wire atx_pg,
		input wire bmc_vr_pg,

		// I2C
		inout i2c_scl,
		inout i2c_sda,

		// Second CPU Present Detection
		input wire cpub_present_n,
		output wire cpub_clk_oea,
		output wire cpub_clk_oeb,

		// Resets
		output wire lpc_rst,
		input wire bmc_software_pg,
		output wire bmc_rst,
		output wire fan_rst,
		output wire usbhub_rst,
		output wire cpu_stby_rst,

		// Reserved for Future Use
		output wire dual_5v_ctrl,
		output wire window_open_n
	);

	wire i2c_scl_in;
	wire i2c_scl_out;
	wire i2c_scl_direction;

	wire i2c_sda_in;
	wire i2c_sda_out;
	wire i2c_sda_direction;

	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b1)
	) i2c_scl_io (
		.PACKAGE_PIN(i2c_scl),
		.OUTPUT_ENABLE(i2c_scl_direction),
		.D_OUT_0(i2c_scl_out),
		.D_IN_0(i2c_scl_in)
	);

	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b1)
	) i2c_sda_io (
		.PACKAGE_PIN(i2c_sda),
		.OUTPUT_ENABLE(i2c_sda_direction),
		.D_OUT_0(i2c_sda_out),
		.D_IN_0(i2c_sda_in)
	);

	// TODO update version
	parameter fpga_version = 8'b00000110;
	parameter rail_size = 15;
	wire [rail_size - 1:0] en_buf = 1'b0;
	wire [rail_size - 1:0] pg_buf;
	wire sysgood_buf;
	wire clk_in;
	wire stdby_sed;
	wire sysen_buf;
	parameter railarray_0 = 1'b0;
	parameter railarray_1 = 1'b1; 	// synchronizing signals
	reg [rail_size - 1:0] pg_s1 = 1'b0;
	reg [rail_size - 1:0] pg_s2 = 1'b0;
	reg sysen_s1 = 1'b0;
	reg sysen_s2 = 1'b0;
	reg seq_s1 = 1'b1;
	reg seq_s2 = 1'b1;		// Timer (Watchdog and Delay) signals
	reg [rail_size - 1:0] delay_done = 1'b0;
	reg [23:0] w_count = 1'b0;
	reg [16:0] d_count = 1'b0; 	// at 4.16MHz, w_count(23) being one means approximately 100ms have passed, good for checking watchdog between EN and PG
					// d_count(16) being one means approximately 15ms have passed, good enough for delay betwen one rail and the next
	reg wait_err = 1'b0;
	reg operation_err = 1'b0;
	reg err_found = 1'b0;
	wire clear_err = 1'b0;

	// i2c signals	
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

	// Divide input 33MHz clock down to 4.125MHz
	reg [2:0] clock_divider;
	always @(posedge lpc_clock) begin
		clock_divider = clock_divider + 1;
	end
	assign clk_in = clock_divider[2];

	// I2C device
	i2c_slave #(
		.SLAVE_ADDR(i2c_addr)
	)
	i2c_slave_instance(
		.scl_in(i2c_scl_in),
		.scl_out(i2c_scl_out),
		.scl_direction(i2c_scl_direction),

		.sda_in(i2c_sda_in),
		.sda_out(i2c_sda_out),
		.sda_direction(i2c_sda_direction),

		.clk(clk_in),
		.rst(i2c_rst),
		.read_req(i2c_read_req),
		.data_to_master(i2c_data_to_master),
		.data_valid(i2c_data_valid),
		.data_from_master(i2c_data_from_master)
	);
	
	assign i2c_rst = 1'b0;
	// Handle I2C
	// 2 8-bit registers with PGOOD state on error
	always @(posedge clk_in) begin
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
				// TODO add CPU1 presence detect
				i2c_data_to_master <= {3'b000, wait_err, operation_err, err_found, sysen_buf, sysgood_buf};
			end
			i2c_version_reg_addr : begin
				i2c_data_to_master <= fpga_version;
			end
			default : begin
				i2c_data_to_master <= 8'b00000000;
			end
		endcase
	end
	
	always @(posedge clk_in) begin
		pg_s1 <= pg_buf;
		pg_s2 <= pg_s1;
		sysen_s1 <= sysen_buf;
		sysen_s2 <= sysen_s1;
		seq_s1 <= seq_cont;
		seq_s2 <= seq_s1;
		if ((clear_err == 1'b1)) begin
			wait_err <= 1'b0;
			operation_err <= 1'b0;
			err_found <= 1'b0;
			w_count <= {24{1'b0}};
			d_count <= {17{1'b0}};
		end
		else if ((sysen_s2 == 1'b0 || err_found == 1'b1)) begin
			w_count <= {24{1'b0}};
			d_count <= {17{1'b0}};
			delay_done <= {(((rail_size - 1))-((0))+1){1'b0}};
		end
		else if ((pg_s2[0] == 1'b1 && en_buf[0] == 1'b1 && delay_done[0] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[0] <= 1'b1;
			end
		end
		else if ((pg_s2[1] == 1'b1 && en_buf[1] == 1'b1 && delay_done[1] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[1] <= 1'b1;
			end
		end
		else if ((pg_s2[2] == 1'b1 && en_buf[2] == 1'b1 && delay_done[2] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[2] <= 1'b1;
			end
		end
		else if ((pg_s2[3] == 1'b1 && en_buf[3] == 1'b1 && delay_done[3] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[3] <= 1'b1;
			end
		end
		else if ((pg_s2[4] == 1'b1 && en_buf[4] == 1'b1 && delay_done[4] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[4] <= 1'b1;
			end
		end
		else if ((pg_s2[5] == 1'b1 && en_buf[5] == 1'b1 && delay_done[5] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[5] <= 1'b1;
			end
		end
		else if ((pg_s2[6] == 1'b1 && en_buf[6] == 1'b1 && delay_done[6] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[6] <= 1'b1;
			end
		end
		else if ((pg_s2[7] == 1'b1 && en_buf[7] == 1'b1 && delay_done[7] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[7] <= 1'b1;
			end
		end
		else if ((pg_s2[8] == 1'b1 && en_buf[8] == 1'b1 && delay_done[8] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[8] <= 1'b1;
			end
		end
		else if ((pg_s2[9] == 1'b1 && en_buf[9] == 1'b1 && delay_done[9] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[9] <= 1'b1;
			end
		end
		else if ((pg_s2[10] == 1'b1 && en_buf[10] == 1'b1 && delay_done[10] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[10] <= 1'b1;
			end
		end
		else if ((pg_s2[11] == 1'b1 && en_buf[11] == 1'b1 && delay_done[11] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[11] <= 1'b1;
			end
		end
		else if ((pg_s2[12] == 1'b1 && en_buf[12] == 1'b1 && delay_done[12] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[12] <= 1'b1;
			end
		end
		else if ((pg_s2[13] == 1'b1 && en_buf[13] == 1'b1 && delay_done[13] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[13] <= 1'b1;
			end
		end
		else if ((pg_s2[14] == 1'b1 && en_buf[14] == 1'b1 && delay_done[14] == 1'b0)) begin
			w_count <= {24{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[14] <= 1'b1;
			end
		end

		// Error Checks
		// Check time between Enables going high and PGOODs arriving. Error out after 100ms
		else if ((pg_s2[0] == 1'b0 && en_buf[0] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[1] == 1'b0 && en_buf[1] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[2] == 1'b0 && en_buf[2] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[3] == 1'b0 && en_buf[3] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[4] == 1'b0 && en_buf[4] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[5] == 1'b0 && en_buf[5] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[6] == 1'b0 && en_buf[6] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[7] == 1'b0 && en_buf[7] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[8] == 1'b0 && en_buf[8] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[9] == 1'b0 && en_buf[9] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[10] == 1'b0 && en_buf[10] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[11] == 1'b0 && en_buf[11] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[12] == 1'b0 && en_buf[12] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[13] == 1'b0 && en_buf[13] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[14] == 1'b0 && en_buf[14] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {24{1'b0}};
				wait_err <= 1'b1;
			end
		end
		if ((( ~(delay_done & ~pg_s2)) != railarray_1)) begin
			operation_err <= 1'b1;
		end
		if (((wait_err | operation_err) == 1'b1 && clear_err == 1'b0)) begin
			err_found <= 1'b1;
		end else begin
			i2c_pg_reg[14:0] <= pg_s2[14:0];
		end
	end
	
	// Assign Ports to Enables
	assign atx_en = ~en_buf[0];
	assign miscio_en = en_buf[1];
	assign vdna_en = en_buf[2];
	assign vdnb_en = en_buf[3] & ~cpub_present_n;
	assign avdd_en = en_buf[4];
	assign vioa_en = en_buf[5];
	assign viob_en = en_buf[6] & ~cpub_present_n;
	assign vdda_en = en_buf[7];
	assign vddb_en = en_buf[8] & ~cpub_present_n;
	assign vcsa_en = en_buf[9];
	assign vcsb_en = en_buf[10] & ~cpub_present_n;
	assign vppab_en = en_buf[11];
	assign vppcd_en = en_buf[12] & ~cpub_present_n;
	assign vddrab_en = en_buf[13];
	assign vttab_en = en_buf[13];
	assign vddrcd_en = en_buf[14] & ~cpub_present_n;
	assign vttcd_en = en_buf[14] & ~cpub_present_n;

	// Assign Ports to PGood buffer
	assign pg_buf[0] = atx_pg;
	assign pg_buf[1] = miscio_pg;
	assign pg_buf[2] = vdna_pg;
	assign pg_buf[3] = vdnb_pg | (cpub_present_n & en_buf[3]);
	assign pg_buf[4] = avdd_pg;
	assign pg_buf[5] = vioa_pg;
	assign pg_buf[6] = viob_pg | (cpub_present_n & en_buf[6]);
	assign pg_buf[7] = vdda_pg;
	assign pg_buf[8] = vddb_pg | (cpub_present_n & en_buf[8]);
	assign pg_buf[9] = vcsa_pg;
	assign pg_buf[10] = vcsb_pg | (cpub_present_n & en_buf[10]);
	assign pg_buf[11] = vppab_pg;
	assign pg_buf[12] = vppcd_pg | (cpub_present_n & en_buf[12]);
	assign pg_buf[13] = vddrab_pg;
	assign pg_buf[14] = vddrcd_pg | (cpub_present_n & en_buf[14]);

	// Enable outputs
	// Shut everything off if an error has occurred
	// Otherwise, if system enable is up, then enable short delay is done after previous rail
	// Otherwise, disable after next rail goes down
	assign en_buf[0] = (sysen_s2 | pg_s2[1]) & ~err_found;
	assign en_buf[1] = ((sysen_s2 & delay_done[0]) | pg_s2[2]) & ~err_found;
	assign en_buf[2] = ((sysen_s2 & delay_done[1]) | pg_s2[3]) & ~err_found;
	assign en_buf[3] = ((sysen_s2 & delay_done[2]) | pg_s2[4]) & ~err_found;
	assign en_buf[4] = ((sysen_s2 & delay_done[ + 1]) | pg_s2[ + 1]) & ~err_found;
	assign en_buf[5] = ((sysen_s2 & delay_done[4]) | pg_s2[6]) & ~err_found;
	assign en_buf[6] = ((sysen_s2 & delay_done[5]) | pg_s2[7]) & ~err_found;
	assign en_buf[7] = ((sysen_s2 & delay_done[6]) | pg_s2[8]) & ~err_found;
	assign en_buf[8] = ((sysen_s2 & delay_done[7]) | pg_s2[9]) & ~err_found;
	assign en_buf[9] = (( ~seq_s2 & sysen_s2 & delay_done[8]) | pg_s2[10]) & ~err_found;
	assign en_buf[10] = ((sysen_s2 & delay_done[9]) | pg_s2[11]) & ~err_found;
	assign en_buf[11] = ((sysen_s2 & delay_done[10]) | pg_s2[12]) & ~err_found;
	assign en_buf[12] = ((sysen_s2 & delay_done[11]) | pg_s2[13]) & ~err_found;
	assign en_buf[13] = ((sysen_s2 & delay_done[12]) | pg_s2[14]) & ~err_found;
	assign en_buf[14] = (sysen_s2 & delay_done[13]) & ~err_found;

	// ERR state reset
	assign clear_err = i2c_clr_err;

	// CPUB clk enables
	assign cpub_clk_oea = ~cpub_present_n;
	assign cpub_clk_oeb = ~cpub_present_n;

	// System PWRGOOD
	assign sysgood_buf = delay_done[14];
	assign sysgood = sysgood_buf & bmc_software_pg;
	assign lpc_rst = sysgood_buf;

	// CPU Reset
	assign cpu_stby_rst = en_buf[0];

	// BMC RESETs
	assign bmc_rst = bmc_vr_pg;
	assign usbhub_rst = sysgood_buf & bmc_software_pg;
	assign fan_rst = ~bmc_vr_pg;

	// debug_in override allows non-BMC control of CPLD
	assign sysen_buf = sysen | ~debug_in;
	// assign sysen_buf = ~debug_in;

	assign dual_5v_ctrl = 1'b0;
	assign window_open_n = 1'b0;
	
endmodule
