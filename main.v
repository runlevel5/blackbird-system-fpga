// Copyright © 2017, International Business Machines Corp.
// Copyright © 2017 - 2018 Raptor Engineering, LLC
// All Rights Reserved
//
// See LICENSE file for licensing details

module system_fpga_top
	(
		// LPC clock
		input wire lpc_clock,

		// General I/O
		input wire sysen,
		output reg sysgood,
		input wire debug_in,

		// DD1 temp fix for VCS overcurrent bug
		input wire seq_cont,

		// Enable outputs
		output reg vdda_en,
		output reg vddb_en,
		output reg vcsa_en,
		output reg vcsb_en,
		output reg vdna_en,
		output reg vdnb_en,
		output reg vioa_en,
		output reg viob_en,
		output reg vppab_en,
		output reg vppcd_en,
		output reg vddrab_en,
		output reg vttab_en,
		output reg vddrcd_en,
		output reg vttcd_en,
		output reg avdd_en,
		output reg miscio_en,
		output reg atx_en,

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

		// Second CPU presence detect
		input wire cpub_present_n,
		output wire cpub_clk_oea,
		output wire cpub_clk_oeb,

		// Resets
		output reg lpc_rst,
		input wire bmc_software_pg,
		output reg bmc_rst,
		output reg fan_rst,
		output reg usbhub_rst,
		inout cpu_stby_rst,

		// Reserved for future use
		output reg dual_5v_ctrl,
		output reg window_open_n,

		// BMC system reset signalling
		output reg bmc_system_reset_request_n,

		// Component disable lines
		output reg pmc_disable_n,

		// System status lines
		inout nic1_act_led_n,
		inout nic2_act_led_n,
		inout nic1_link_led_n,
		inout nic2_link_led_n,
		input wire nic1_green_led_n,
		input wire nic2_green_led_n,
		input wire bmc_uid_led_req,

		// Front panel indicators
		output reg panel_nic1_led_cathode,
		output reg panel_nic2_led_cathode,
		output reg panel_uid_led,

		// Front panel switches
		input wire panel_reset_in_l,

		// FlexVer™ connections
		input wire flexver_reset_in_l
	);

	// CPU standby reset is on 1.1V domain, but FPGA I/Os are on 3.3V domain
	// Use open-drain reset signal
	reg cpu_stby_rst_assert = 1'b1;

	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b0)
	) cpu_stby_rst_io (
		.PACKAGE_PIN(cpu_stby_rst),
		.OUTPUT_ENABLE(cpu_stby_rst_assert),
		.D_OUT_0(1'b0)
	);

	// Make NIC activity lights work
	// WARNING: Hic Sunt Dracones!
	//
	// The base reference design Talos is built on never quite got the networking LEDs correct.
	// After two iterations of nonfunctional SuperMicro / IBM design and no direct access to Broadcom
	// documentation, Raptor decided the rear LED functionality was not important enough to hold up
	// the entire Talos project.  As a result, the three offending signals were wired up to this
	// FPGA, and the front panel LEDs were isolated from the Broadcom NIC and connected to this FPGA.
	//
	// Detailed testing on production hardware subsequently revealed the LED drivers are all open drain,
	// active low, with no pull-up provided on the activity line.  Furthermore, LINKLED_L is anything but
	// what it says on the tin; it appears to only go low when in 10Mbps mode.  To top off this whole mess,
	// the last reference design errata led to the anode and cathode of the activity LED being swapped.
	//
	// What this means in practice:
	// 1.) The FPGA needs to provide some degree of pull-up to 3.3V on the NIC activity lines
	// 2.) LINKLED_L needs to be pulled high whenever GRNLED_L is low
	//
	// With these workarounds, the network link and activity LEDs on the front and rear panel function normally.
	wire nic1_act_led_n_in;
	wire nic2_act_led_n_in;

	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b1)
	) nic1_act_led_n_io (
		.PACKAGE_PIN(nic1_act_led_n),
		.OUTPUT_ENABLE(1'b0),
		.D_OUT_0(1'b1),
		.D_IN_0(nic1_act_led_n_in)
	);
	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b1)
	) nic2_act_led_n_io (
		.PACKAGE_PIN(nic2_act_led_n),
		.OUTPUT_ENABLE(1'b0),
		.D_OUT_0(1'b1),
		.D_IN_0(nic2_act_led_n_in)
	);
	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b1)
	) nic1_link_led_n_io (
		.PACKAGE_PIN(nic1_link_led_n),
		.OUTPUT_ENABLE(~nic1_green_led_n),
		.D_OUT_0(1'b1)
	);
	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b1)
	) nic2_link_led_n_io (
		.PACKAGE_PIN(nic2_link_led_n),
		.OUTPUT_ENABLE(~nic2_green_led_n),
		.D_OUT_0(1'b1)
	);

	// I2C pin control lines
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
	parameter vendor_id1 = 8'h52;
	parameter vendor_id2 = 8'h43;
	parameter vendor_id3 = 8'h53;
	parameter vendor_id4 = 8'h20;
	parameter RAIL_SIZE = 15;
	reg [RAIL_SIZE - 1:0] en_buf = {RAIL_SIZE{1'b0}};
	reg [RAIL_SIZE - 1:0] pg_buf;
	reg sysgood_buf;
	wire clk_in;
	wire clk_in_lpc;
	wire clk_in_ring;
	wire stdby_sed;
	reg sysen_buf;
	parameter railarray_0 = {RAIL_SIZE{1'b0}};
	parameter railarray_1 = {RAIL_SIZE{1'b1}}; 	// synchronizing signals
	reg [RAIL_SIZE - 1:0] pg_s1 = {RAIL_SIZE{1'b0}};
	reg [RAIL_SIZE - 1:0] pg_s2 = {RAIL_SIZE{1'b0}};
	reg sysen_s1 = 1'b0;
	reg sysen_s2 = 1'b0;
	reg seq_s1 = 1'b1;
	reg seq_s2 = 1'b1;		// Timer (Watchdog and Delay) signals
	reg [RAIL_SIZE - 1:0] delay_done = {RAIL_SIZE{1'b0}};
	reg [23:0] w_count = 0;
	reg [16:0] d_count = 0; 	// at 4.16MHz, w_count(23) being one means approximately 100ms have passed, good for checking watchdog between EN and PG
					// d_count(16) being one means approximately 15ms have passed, good enough for delay betwen one rail and the next
	reg wait_err = 1'b0;
	reg operation_err = 1'b0;
	reg err_found = 1'b0;
	wire clear_err = 1'b0;
	wire master_reset_reqest;

	// I2C signals
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
	parameter i2c_pwr_en_stat_reg_addr1 = i2c_status_reg_addr + 1;
	parameter i2c_pwr_en_stat_reg_addr2 = i2c_pwr_en_stat_reg_addr1 + 1;
	parameter i2c_pg_stat_reg_addr1 = i2c_pwr_en_stat_reg_addr2 + 1;
	parameter i2c_pg_stat_reg_addr2 = i2c_pg_stat_reg_addr1 + 1;
	parameter i2c_version_reg_addr = 8'b00000000;
	parameter i2c_vendor_id_reg_addr1 = 8'b00001100;
	parameter i2c_vendor_id_reg_addr2 = i2c_vendor_id_reg_addr1 + 1;
	parameter i2c_vendor_id_reg_addr3 = i2c_vendor_id_reg_addr1 + 2;
	parameter i2c_vendor_id_reg_addr4 = i2c_vendor_id_reg_addr1 + 3;
	reg [15:0] i2c_pg_reg = 1'b0;
	reg i2c_clr_err = 1'b0;

	// Front panel control signals
	wire panel_nic1_led_cathode_std;
	wire panel_nic2_led_cathode_std;
	wire panel_uid_led_std;
	reg [2:0] bmc_startup_kr = 3'b000;

	// Implement nasty ring oscillator for fallback use when main system clock is offline
	// Thanks to Clifford Wolf for the idea and basic code!
	wire chain_in;
	wire chain_out;
	wire [99:0] buffers_in;
	wire [99:0] buffers_out;
	assign buffers_in = {buffers_out[98:0], chain_in};
	assign chain_out = buffers_out[99];
	assign chain_in = !chain_out;

	SB_LUT4 #(
		.LUT_INIT(16'd2)
	) buffers [99:0] (
		.O(buffers_out),
		.I0(buffers_in),
		.I1(1'b0),
		.I2(1'b0),
		.I3(1'b0)
	);

	// Divide unstable 10MHz ring clock down to ~2MHz
	reg [2:0] ring_clock_divider;
	always @(posedge chain_out) begin
		ring_clock_divider = ring_clock_divider + 1;
	end
	assign clk_in_ring = ring_clock_divider[2];

	// Divide input 33MHz clock down to 4.125MHz
	reg [2:0] lpc_clock_divider;
	always @(posedge lpc_clock) begin
		lpc_clock_divider = lpc_clock_divider + 1;
	end
	assign clk_in_lpc = lpc_clock_divider[2];

	reg clock_select = 1'b1;

	assign clk_in = (clock_select)?clk_in_ring:clk_in_lpc;

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

	// Generate BMC startup "Knight Rider" display for front panel
	wire slow_clk;
	reg [24:0] slow_clk_counter;
	always @(posedge clk_in) begin
		slow_clk_counter <= slow_clk_counter + 1;
	end
	assign slow_clk = slow_clk_counter[24];

	reg [1:0] bmc_startup_kr_state = 0;
	always @(posedge slow_clk) begin
		case (bmc_startup_kr_state)
			0: begin
				bmc_startup_kr <= 3'b100;
				bmc_startup_kr_state <= 1;
			end
			1: begin
				bmc_startup_kr <= 3'b010;
				bmc_startup_kr_state <= 2;
			end
			2: begin
				bmc_startup_kr <= 3'b001;
				bmc_startup_kr_state <= 3;
			end
			3: begin
				bmc_startup_kr <= 3'b010;
				bmc_startup_kr_state <= 0;
			end
			default: begin
				bmc_startup_kr_state = 0;
			end
		endcase
	end
	
	assign i2c_rst = 1'b0;
	// Handle I2C
	// 2 8-bit registers with PGOOD state on error
	always @(posedge clk_in) begin
		i2c_clr_err <= 1'b0;

		if (i2c_data_valid == 1'b1) begin
			// data from master is register to be read
			i2c_reg_cur <= i2c_data_from_master;
	
			// pulse clear err signal if i2c master reads register 0x03
			if (((i2c_data_from_master) == i2c_clr_err_addr)) begin
				i2c_clr_err <= 1'b1;
			end
		end
		else if (i2c_read_req == 1'b1) begin
			i2c_reg_cur <= i2c_reg_cur + 1;
		end
		case (i2c_reg_cur)
			i2c_clr_err_addr: begin
				i2c_data_to_master <= 8'b11111111;
			end
			i2c_pg_reg_addr1: begin
				i2c_data_to_master <= i2c_pg_reg[15:8];
			end
			i2c_pg_reg_addr2: begin
				i2c_data_to_master <= i2c_pg_reg[7:0];
			end
			i2c_status_reg_addr: begin
				i2c_data_to_master <= {2'b00, ~cpub_present_n, wait_err, operation_err, err_found, sysen_buf, sysgood_buf};
			end
			i2c_pwr_en_stat_reg_addr1: begin
				i2c_data_to_master <= en_buf[7:0];
			end
			i2c_pwr_en_stat_reg_addr2: begin
				i2c_data_to_master <= en_buf[RAIL_SIZE-1:8];
			end
			i2c_pg_stat_reg_addr1: begin
				i2c_data_to_master <= pg_buf[7:0];
			end
			i2c_pg_stat_reg_addr2: begin
				i2c_data_to_master <= pg_buf[RAIL_SIZE-1:8];
			end
			i2c_vendor_id_reg_addr1: begin
				i2c_data_to_master <= vendor_id1;
			end
			i2c_vendor_id_reg_addr2: begin
				i2c_data_to_master <= vendor_id2;
			end
			i2c_vendor_id_reg_addr3: begin
				i2c_data_to_master <= vendor_id3;
			end
			i2c_vendor_id_reg_addr4: begin
				i2c_data_to_master <= vendor_id4;
			end
			i2c_version_reg_addr: begin
				i2c_data_to_master <= fpga_version;
			end
			default: begin
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
			delay_done <= {(((RAIL_SIZE - 1))-((0))+1){1'b0}};
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
	always @(posedge clk_in) begin
		atx_en = ~en_buf[0];
		miscio_en = en_buf[1];
		vdna_en = en_buf[2];
		vdnb_en = en_buf[3] & ~cpub_present_n;
		avdd_en = en_buf[4];
		vioa_en = en_buf[5];
		viob_en = en_buf[6] & ~cpub_present_n;
		vdda_en = en_buf[7];
		vddb_en = en_buf[8] & ~cpub_present_n;
		vcsa_en = en_buf[9];
		vcsb_en = en_buf[10] & ~cpub_present_n;
		vppab_en = en_buf[11];
		vppcd_en = en_buf[12] & ~cpub_present_n;
		vddrab_en = en_buf[13];
		vttab_en = en_buf[13];
		vddrcd_en = en_buf[14] & ~cpub_present_n;
		vttcd_en = en_buf[14] & ~cpub_present_n;
	end

	// Assign Ports to PGood buffer
	always @(posedge clk_in) begin
		pg_buf[0] = atx_pg;
		pg_buf[1] = miscio_pg;
		pg_buf[2] = vdna_pg;
		pg_buf[3] = vdnb_pg | (cpub_present_n & en_buf[3]);
		pg_buf[4] = avdd_pg;
		pg_buf[5] = vioa_pg;
		pg_buf[6] = viob_pg | (cpub_present_n & en_buf[6]);
		pg_buf[7] = vdda_pg;
		pg_buf[8] = vddb_pg | (cpub_present_n & en_buf[8]);
		pg_buf[9] = vcsa_pg;
		pg_buf[10] = vcsb_pg | (cpub_present_n & en_buf[10]);
		pg_buf[11] = vppab_pg;
		pg_buf[12] = vppcd_pg | (cpub_present_n & en_buf[12]);
		pg_buf[13] = vddrab_pg;
		pg_buf[14] = vddrcd_pg | (cpub_present_n & en_buf[14]);
	end

	// Enable outputs
	// Shut everything off if an error has occurred
	// Otherwise, if system enable is up, then enable short delay is done after previous rail
	// Otherwise, disable after next rail goes down
	always @(posedge clk_in) begin
		en_buf[0] = (sysen_s2 | pg_s2[1]) & ~err_found;
		en_buf[1] = ((sysen_s2 & delay_done[0]) | pg_s2[2]) & ~err_found;
		en_buf[2] = ((sysen_s2 & delay_done[1]) | pg_s2[3]) & ~err_found;
		en_buf[3] = ((sysen_s2 & delay_done[2]) | pg_s2[4]) & ~err_found;
		en_buf[4] = ((sysen_s2 & delay_done[ + 1]) | pg_s2[ + 1]) & ~err_found;
		en_buf[5] = ((sysen_s2 & delay_done[4]) | pg_s2[6]) & ~err_found;
		en_buf[6] = ((sysen_s2 & delay_done[5]) | pg_s2[7]) & ~err_found;
		en_buf[7] = ((sysen_s2 & delay_done[6]) | pg_s2[8]) & ~err_found;
		en_buf[8] = ((sysen_s2 & delay_done[7]) | pg_s2[9]) & ~err_found;
		en_buf[9] = (( ~seq_s2 & sysen_s2 & delay_done[8]) | pg_s2[10]) & ~err_found;
		en_buf[10] = ((sysen_s2 & delay_done[9]) | pg_s2[11]) & ~err_found;
		en_buf[11] = ((sysen_s2 & delay_done[10]) | pg_s2[12]) & ~err_found;
		en_buf[12] = ((sysen_s2 & delay_done[11]) | pg_s2[13]) & ~err_found;
		en_buf[13] = ((sysen_s2 & delay_done[12]) | pg_s2[14]) & ~err_found;
		en_buf[14] = (sysen_s2 & delay_done[13]) & ~err_found;
	end

	// ERR state reset
	always @(posedge clk_in) begin
		clear_err = i2c_clr_err;
	end

	// CPUB clk enables
	always @(posedge clk_in) begin
		cpub_clk_oea = ~cpub_present_n;
		cpub_clk_oeb = ~cpub_present_n;
	end

	// System PWRGOOD
	always @(posedge clk_in) begin
		sysgood_buf = delay_done[14];
		sysgood = sysgood_buf & bmc_software_pg;
		lpc_rst = sysgood_buf;
	end

	// CPU Reset
	always @(posedge clk_in) begin
		cpu_stby_rst_assert = ~en_buf[0];
	end

	// BMC RESETs
	always @(posedge clk_in) begin
		bmc_rst = bmc_vr_pg;
		usbhub_rst = sysgood_buf & bmc_software_pg;
		fan_rst = bmc_vr_pg;
	end

	// debug_in override allows non-BMC control of FPGA
	always @(posedge clk_in) begin
		sysen_buf = sysen | ~debug_in;
		// sysen_buf = ~debug_in;
	end

	// Enable V5_0_DUAL rail
	always @(posedge clk_in) begin
		dual_5v_ctrl = 1'b0;
	end

	// Enable PMC
	always @(posedge clk_in) begin
		pmc_disable_n = 1'b1;
	end

	// Not used
	always @(posedge clk_in) begin
		window_open_n = 1'b0;
	end

	// Generate standard front panel NIC activity indications
	always @(posedge clk_in) begin
		panel_nic1_led_cathode_std = ~(nic1_act_led_n_in & ~nic1_green_led_n);
		panel_nic2_led_cathode_std = ~(nic2_act_led_n_in & ~nic2_green_led_n);
	end

	// Wire up UID request to front panel
	always @(posedge clk_in) begin
		panel_uid_led_std = bmc_uid_led_req;
	end

	// Assign front panel indicators according to BMC status
	always @(posedge clk_in) begin
		if (bmc_software_pg) begin
			panel_nic1_led_cathode = panel_nic1_led_cathode_std;
			panel_nic2_led_cathode = panel_nic2_led_cathode_std;
			panel_uid_led = panel_uid_led_std;
		end else begin
			panel_nic1_led_cathode = bmc_startup_kr[0];
			panel_nic2_led_cathode = bmc_startup_kr[1];
			panel_uid_led = bmc_startup_kr[2];
		end
	end

	// Generate master reset request signals
	always @(posedge clk_in) begin
		master_reset_reqest = ~(panel_reset_in_l & flexver_reset_in_l);
		bmc_system_reset_request_n = ~master_reset_reqest;
	end
	
endmodule
