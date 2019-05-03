// Copyright © 2017, International Business Machines Corp.
// Copyright © 2017 - 2019 Raptor Engineering, LLC
// All Rights Reserved
//
// See LICENSE file for licensing details

module system_fpga_top
	(
		// FPGA clock
		input wire fpga_clock,

		// General I/O
		input wire sysen,
		output reg sysgood,

		// BMC status
		input wire bmc_boot_phase_in,

		// Enable outputs
		output reg vdda_en,
		output reg vcsa_en,
		output reg vdna_en,
		output reg vioa_en,
		output reg vppab_en,
		output reg vddrab_en,
		output reg vttab_en,
		output reg avdd_en,
		output reg miscio_en,
		output reg atx_en,

		// Power Good inputs
		input wire vdda_pg,
		input wire vcsa_pg,
		input wire vdna_pg,
		input wire vioa_pg,
		input wire vppab_pg,
		input wire vddrab_pg,
		input wire avdd_pg,
		input wire miscio_pg,
		input wire atx_pg,
		input wire bmc_vr_pg,

		// I2C
		inout i2c_scl,
		inout i2c_sda,

		// Resets
		output reg lpc_rst,
		input wire bmc_boot_complete_n,
		output reg bmc_rst,
		output reg usbhub_rst,
		inout cpu_stby_rst,

		// Reserved for future use
		output reg dual_5v_ctrl,
		output reg window_open_n,

		// BMC system reset signalling
		inout bmc_system_reset_request_n,

		// Component disable lines
		input wire ast_video_disable_n,
		input wire audio_power_down,
		input wire mode_set_n,

		// System status lines
		input wire bmc_power_led_req,
		input wire bmc_uid_led_req,
		input wire sata_hdd_act_req,
		inout nic1_act_led_n,
		inout nic2_act_led_n,
		inout nic3_act_led_n,
		inout nic1_link_led_n,
		inout nic2_link_led_n,
		inout nic3_link_led_n,
		input wire nic1_green_led_n,
		input wire nic2_green_led_n,
		input wire nic3_green_led_n,

		// Front panel indicators
		output reg panel_power_led,
		output reg panel_uid_led,
		output reg panel_hdd_led,
		output reg panel_nic1_led_cathode,
		output reg panel_nic2_led_cathode,
		output reg panel_nic3_led_cathode,

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

	// The reset line from FlexVer™ requires a pullup to 3.3V
	wire flexver_reset_req_l;
	SB_IO #(
		.PIN_TYPE(6'b000001),
		.PULLUP(1'b1)
	) flexver_reset_in_l_io (
		.PACKAGE_PIN(flexver_reset_in_l),
		.D_IN_0(flexver_reset_req_l)
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
	wire nic3_act_led_n_in;

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
	) nic3_act_led_n_io (
		.PACKAGE_PIN(nic3_act_led_n),
		.OUTPUT_ENABLE(1'b0),
		.D_OUT_0(1'b1),
		.D_IN_0(nic3_act_led_n_in)
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
	SB_IO #(
		.PIN_TYPE(6'b101001),
		.PULLUP(1'b1)
	) nic3_link_led_n_io (
		.PACKAGE_PIN(nic3_link_led_n),
		.OUTPUT_ENABLE(~nic3_green_led_n),
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

	parameter fpga_version = 8'h01;
	parameter vendor_id1 = 8'h52;
	parameter vendor_id2 = 8'h43;
	parameter vendor_id3 = 8'h53;
	parameter vendor_id4 = 8'h20;
	parameter RAIL_SIZE = 15;
	reg [RAIL_SIZE - 1:0] en_buf = 0;
	reg [RAIL_SIZE - 1:0] pg_buf = 0;
	reg sysgood_buf = 1'b0;
	wire clk_in;
	wire clk_in_fpga;
	wire clk_in_ring;
	wire stdby_sed = 1'b0;
	reg sysen_buf = 1'b0;
	reg atx_force_enable = 1'b0;
	reg mfr_force_enable = 1'b0;
	reg mfr_force_cpub_present = 1'b0;
	reg atx_debug_force_enable = 1'b0;
	reg reg_debug_force_enable = 1'b0;
	reg atx_en_lockout = 1'b0;
	reg invert_sata_hdd_act_req = 1'b0;
	parameter railarray_0 = {RAIL_SIZE{1'b0}};
	parameter railarray_1 = {RAIL_SIZE{1'b1}}; 	// synchronizing signals
	reg [RAIL_SIZE - 1:0] pg_s1 = {RAIL_SIZE{1'b0}};
	reg [RAIL_SIZE - 1:0] pg_s2 = {RAIL_SIZE{1'b0}};
	reg [RAIL_SIZE - 1:0] wait_err_detail = {RAIL_SIZE{1'b0}};
	reg sysen_s1 = 1'b0;
	reg sysen_s2 = 1'b0;
	reg [RAIL_SIZE - 1:0] delay_done = {RAIL_SIZE{1'b0}};
	reg [26:0] w_count = 0;
	reg [16:0] d_count = 0; 	// at 4.16MHz, w_count(23) being one means approximately 100ms have passed, good for checking watchdog between EN and PG
					// d_count(16) being one means approximately 15ms have passed, good enough for delay betwen one rail and the next
	reg wait_err = 1'b0;
	reg operation_err = 1'b0;
	reg err_found = 1'b0;
	reg err_found_s1 = 1'b0;
	reg clear_err = 1'b0;
	reg master_reset_reqest = 1'b0;

	// I2C signals
	wire i2c_read_req;
	reg [7:0] i2c_data_to_master = 8'b00000000;
	wire [7:0] i2c_data_from_master;
	wire [7:0] i2c_write_cycle_count;
	wire i2c_data_valid;
	wire i2c_rst = 1'b0;
	reg [7:0] i2c_reg_cur = 8'b00000000;
	parameter i2c_addr = 7'b0110001;
	parameter i2c_clr_err_addr = 8'b00000011;
	parameter i2c_pg_reg_addr1 = 8'b00000101;
	parameter i2c_pg_reg_addr2 = 8'b00000110;
	parameter i2c_status_reg_addr = 8'b00000111;
	parameter i2c_pwr_en_stat_reg_addr1 = 8'b00001000;
	parameter i2c_pwr_en_stat_reg_addr2 = 8'b00001001;
	parameter i2c_pg_stat_reg_addr1 = 8'b00001010;
	parameter i2c_pg_stat_reg_addr2 = 8'b00001011;
	parameter i2c_version_reg_addr = 8'b00000000;
	parameter i2c_vendor_id_reg_addr1 = 8'b00001100;
	parameter i2c_vendor_id_reg_addr2 = i2c_vendor_id_reg_addr1 + 1;
	parameter i2c_vendor_id_reg_addr3 = i2c_vendor_id_reg_addr1 + 2;
	parameter i2c_vendor_id_reg_addr4 = i2c_vendor_id_reg_addr1 + 3;
	parameter i2c_led_override_reg_addr = 8'b00010000;
	parameter i2c_led_config_reg_addr = 8'b00010001;
	parameter i2c_seq_fail_stat_reg_addr1 = 8'b00011000;
	parameter i2c_seq_fail_stat_reg_addr2 = 8'b00011001;
	parameter i2c_system_override_reg_addr = 8'b00110011;
	reg [15:0] i2c_pg_reg = 0;
	reg i2c_clr_err = 1'b0;
	reg host_clr_err = 1'b0;
	reg [7:0] i2c_write_reg_latch = 0;

	// Front panel control signals
	wire panel_nic1_led_cathode_std;
	wire panel_nic2_led_cathode_std;
	wire panel_nic3_led_cathode_std;
	wire panel_power_led_std;
	wire panel_uid_led_std;
	wire panel_hdd_led_std;
	reg [2:0] bmc_startup_kr = 3'b000;
	reg [2:0] bmc_startup_fader = 3'b000;
	reg [2:0] bmc_startup_staggered_fader = 3'b000;
	reg bmc_startup_staggered_fader_common = 1'b0;
	reg hostboot_startup_fader_common_low_internal = 1'b0;
	reg hostboot_startup_fader_common_low = 1'b0;
	reg hostboot_startup_fader_common_high = 1'b0;
	reg [7:0] led_override_request = 8'b00000000;

	// Divide input 8MHz clock down to 1MHz
	reg [2:0] fpga_clock_divider = 0;
	always @(posedge fpga_clock) begin
		fpga_clock_divider = fpga_clock_divider + 1;
	end
	assign clk_in_fpga = fpga_clock_divider[2];

	// Divide 4MHz clock down to 488Hz, 122Hz, and 6Hz, respectively
	wire timer_clk_2;
	wire timer_clk_3;
	wire timer_clk_4;
	reg [16:0] timer_clk_counter = 0;
	always @(posedge clk_in) begin
		timer_clk_counter <= timer_clk_counter + 1;
	end
	assign timer_clk_2 = timer_clk_counter[12];
	assign timer_clk_3 = timer_clk_counter[14];
	assign timer_clk_4 = timer_clk_counter[16];

	assign clk_in = clk_in_fpga;

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
		.data_from_master(i2c_data_from_master),
		.write_cycle_count(i2c_write_cycle_count)
	);

	// Generate BMC startup "Knight Rider" display for front panel
	reg [1:0] bmc_startup_kr_state = 0;
	always @(posedge timer_clk_4) begin
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

	// Generate fading lamp test for front panel
	reg [5:0] fader_pwm_level = 0;
	reg [6:0] fader_pwm_internal_counter = 0;
	always @(posedge timer_clk_2) begin
		fader_pwm_internal_counter = fader_pwm_internal_counter + 1;
		if (fader_pwm_internal_counter >= 64) begin
			fader_pwm_level = 63 - (fader_pwm_internal_counter - 64);
		end else begin
			fader_pwm_level = fader_pwm_internal_counter;
		end

		if (fader_pwm_internal_counter == 0) begin
			fader_sequence_step = fader_sequence_step + 1;
			if (fader_sequence_step > 2) begin
				fader_sequence_step = 0;
			end
		end
	end

	reg [1:0] fader_sequence_step = 0;
	reg [5:0] fader_pwm_counter = 0;
	always @(posedge clk_in) begin
		fader_pwm_counter = fader_pwm_counter + 1;
		if (fader_pwm_counter >= fader_pwm_level) begin
			bmc_startup_fader = 3'b000;
		end else begin
			bmc_startup_fader = 3'b111;
		end
		if (fader_pwm_counter >= fader_pwm_level) begin
			bmc_startup_staggered_fader_common = 1'b0;
		end else begin
			bmc_startup_staggered_fader_common = 1'b1;
		end
		if (fader_pwm_counter >= 32) begin
			hostboot_startup_fader_common_low_internal = 1'b0;
		end else begin
			hostboot_startup_fader_common_low_internal = 1'b1;
		end
		if (fader_pwm_counter >= (fader_pwm_level >> 1) + 32) begin
			hostboot_startup_fader_common_high = 1'b0;
		end else begin
			hostboot_startup_fader_common_high = 1'b1;
		end
	end

	always @(posedge clk_in) begin
		if (fader_sequence_step == 0) begin
			bmc_startup_staggered_fader[0] = bmc_startup_staggered_fader_common;
			bmc_startup_staggered_fader[1] = 1'b0;
			bmc_startup_staggered_fader[2] = 1'b0;
		end else if (fader_sequence_step == 1) begin
			bmc_startup_staggered_fader[0] = 1'b0;
			bmc_startup_staggered_fader[1] = bmc_startup_staggered_fader_common;
			bmc_startup_staggered_fader[2] = 1'b0;
		end else begin
			bmc_startup_staggered_fader[0] = 1'b0;
			bmc_startup_staggered_fader[1] = 1'b0;
			bmc_startup_staggered_fader[2] = bmc_startup_staggered_fader_common;
		end

		if (fader_sequence_step == 0) begin
			hostboot_startup_fader_common_low = hostboot_startup_fader_common_high;
		end else if (fader_sequence_step == 1) begin
			hostboot_startup_fader_common_low = hostboot_startup_fader_common_low_internal;
		end else begin
			hostboot_startup_fader_common_low = hostboot_startup_fader_common_low_internal;
		end
	end

	// Determine BMC boot phase
	reg [1:0] bmc_boot_phase = 0;
	always @(posedge clk_in) begin
		if (!bmc_rst || (bmc_boot_complete_n && (bmc_boot_phase == 2))) begin
			bmc_boot_phase = 0;
		end else begin
			// While the BMC is offline, bmc_boot_phase_in indicates U-Boot / Kernel boot phase (1 / 0, respectively)
			if (bmc_boot_phase == 0) begin
				if (!bmc_boot_phase_in) begin
					bmc_boot_phase = 1;
				end
			end
			if (!bmc_boot_complete_n) begin
				bmc_boot_phase = 2;
			end
		end
	end

	// BMC initial startup watchdog
	reg [8:0] bmc_watchdog_counter = 0;
	reg bmc_watchdog_reset = 1'b0;
	always @(posedge timer_clk_4) begin
		if (bmc_rst && (bmc_boot_phase == 0)) begin
			bmc_watchdog_counter <= bmc_watchdog_counter + 1;
		end else begin
			bmc_watchdog_counter <= 0;
		end

		if (bmc_watchdog_counter[8]) begin
			bmc_watchdog_reset = 1'b1;
		end else begin
			bmc_watchdog_reset = 1'b0;
		end
	end

	assign i2c_rst = 1'b0;
	// Handle I2C
	always @(posedge clk_in) begin
		i2c_clr_err <= 1'b0;

		if (i2c_data_valid == 1'b1) begin
			// data from master is register to be read
			i2c_reg_cur <= i2c_data_from_master;
	
			// pulse clear err signal if i2c master reads register 0x03
			if (((i2c_data_from_master) == i2c_clr_err_addr)) begin
				i2c_clr_err <= 1'b1;
			end

			// handle write setup
			if (i2c_write_cycle_count == 1) begin
				i2c_write_reg_latch <= i2c_data_from_master;
			end else if (i2c_write_cycle_count == 2) begin
				case (i2c_write_reg_latch)
					i2c_led_override_reg_addr: begin
						led_override_request <= i2c_data_from_master;
					end
					i2c_led_config_reg_addr: begin
						invert_sata_hdd_act_req <= i2c_data_from_master[0];
					end
					i2c_system_override_reg_addr: begin
						atx_force_enable <= i2c_data_from_master[0];
						mfr_force_enable <= i2c_data_from_master[1];
						mfr_force_cpub_present <= i2c_data_from_master[2];
						atx_debug_force_enable <= i2c_data_from_master[3];
						reg_debug_force_enable <= i2c_data_from_master[4];
					end
				endcase
			end
		end
		else if (i2c_read_req == 1'b1) begin
			i2c_reg_cur <= i2c_reg_cur + 1;
		end
		case (i2c_reg_cur)
			// FIXME
			// Temporarily disabled to save die area
			// i2c_pg_reg_addr1: begin
			// 	i2c_data_to_master <= i2c_pg_reg[15:8];
			// end
			// i2c_pg_reg_addr2: begin
			// 	i2c_data_to_master <= i2c_pg_reg[7:0];
			// end
			i2c_status_reg_addr: begin
				i2c_data_to_master <= {~mode_set_n, ~ast_video_disable_n, 0, wait_err, operation_err, err_found, sysen_buf, sysgood_buf};
			end
			i2c_pwr_en_stat_reg_addr1: begin
				i2c_data_to_master <= en_buf[7:0];
			end
			i2c_pwr_en_stat_reg_addr2: begin
				i2c_data_to_master <= {1'b0, en_buf[RAIL_SIZE-1:8]};
			end
			i2c_pg_stat_reg_addr1: begin
				i2c_data_to_master <= pg_buf[7:0];
			end
			i2c_pg_stat_reg_addr2: begin
				i2c_data_to_master <= {1'b0, pg_buf[RAIL_SIZE-1:8]};
			end
			i2c_seq_fail_stat_reg_addr1: begin
				i2c_data_to_master <= wait_err_detail[7:0];
			end
			i2c_seq_fail_stat_reg_addr2: begin
				i2c_data_to_master <= {1'b0, wait_err_detail[RAIL_SIZE-1:8]};
			end
			i2c_led_override_reg_addr: begin
				i2c_data_to_master <= led_override_request;
			end
			i2c_led_config_reg_addr: begin
				i2c_data_to_master <= {7'b0, invert_sata_hdd_act_req};
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
			i2c_system_override_reg_addr: begin
				i2c_data_to_master <= {audio_power_down, 6'b000000, atx_force_enable};
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
		err_found_s1 <= err_found;
		if ((sysen_s1 == 1'b1 ) && (sysen_buf == 1'b0)) begin
			host_clr_err <= 1'b1;
		end else begin
			host_clr_err <= 1'b0;
		end
		if ((clear_err == 1'b1)) begin
			wait_err <= 1'b0;
			wait_err_detail <= {RAIL_SIZE{1'b0}};
			operation_err <= 1'b0;
			err_found <= 1'b0;
			w_count <= {27{1'b0}};
			d_count <= {17{1'b0}};
		end
		else if ((sysen_s2 == 1'b0 || err_found == 1'b1)) begin
			w_count <= {27{1'b0}};
			d_count <= {17{1'b0}};
			delay_done <= 0;
		end
		else if ((pg_s2[0] == 1'b1 && en_buf[0] == 1'b1 && delay_done[0] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[0] <= 1'b1;
			end
		end
		else if ((pg_s2[1] == 1'b1 && en_buf[1] == 1'b1 && delay_done[1] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[1] <= 1'b1;
			end
		end
		else if ((pg_s2[2] == 1'b1 && en_buf[2] == 1'b1 && delay_done[2] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[2] <= 1'b1;
			end
		end
		else if ((pg_s2[3] == 1'b1 && en_buf[3] == 1'b1 && delay_done[3] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[3] <= 1'b1;
			end
		end
		else if ((pg_s2[4] == 1'b1 && en_buf[4] == 1'b1 && delay_done[4] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[4] <= 1'b1;
			end
		end
		else if ((pg_s2[5] == 1'b1 && en_buf[5] == 1'b1 && delay_done[5] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[5] <= 1'b1;
			end
		end
		else if ((pg_s2[6] == 1'b1 && en_buf[6] == 1'b1 && delay_done[6] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[6] <= 1'b1;
			end
		end
		else if ((pg_s2[7] == 1'b1 && en_buf[7] == 1'b1 && delay_done[7] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[7] <= 1'b1;
			end
		end
		else if ((pg_s2[8] == 1'b1 && en_buf[8] == 1'b1 && delay_done[8] == 1'b0)) begin
			w_count <= {27{1'b0}};
			d_count <= d_count + 1;
			if ((d_count[16] == 1'b1)) begin
				d_count <= {17{1'b0}};
				delay_done[8] <= 1'b1;
			end
		end

		// Error Checks
		// Check time between Enables going high and PGOODs arriving. Error out after 100ms
		// ATX power good is special.  According to the ATX specification, ATX power good can
		// take up to 500ms to assert.  Wait for 800ms to be safe.
		else if ((pg_s2[0] == 1'b0 && en_buf[0] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[26] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[1] == 1'b0 && en_buf[1] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[2] == 1'b0 && en_buf[2] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[3] == 1'b0 && en_buf[3] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[4] == 1'b0 && en_buf[4] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[5] == 1'b0 && en_buf[5] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[6] == 1'b0 && en_buf[6] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[7] == 1'b0 && en_buf[7] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		else if ((pg_s2[8] == 1'b0 && en_buf[8] == 1'b1)) begin
			w_count <= w_count + 1;
			if ((w_count[23] == 1'b1)) begin
				w_count <= {27{1'b0}};
				wait_err <= 1'b1;
			end
		end
		if ((( ~(delay_done & ~pg_s2)) != railarray_1)) begin
			operation_err <= 1'b1;
		end
		if (((wait_err | operation_err) == 1'b1 && clear_err == 1'b0)) begin
			err_found <= 1'b1;
		end else begin
			i2c_pg_reg[9:0] <= pg_s2[9:0];
		end

		if (err_found && ~err_found_s1) begin
			wait_err_detail = en_buf ^ pg_buf;
		end
	end
	
	// Assign Ports to Enables
	always @(posedge clk_in) begin
		atx_en = ~(en_buf[0] | atx_debug_force_enable);
		miscio_en = en_buf[1] | reg_debug_force_enable;
		vdna_en = en_buf[2] | reg_debug_force_enable;
		avdd_en = en_buf[3] | reg_debug_force_enable;
		vioa_en = en_buf[4] | reg_debug_force_enable;
		vdda_en = en_buf[5] | reg_debug_force_enable;
		vcsa_en = en_buf[6] | reg_debug_force_enable;
		vppab_en = en_buf[7] | reg_debug_force_enable;
		vddrab_en = en_buf[8] | reg_debug_force_enable;
		vttab_en = en_buf[8] | reg_debug_force_enable;
	end

	// Assign Ports to PGood buffer
	always @(posedge clk_in) begin
		pg_buf[0] = atx_pg;
		pg_buf[1] = miscio_pg;
		pg_buf[2] = vdna_pg;
		pg_buf[3] = avdd_pg;
		pg_buf[4] = vioa_pg;
		pg_buf[5] = vdda_pg;
		pg_buf[6] = vcsa_pg;
		pg_buf[7] = vppab_pg;
		pg_buf[8] = vddrab_pg;
	end

	// Enable outputs
	// Shut everything off if an error has occurred
	// Otherwise, if system enable is up, then enable short delay is done after previous rail
	// Otherwise, disable after next rail goes down
	always @(posedge clk_in) begin
		en_buf[0] = ((sysen_s2 | pg_s2[1]) & ~err_found & ~atx_en_lockout);
		en_buf[1] = (((sysen_s2 & delay_done[0]) | pg_s2[2]) & ~err_found);
		en_buf[2] = (((sysen_s2 & delay_done[1]) | pg_s2[3]) & ~err_found);
		en_buf[3] = (((sysen_s2 & delay_done[2]) | pg_s2[4]) & ~err_found);
		en_buf[4] = (((sysen_s2 & delay_done[3]) | pg_s2[5]) & ~err_found);
		en_buf[5] = (((sysen_s2 & delay_done[4]) | pg_s2[6]) & ~err_found);
		en_buf[6] = (((sysen_s2 & delay_done[5]) | pg_s2[7]) & ~err_found);
		en_buf[7] = (((sysen_s2 & delay_done[6]) | pg_s2[8]) & ~err_found);
		en_buf[8] = ((sysen_s2 & delay_done[7]) & ~err_found);
	end

	// ERR state reset
	always @(posedge clk_in) begin
		clear_err = i2c_clr_err | host_clr_err;
	end

	// System PWRGOOD
	always @(posedge clk_in) begin
		sysgood_buf = delay_done[8];
		sysgood = sysgood_buf & ~bmc_boot_complete_n;
		lpc_rst = sysgood_buf;
	end

	// CPU Reset
	always @(posedge clk_in) begin
		cpu_stby_rst_assert = ~en_buf[0];
	end

	// BMC RESETs
	always @(posedge clk_in) begin
		bmc_rst = bmc_vr_pg & ~bmc_watchdog_reset;
		usbhub_rst = sysgood_buf & ~bmc_boot_complete_n;
	end

	// atx_force_enable override allows non-BMC control of FPGA
	always @(posedge clk_in) begin
		sysen_buf = sysen | atx_force_enable;
	end

	// Enable V5_0_DUAL rail
	always @(posedge clk_in) begin
		dual_5v_ctrl = 1'b0;
	end

	// Not used
	always @(posedge clk_in) begin
		window_open_n = 1'b0;
	end

	// Generate standard front panel NIC activity indications
	always @(posedge clk_in) begin
		panel_nic1_led_cathode_std = ~(nic1_act_led_n_in & ~nic1_green_led_n);
		panel_nic2_led_cathode_std = ~(nic2_act_led_n_in & ~nic2_green_led_n);
		panel_nic3_led_cathode_std = ~(nic3_act_led_n_in & ~nic3_green_led_n);
	end

	// Wire up power and UID requests to front panel
	always @(posedge clk_in) begin
		panel_power_led_std = bmc_power_led_req;
		panel_uid_led_std = bmc_uid_led_req;
		if (invert_sata_hdd_act_req) begin
			panel_hdd_led_std = sata_hdd_act_req;
		end else begin
			panel_hdd_led_std = ~sata_hdd_act_req;
		end
	end

	// Assign front panel indicators according to BMC status
	reg panel_power_led_req = 1'b0;
	reg panel_uid_led_req = 1'b0;
	reg panel_hdd_led_req = 1'b0;
	reg bmc_startup_indicators_active = 1'b0;
	always @(posedge clk_in) begin
		if (bmc_boot_phase == 0) begin
			// U-Boot phase
			panel_nic1_led_cathode = ~bmc_startup_staggered_fader[0];
			panel_nic2_led_cathode = ~bmc_startup_staggered_fader[1];
			panel_nic3_led_cathode = ~bmc_startup_staggered_fader[2];
			panel_power_led_req = ~bmc_startup_staggered_fader[2];
			panel_uid_led_req = bmc_startup_staggered_fader[2];
			panel_hdd_led_req = ~bmc_startup_staggered_fader[0];
			bmc_startup_indicators_active = 1'b1;
		end else if (bmc_boot_phase == 1) begin
			// Kernel phase
			panel_nic1_led_cathode = ~bmc_startup_fader[0];
			panel_nic2_led_cathode = ~bmc_startup_fader[1];
			panel_nic3_led_cathode = ~bmc_startup_fader[2];
			panel_power_led_req = ~bmc_startup_fader[2];
			panel_uid_led_req = bmc_startup_fader[2];
			panel_hdd_led_req = ~bmc_startup_fader[0];
			bmc_startup_indicators_active = 1'b1;
		end else if (bmc_boot_phase == 2) begin
			if (led_override_request != 0) begin
				if (led_override_request[5]) begin
					panel_nic1_led_cathode = ~(led_override_request[0] & hostboot_startup_fader_common_high);
					panel_nic2_led_cathode = ~(led_override_request[1] & hostboot_startup_fader_common_high);
					panel_nic3_led_cathode = ~(led_override_request[4] & hostboot_startup_fader_common_high);
					panel_power_led_req = panel_power_led_std;
					panel_uid_led_req = ~(led_override_request[3] & hostboot_startup_fader_common_high);
					panel_hdd_led_req = ~(led_override_request[2] & hostboot_startup_fader_common_high);
					bmc_startup_indicators_active = 1'b1;
				end else begin
					panel_nic1_led_cathode = ~(led_override_request[0] & hostboot_startup_fader_common_low);
					panel_nic2_led_cathode = ~(led_override_request[1] & hostboot_startup_fader_common_low);
					panel_nic3_led_cathode = ~(led_override_request[4] & hostboot_startup_fader_common_low);
					panel_power_led_req = panel_power_led_std;
					panel_uid_led_req = ~(led_override_request[3] & hostboot_startup_fader_common_low);
					panel_hdd_led_req = ~(led_override_request[2] & hostboot_startup_fader_common_low);
					bmc_startup_indicators_active = 1'b1;
				end
			end else begin
				panel_nic1_led_cathode = panel_nic1_led_cathode_std;
				panel_nic2_led_cathode = panel_nic2_led_cathode_std;
				panel_nic3_led_cathode = panel_nic3_led_cathode_std;
				panel_power_led_req = panel_power_led_std;
				panel_uid_led_req = panel_uid_led_std;
				panel_hdd_led_req = panel_hdd_led_std;
				bmc_startup_indicators_active = 1'b0;
			end
		end else begin
			panel_nic1_led_cathode = 1'b1;
			panel_nic2_led_cathode = 1'b1;
			panel_nic3_led_cathode = 1'b1;
			panel_power_led_req = panel_power_led_std;
			panel_uid_led_req = panel_uid_led_std;
			panel_hdd_led_req = panel_hdd_led_std;
			bmc_startup_indicators_active = 1'b0;
		end

		// The SuperMicro chassis front panel has some interesting quirks
		// A bidirectional LED is used to either indicate UID or Fault status
		// When chassis power is off, driving UID high will actually light the Fault LED
		// Conversely, when chassis power is on, driving UID high will turn off both LEDs
		// unless the fan failure signal is asserted on the mainboard.
		//
		// Ensure that the Fault LED is not lit when chassis power is off by inverting
		// the polarity of the front panel UID signal when ATX power good is deasserted.
		if (atx_pg == 1'b1) begin
			panel_uid_led = panel_uid_led_req;
		end else begin
			panel_uid_led = ~panel_uid_led_req;
		end

		// Front panel power LED
		panel_power_led = panel_power_led_req;

		// Front panel disk activity LED
		// Keep LED off if system power is off and BMC has booted, otherwise
		// pass through the SATA controller activity LED signal.
		if ((atx_pg == 1'b1) || (bmc_startup_indicators_active == 1'b1)) begin
			panel_hdd_led = panel_hdd_led_req;
		end else begin
			panel_hdd_led = 1'b1;
		end
	end

	// Generate master reset request signals
	always @(posedge clk_in) begin
		master_reset_reqest = ~(panel_reset_in_l & flexver_reset_req_l);
		bmc_system_reset_request_n = ~master_reset_reqest;
	end
	
endmodule
