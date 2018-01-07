# This file is part of the Talos™ II system FPGA implementation
#
# © 2017 - 2018 Raptor Engineering, LLC
# All Rights Reserved
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 1) Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer: 2) Redistributions in binary
# form must reproduce the above copyright notice, this list of conditions and the
# following disclaimer in the documentation and/or other materials provided with the
# distribution, and; 3) Neither the name of Raptor Engineering, LLC, nor the names of its
# contributors may be used to endorse or promote products derived from this software
# without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS,
# STATUTORY OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

MAX_FPGA_ROUTE_PASSES = 100

# Default seed
#ARACHNE_PNR_SEED = 1

# Selected seed from fastest placement search
# NOTE: Must be updated every time the Verilog source is modified, no matter how trivially!
# Does not need to be updated if firmware program (C) sources are modified
# 0 automatically uses the best placement result
ARACHNE_PNR_SEED = 0
#ARACHNE_PNR_SEED = 1

YOSYS_ICE40_SIM_LIB = $(shell yosys-config --datdir/ice40/cells_sim.v)

.PRECIOUS: system_fpga_%.int

system_fpga_%.tmg: system_fpga_%.int system_fpga.pcf
	echo "Total path delay: inf ns (0.0 MHz)" > $@
	-icetime -tmd hx1k -p system_fpga.pcf -P vq100 $< > $@ 2>&1

system_fpga_%.int: system_fpga.blif system_fpga.pcf
	echo "" > $@
	-arachne-pnr -s $* -d 1k -P vq100 -m $(MAX_FPGA_ROUTE_PASSES) -p system_fpga.pcf $< -o $@

system_fpga.int: system_fpga_1.tmg system_fpga_2.tmg system_fpga_3.tmg system_fpga_4.tmg system_fpga_5.tmg system_fpga_6.tmg system_fpga_7.tmg system_fpga_8.tmg system_fpga_9.tmg \
		system_fpga_10.tmg system_fpga_11.tmg system_fpga_12.tmg system_fpga_13.tmg system_fpga_14.tmg system_fpga_15.tmg system_fpga_16.tmg system_fpga_17.tmg system_fpga_18.tmg system_fpga_19.tmg \
		system_fpga_20.tmg system_fpga_21.tmg system_fpga_22.tmg system_fpga_23.tmg system_fpga_24.tmg system_fpga_25.tmg system_fpga_26.tmg system_fpga_27.tmg system_fpga_28.tmg system_fpga_29.tmg \
		system_fpga_30.tmg system_fpga_31.tmg system_fpga_32.tmg system_fpga_33.tmg system_fpga_34.tmg system_fpga_35.tmg system_fpga_36.tmg system_fpga_37.tmg system_fpga_38.tmg system_fpga_39.tmg \
		system_fpga_40.tmg system_fpga_41.tmg system_fpga_42.tmg system_fpga_43.tmg system_fpga_44.tmg system_fpga_45.tmg system_fpga_46.tmg system_fpga_47.tmg system_fpga_48.tmg system_fpga_49.tmg \
		system_fpga_50.tmg system_fpga_51.tmg system_fpga_52.tmg system_fpga_53.tmg system_fpga_54.tmg system_fpga_55.tmg system_fpga_56.tmg system_fpga_57.tmg system_fpga_58.tmg system_fpga_59.tmg \
		system_fpga_60.tmg system_fpga_61.tmg system_fpga_62.tmg system_fpga_63.tmg system_fpga_64.tmg
	BEST_TRIAL=0;																														\
	BEST_TRIAL_RESULT=0;																													\
	for trial in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64; do							\
		CURRENT_TRIAL_RESULT=$$(cat system_fpga_$${trial}.tmg | grep "Total path delay" | awk '{print $$6}' | sed 's/(//g');																\
		if [ "$$CURRENT_TRIAL_RESULT" != "" ]; then																									\
			echo "system_fpga_$${trial}.tmg : $$CURRENT_TRIAL_RESULT";																							\
			COMPARISON_RESULT=$$(echo "$$CURRENT_TRIAL_RESULT > $$BEST_TRIAL_RESULT" | bc -l);																			\
			if [ $$COMPARISON_RESULT -eq 1 ]; then																									\
				BEST_TRIAL=system_fpga_$${trial}.tmg;																								\
				BEST_TRIAL_RESULT=$$CURRENT_TRIAL_RESULT;																							\
			fi;																													\
		fi;																														\
	done;																															\
	echo "Fastest result: $$BEST_TRIAL : $$BEST_TRIAL_RESULT";																								\
	cp `echo $$BEST_TRIAL | sed 's/\.tmg/\.int/g'` system_fpga.int;																								\
	cp $$BEST_TRIAL system_fpga.tmg
ifneq ($(ARACHNE_PNR_SEED),0)
	cp system_fpga_$(ARACHNE_PNR_SEED).int system_fpga.int
	cp system_fpga_$(ARACHNE_PNR_SEED).tmg system_fpga.tmg
endif
	cat system_fpga.tmg

system_fpga.ex: system_fpga.int
	icebox_explain system_fpga.int > system_fpga.ex

system_fpga.blif: main.v i2c_slave.v
	yosys -l yosys.log -q -p "synth_ice40 -top system_fpga_top -blif system_fpga.blif" main.v i2c_slave.v

system_fpga.bin: system_fpga.int
	icepack system_fpga.int system_fpga.bin

blank.rom:
	dd if=/dev/zero ibs=1k count=256 | tr "\000" "\377" > blank.rom

system_fpga.rom: blank.rom system_fpga.bin
	cp blank.rom system_fpga.rom
	dd if=system_fpga.bin of=system_fpga.rom conv=notrunc

all: system_fpga.rom

dump_toolchain_info:
	-@echo "================================================================================"
	-@echo "Base system:\t"
	-@echo -n "Architecture:\t"
	-@uname -m 2>/dev/null
	-@echo -n "gcc:\t\t"
	-@gcc -dumpversion 2>/dev/null
	-@echo -n "clang:\t\t"
	-@clang --version 2>/dev/null | head -n 1
	-@echo "\nFPGA toolchain:"
	-@echo -n "Icarus verilog:\t"
	-@iverilog -V 2>/dev/null | head -n 1
	-@echo -n "Yosys:\t\t"
	-@yosys -V 2>/dev/null
	-@echo -n "arachne-pnr:\t"
	-@arachne-pnr -v 2>/dev/null
	-@echo "================================================================================"

test: system_fpga.bin
	iceprog -S system_fpga.bin

flash: system_fpga.bin
	iceprog system_fpga.bin

clean:
	rm -f system_fpga.blif system_fpga.ex system_fpga.int system_fpga.tmg system_fpga_*.int system_fpga_*.tmg system_fpga.bin yosys.log
