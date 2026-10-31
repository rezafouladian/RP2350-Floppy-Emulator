# A2Bench with cc65 (brew install cc65), from this directory:
#   make               build/BENCH.SYSTEM (65C02: an enhanced IIe, IIc, IIgs) and build/A2BENCH.po,
#                      an 800K ProDOS volume (/A2BENCH) holding it
#   make BOOT=<image>  also build/A2BENCH140.po, a 140K disk that boots that image's ProDOS 8
#                      (its blocks 0-1 and PRODOS file) and then BENCH.SYSTEM
#   make smoke         build/BENCH6502.SYSTEM, the NMOS 6502 build for tools/a2emu (40 columns)
# The disks are made with tools/prodos.rs.

ROOT := ../..
PRODOS := $(ROOT)/target/prodos
SRC := a2bench.c gs.s

all: build/A2BENCH.po $(if $(BOOT),build/A2BENCH140.po)

build/BENCH.SYSTEM: $(SRC)
	mkdir -p build
	cl65 -t apple2enh -C apple2enh-system.cfg -O -m build/a2bench.map -o $@ $(SRC)

build/BENCH6502.SYSTEM: $(SRC)
	mkdir -p build
	cl65 -t apple2 -C apple2-system.cfg -O -o $@ $(SRC)

smoke: build/BENCH6502.SYSTEM

$(PRODOS): $(ROOT)/tools/prodos.rs
	mkdir -p $(dir $@)
	rustc --edition 2024 -O $< -o $@

build/A2BENCH.po: build/BENCH.SYSTEM $(PRODOS)
	$(PRODOS) new $@ 1600 A2BENCH
	$(PRODOS) put $@ build/BENCH.SYSTEM

build/A2BENCH140.po: build/BENCH.SYSTEM $(PRODOS)
	$(PRODOS) new $@ 280 A2BENCH --boot "$(BOOT)"
	$(PRODOS) put $@ build/BENCH.SYSTEM

clean:
	rm -rf build

.PHONY: all smoke clean
