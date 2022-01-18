# This Makefile is currently designed to only build the Raspberry Pi build of
# the DW3000 Simple Examples + API code.
# NOTE: This is designed to be run on the Raspbian OS running on a Raspberry
# Pi.

# Build Raspberry Pi target.
all: rpi

# Build the Raspberry Pi target.
rpi:
	$(MAKE) -j 4 -f Makefile.rpi

# Install the libdw3000 shared library to filesystem.
rpi-install:
	$(MAKE) -j 4 -f Makefile.rpi install

# Uninstall the libdw3000 shared library from filesystem.
rpi-uninstall:
	$(MAKE) -j 4 -f Makefile.rpi uninstall

# Clean the Raspberry Pi build.
clean: rpi-clean

# Clean the Raspberry Pi build.
rpi-clean:
	$(MAKE) -f Makefile.rpi clean

.PHONY: clean rpi-clean
