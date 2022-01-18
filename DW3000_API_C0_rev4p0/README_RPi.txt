###############################################################################
#          Raspberry Pi Build for DW3000 API + Simple Examples                #
###############################################################################
This README file will describe the following:
-> Compiling the code.
-> Installing the DW3000 API shared library.
-> Running a simple example executable.
-> Extra features of the Makefile.
-> Notes of the hardware used.
-> Notes on software package.

###############################################################################
#                        Compiling the code                                   #
###############################################################################
In order to compile the code, you will need the following:
-> The code that accompanies this README.
-> A Raspberry Pi 3 Model B+ with Raspbian OS installed.
-> An SSH connection to the Raspberry Pi.

This section will presume that you have an SSH connection through a command
line terminal to the Raspberry Pi, or you are running a command line terminal
on the Raspberry Pi itself. It also presumes that the reader has prior
knowledge and experience with Linux-based operating systems.

Once the code has been copied over to the Raspberry Pi, navigate to the
../dw3000_api/API directory. Presuming that it is installed into the home
directory of the Raspbian OS, you can do that like so:

$ cd ~/dw3000_api/API

Once you are in this directory, you can compile the shared library and
simple example executables by invoking the Makefile like so:

$ make

This should compile the shared library and all of the simple example
executables.

###############################################################################
#               Installing the DW3000 API Shared Library                      #
###############################################################################
Since the simple example executables use the libdw3000.so shared library during
run-time, the library needs to be installed on to the file system of the
Raspbian OS. This can be achieved by running the following command:

$ sudo make rpi-install

The 'sudo' command is required as the shared library (and symlinks) are
installed to the "/usr/lib" directory of the filesystem. This location
generally has admin-only access.

Once the shared library is installed, the user can create their own programs
and link against it. Details of the APIs available in the shared library can be
found in the source code and in the DW3000 API Guide.

If the user wishes to uninstall the shared library, they can run the following
command:

$ sudo make rpi-uninstall

This will remove the libdw3000 shared library from the "/usr/lib" location in
the filesystem.

###############################################################################
#                  Running a Simple Example Executable                        #
###############################################################################
Once all the code has been compiled and the shared library has been installed
to the filesystem, the simple example executables are ready to be run.

Each of the simple example executables are located in their own specific
directory. For example, the "ex_00a_reading_dev_id" is located here:

../dw3000_api/API/output/ex_00a_reading_dev_id/test_reading_dev_id

The "ex_01a_simple_tx" executable is located here:

../dw3000_api/API/output/ex_01a_simple_tx/test_simple_tx

The same pattern follows for all of the simple example executables.

If you wish to run any of the executables, you just need to invoke it from the
command line like so (presuming you are still in the same directory in which
the code was compiled):

$ output/ex_00a_reading_dev_id/test_reading_dev_id

This produces an output like so:

SPI communication successfully setup.
READ DEV ID
SPI communication successfully setup.
DEV ID OK

This shows that the program successfully read the device ID on the DW3000 IC.

The user can change the output messages if they wish by altering the code for
the desired executable.

###############################################################################
#                    Extra Features of the Makefile                           #
###############################################################################
While the default rule of the makefile is to build all of the simple examples,
it is also possible to specify any combination of examples that you may wish to
compile. This is done like so:

$ make -f Makefile.rpi <list of simple example targets>

The available targets are as follows:
test_reading_dev_id
test_simple_tx
test_tx_sleep
test_tx_sleep_idle_rc
test_tx_sleep_auto
test_tx_sleep_timed
test_simple_tx_sts_sdc
test_simple_tx_pdoa
test_simple_tx_aes
test_simple_rx
test_rx_diag
test_rx_trim
test_simple_rx_sts_sdc
test_simple_rx_pdoa
test_simple_rx_aes
test_tx_wait_resp
test_rx_send_resp
test_tx_wait_resp_int
test_continuous_wave
test_continuous_frame
test_ds_twr_initiator
test_ds_twr_initiator_sts
test_ds_twr_responder
test_ds_twr_responder_sts
test_ds_twr_sts_sdc_initiator
test_ds_twr_sts_sdc_responder
test_ss_twr_initiator
test_ss_twr_initiator_sts
test_ss_twr_initiator_sts_no_data
test_ss_twr_responder
test_ss_twr_responder_sts
test_ss_twr_responder_sts_no_data
test_aes_ss_twr_initiator
test_aes_ss_twr_responder
test_ack_data_tx
test_ack_data_rx
test_ack_data_rx_dbl_buff
test_spi_crc
test_gpio
test_otp_write
test_le_pend_rx
test_le_pend_tx

You can run the following command to build all single-sided two way ranging
examples:

$ make -f Makefile.rpi test_ss_twr_initiator test_ss_twr_initiator_sts \
test_ss_twr_initiator_sts_no_data test_ss_twr_responder \
test_ss_twr_responder_sts test_ss_twr_responder_sts_no_data \
test_aes_ss_twr_initiator test_aes_ss_twr_responder

###############################################################################
#                      Notes on the Hardware Used                             #
###############################################################################
This code has been developed and tested on a particular set of hardware.
Decawave/Qorvo are only supporting the hardware combination described below.
However, this code is meant to be seen as an example of how to interface with
a DW3000 device using an embedded Linux platform. To that end, the user should
use this code as a template of how it may work on their embedded Linux platform
of choice.

That hardware used to develop and test this source code is as follows:
-> A Raspberry Pi Model 3B +
-> A Waveshare ARPI600 Arduino Adapter for Raspberry Pi
   (https://www.waveshare.com/wiki/ARPI600)
-> A DW3000 Arduino Shield (Available from Decawave/Qorvo)

These devices are connected together to form a working connection between the
Raspberry Pi and DW3000 device. SPI communications are used to communicate
between the Raspberry Pi and DW3000 device. The WiringPi library (that comes
pre-installed on the Raspbian OS) is used to handle the SPI communications from
the software.

###############################################################################
#                     Notes on the Software Package                           #
###############################################################################
The Raspbian OS release used when testing these examples is:
$ cat /etc/os-release
PRETTY_NAME="Raspbian GNU/Linux 10 (buster)"
NAME="Raspbian GNU/Linux"
VERSION_ID="10"
VERSION="10 (buster)"
VERSION_CODENAME=buster
ID=raspbian
ID_LIKE=debian
HOME_URL="http://www.raspbian.org/"
SUPPORT_URL="http://www.raspbian.org/RaspbianForums"
BUG_REPORT_URL="http://www.raspbian.org/RaspbianBugs"

While this release of the Raspberry Pi build of the DW3000 C0 API and simple
examples will compile each of the simple examples, Decawave/Qorvo have tested
this limited list of simple examples as working:
 -> ex_00a_reading_dev_id
 -> ex_01a_simple_tx
 -> ex_02a_simple_rx
 -> ex_04a_cont_wave
 -> ex_04b_cont_frame
 -> ex_05a_ds_twr_init/ds_twr_initiator_sts.c
 -> ex_05b_ds_twr_resp/ds_twr_responder_sts.c
 -> ex_06a_ss_twr_initiator/ss_twr_initiator_sts_no_data.c
 -> ex_06b_ss_twr_responder/ss_twr_responder_sts_no_data.c
 -> ex_06a_ss_twr_initiator/ss_twr_initiator_sts.c
 -> ex_06b_ss_twr_responder/ss_twr_responder_sts.c
 -> ex_07a_ack_data_tx
 -> ex_07b_ack_data_rx
 -> ex_14_otp_write

The user is free to trial the rest of the available simple examples, but there
is no support available for these examples on Raspberry Pi as of yet. Please
trial them on the supported STM or Nordic platforms instead.

Errata:
It was found during testing that delayed TX and delayed RX timings for the
SS-TWR and DS-TWR examples (that do not use STS functionality) have incorrect
timings and do not perform very well.

Also, it was seen that the interrupt GPIO line is not configured correctly on
the Raspberry Pi build. This will cause any simple examples that use interrupt
functionality to fail.