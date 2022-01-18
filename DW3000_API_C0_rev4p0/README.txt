=============================================================================
README.txt - Decawave/Qorvo DW3000 Application Programming Interface (API)
=============================================================================

The DW3000 API package is composed of the following folders:

    - API/Src/decadriver:
    
      Driver for DW3000 UWB transceiver IC. Details about each function can
      be found in DW3000 API Guide.

    - API/Src/examples:
    
      A set of individual (simple) examples showing how to achieve different
      functionalities, e.g. sending a frame, receiving a frame, putting the
      DW3000 to sleep, two-way ranging.  The emphasis of theses examples is
      to be readable with explanatory notes to explain and show how the main
      features of the DW3000 work and can be used.

    - Libraries, Linkers:
    
      Hardware abstraction layer (system start-up code and peripheral
      drivers) for ARM Cortex-M and ST STM32 F1 processors. Provided by ST
      Microelectronics.
      
    - API/Src/platform:

      Platform dependent implementation of low-level features (IT management,
      mutex, sleep, etc).

Please refer to DW3000 API Guide accompanying this package for more details
about provided API and examples.

NOTE: The DW3000 API/driver code included in this package is an unbundled
      version of the DW3000 API/driver. This version may be different to
      (generally by being newer than) those bundled with Decawave's other
      products. This particular release covers the DW3000 C0 hardware
      versions:
	  
	  //!< DW3000 MP C0 (non PDOA) silicon device ID
	  #define DWT_C0_DEV_ID (0xDECA0302)
	  //!< DW3000 MP C0 (with PDOA) silicon device ID
	  #define DWT_C0_PDOA_DEV_ID (0xDECA0312)

=============================================================================
=                                                                           =
=                               Release Notes                               =
=                                                                           =
=============================================================================

=============================================================================
Package v4.0 / Driver v04.00.00  (29th July 2020)
=============================================================================

a) Updated release of DW3000 Driver and API for MP/C0 device, with some simple
example projects.

API + Simple Examples Changes:
- Have added build options for both Raspberry Pi and Nordic platforms.
  Additional information can be found in the respective README files for those
  platforms.
- Added extra compilations flags (“-Wextra”, “-pedantic”, etc.) to help remove
  warnings and such.
- Header file includes have been modified to use "#include <foo.h>" format
  where appropriate.
- The following typedefs have been changed throughout the code:
  -	uint8 -> uint8_t
  -	uint16 -> uint16_t
  -	uint32 -> uint32_t
  -	uint64 -> uint64_t
  -	int8 -> int8_t
  -	int16 -> int16_t
  -	int32 -> int32_t
  -	int64 -> int64_t
- deca_types.h is still included in the software release, but code is compiled
  with stdint.h as the first preference in defining the typedefs listed above.
  The typedefs in deca_types.h are to be used as a fallback in the cases that
  the above typedefs are not defined.
- Most simple examples now use a single DW3000 configuration structure which is
  located in config_options.c. Previously, there was separate structures within
  each of the simple examples.
- All examples that use STS functionality now program the STS key as described
  in the IEEE 802.15.4z amendment (Appendix H).
- All dwt_configure() calls within the simple examples now stall in an infinite
  while loop if there is a configuration error.
- Delayed TX and delayed RX timings have changed in all SS-TWR and DS-TWR
  examples in order to allow the same code to run across STM/Nordic/RPi
  platforms. This is due to different SPI and MCU clock speeds across the
  platforms.
- deca_param_types.h and deca_params_init.c have now been removed from the
  build as they are no longer needed. Any parts that were required have been
  migrated to other source files.

Simple Examples Updates:
Modified:
02h - PDOA RX - Added error interrupt handler to catch errors missed in
                previous version.

Added:
15 - LE Pend - New example added that shows how to use the low-energy (LE)
               functionality paired with the frame pending field of a frame.
			   Please see the the DW3000 User Manual for more details of this
			   functionality.
			   

c) Notes on major differences between DW3000 (MP/C0) CR3 and DW3000 (MP/C0)
    CR4, regarding APIs:
Changed:
- dwt_starttx() modified to correctly handle a late invocation case not flagged by HPDWARN.
- dwt_configure() and dwt_restoreconfig() APIs will now load DGC configuration
  from OTP if available.
- dwt_isr() API has been updated to allow for a greater range of interrupts to
  be used. It has also been redesigned to be more efficient than previous
  version. The number of SPI reads have been reduced, and the callback data 
  structure has been updated. The callback data does not have fctrl bytes any
  longer, it is up to the host to read the data frame and any fctrl bytes in the 
  callback on reception of a good frame. The reading of system status regiser
  is also limited and it is again up to the host to read this in the callback.
- dwt_setinterrupt() now allows for interrupt masks to be set for both the
  SYS_ENABLE_LO & SYS_ENABLE_HI registers. Previously it only allowed masks to
  be set for the SYS_ENABLE_LO register.
- dwt_run_pgfcal() has an added fix for fast SPI writes. It previously failed
  to calibrate for fast SPI rates.
- the CCM nonce formating has been fixed, the CCM encryption/decryption works
  correctly.
- the XTAL trim range in dwt_setxtltrim() has been reduced to 64 steps, as
  using more than 64 steps will disimprove the performance of the device.

New:
- dwt_configurestsmode() is a new function that has been added to the API to
  allow for changing of the STS mode (mostly to switch between STS No Data and
  other modes) during run-time.
- dwt_configuresfdtype() is a new API added to set the SFD type.
- deca_usleep() added to allow for micro-second delays. Please note that each
  platform will have different implementations of this. Thus, timings may be
  slightly different between platforms.

d) the following default TX power/bandwidth configurations should be used:
Channel 5: 
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0xFDFDFDFD,     /* TX power. */
    0x0             /* PG Count */
};
Channel 9:
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0xFEFEFEFE,     /* TX power. */
    0x0             /* PG Count */
};

=============================================================================
Package v3.0 / Driver v03.04.01  (10th April 2020)
=============================================================================

a) Updated release of DW3000 Driver and API for MP/C0 device, with some simple
example projects.

Modified:
01b - TX Sleep - Configuration is now restored upon wakup from sleep.
01b - TX Sleep IDLERC - Configuration is now restored upon wakup from sleep.
01c - TX Sleep Auto - Configuration is now restored upon wakup from sleep.
01d - TX Timed Sleep - Configuration is now restored upon wakup from sleep.
01h - Simple TX PDOA - Configuration options have been optimised.
02h - PDOA RX - Configuration options have been optimised.
05a - DS-TWR Initiator (with STS) - Fixes and improvements made throughout.
05b - DS-TWR Responder (with STS) - Fixes and improvements made throughout.
06a - SS-TWR Initiator (with STS) - Fixes and improvements made throughout.
06b - SS-TWR Responder (with STS) - Fixes and improvements made throughout.

Minor code refactoring throughout.
Notes/comments have been reviewed and changed throughout.
Some functions that are copied throughout the examples have been consolidated
into the shared functions.

Added:
06a/06b SS-TWR with STS No Data (STS Mode 3) - version of SS-TWR example
    program that uses the STS No Data packets for ranging. Please see code
    and IEEE 802.15.4z for more details.
14 - OTP Write - New example added that shows how to write to (and verify) OTP
    memory.

c) Notes on major diffrences between DW3000 (MP/C0) CR2 and DW3000 (MP/C0)
    CR3, regarding APIs:
Changed:
- Refactoring made throughout regarding the comments described the APIS.
- dwt_initialise: Setting of XTAL trim has changed slightly.
- dwt_configuretxrf: PG count is now an added parameter to the structure
  passed to this function.
- dwt_configmrxlut: DGC tables have been updated.
- dwt_restoreconfig: DGC gear tables are now 'kicked' on wakeup. Channel 5 & 9
  configurations are also applied.
- dwt_readdiagnostics: 'magic' numbers are now removed and defined. TDOA data
  is now recorded correctly.
- dwt_isr: Interrupt routine has been updated to check for STS Mode 3
  (No Data) packets before other STS frames.
- dwt_setxtaltrim: XTAL trim value is now saved locally by the driver.
- dwt_wait_aes_poll: Function is no longer exported for use by the API.
- dwt_update_nonce_CCM: Function is no longer exported for use by the API.
- dwt_update_nonce_GCM: Function is no longer exported for use by the API.

New:
- dwt_readtdoa: Read TDOA value from IC.
- dwt_configure_le_address: Configures the desired LE address for frame
  pending functionality.

Deleted:
- dwt_loadopsettabfromotp
- dwt_calcpowertempadj

d) the following default TX power/bandwidth configurations should be used:
Channel5: 
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0xFDFDFDFD,     /* TX power. */
    0x0             /* PG Count */
};
Channel9:
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0xFEFEFEFE,     /* TX power. */
    0x0             /* PG Count */
};

=============================================================================
Package v2.0 / Driver v03.02.00  (11th February 2020)
=============================================================================

a) Updated release of DW3000 Driver and API for MP/C0 device, with some
simple example projects.

b) Following simple examples have are being modified/added since the last
	release:
Modified:
01a - Simple TX - Minor code refactoring.
01i/02i - Simple TX/RX with AES - minor update to the dwt_aes_config_t
	structure.
04a - Continuous Wave - Minor code refactoring.
04b - Continuous Frame - Start-to-start delay adjusted + minor code
	refactoring.
05a/05b - DS-TWR with STS - Changes to timing of frames and resync of STS
	made.
06a - SS-TWR Initiator - Minor code refactoring to move some functions to a
	shared location for other examples to use.
06e/f - SS-TWR with AES - Minor code refactoring to move some functions to a
	shared location for other examples to use.
07a - ACK Data TX - Minor code refactoring.

Added:
01h/02h - PDOA TX/RX - Phase Difference On Arrival (PDOA) example has been
	added.
06a/06b - SS-TWR with STS - A version of the SS-TWR simple example with STS
	has been added.
07c - ACK Data RX with Double Buffer - A version of the ACK Data RX code using
	double buffering feature.

c) Notes on major diffrences between previous release and this release,
	regarding APIs:
Changed:
- dwt_initialise: LDO tuning bias tuning has been adjusted.
- dwt_setdwstate: Force clocks to AUTO mode before adjusting sequence control.
- dwt_configure: DGC is now enabled by default.
- dwt_run_pgfcal: Minor code refactoring.
- dwt_readclockoffset: Changes made to cater for better use of double buffering.
- dwt_readstsstatus: Changes made to cater for better use of double buffering.
- dwt_readdiagnostics: Changes made to cater for better use of double buffering.
- dwt_readpdoa: Changes made to cater for better use of double buffering.
- dwt_readrxtimestamp: Changes made to cater for better use of double buffering.
- dwt_readrxtimestamp_ipatov: Changes made to cater for better use of double buffering.
- dwt_readrxtimestamp_sts: Changes made to cater for better use of double buffering.
- _dwt_otpprogword32: Changes made to help correctly program OTP bits.
- dwt_calibratesleepcnt: Minor code refactoring.
- dwt_setdblrxbuffmode: Changes made to cater for better use of double buffering.
- dwt_isr: Changes made to cater for better use of double buffering.
- dwt_readtempvbat: Changes made to enable MS2 LDO for reading SAR correctly.
- dwt_wait_aes_poll: Changed to be a static function.
- dwt_update_nonce_CCM: Changed to be a static function.
- dwt_update_nonce_GCM: Changed to be a static function.

- dwt_aes_src_port_e: Structure has been updated.
- dwt_aes_dst_port_e: Structure has been updated.
- dwt_configmrxlut_ch5_e: Structure has been updated.
- dwt_configmrxlut_ch9_e: Structure has been updated.

New:
- dwt_restoreconfig: Used to restore configuration upon waking from DEEPSLEEP/SLEEP.

Deleted:
- dwt_spicswakeup: Function has been removed.

=============================================================================
Package v1.0 / Driver v03.00.00  (8th January 2019)
=============================================================================

a) Initial release of DW3000 Driver and API for MP/C0 device, with some simple example
projects.

b) Following simple examples have are being released:
0a - read dev ID
1a/2a - simple TX/RX
1b - TX sleep
1c - TX sleep auto
1d - TX timed sleep
1e - TX with CCA
1g/2g - TX/RX with STS using SDC
1i/2i - TX/RX with AES
2c - RX with diagnostics
2d - RX with sniff mode enabled
2f - RX and crystal trim
3a/3b/3d - TX wait for response and responder
4a - continuous wave
4b - continuous frame
5a/5b - double sided (DS) TWR example
5c/5d - DS TWR with STS using SDC
6a/6b - single sided (SS) TWR example
6e/6f - SS TWR with AES
7a/7b - TX and ACK
11a - SPI CRC mode
13a - use of GPIOs 

c) Notes on major diffrences between DW1000 and DW3000 (MP/C0), regarding APIs:
Changed:
- dwt_initialise: this does not put DW3000 into IDLE mode
- dwt_configure: the configuration structure (dwt_config_t) has been changed, and new 
configuration options added: e.g. STS, PDOA. At the end of this function the device 
can be put into IDLE mode.
- dwt_readdiagnostics: the diagnostics structure (dwt_rxdiag_t) has been updated
- dwt_enableframefilter has been replaced by dwt_configureframefilter
- dwt_otpwriteandverify: updated - does not do retrys
- dwt_configuresleep: updated
- dwt_setdblrxbuffmode: updated
- dwt_setcallbacks: new SPI CRC error and SPI ready callback added
- dwt_checkidlerc: checks the device is in IDLE_RC state (ready for high speed SPI)
- dwt_isr: updated
- dwt_entersleep: updated
New:
- there are new modify SPI functions: dwt_modify8/16/32bitoffsetreg
- dwt_enablespicrccheck: used to enable SPI CRC mode
- dwt_signal_rx_buff_free: signal host has finished with the buffer
- dwt_configciadiag: configure level of CIA dignostic logging
- dwt_setreferencetrxtime: sets the reference time for delayed TX/RX
- dwt_calcbandwidthadj
- dwt_configure_aes
- dwt_mic_size_from_bytes
- dwt_set_keyreg_128
- dwt_do_aes
- dwt_configurestskey
- dwt_configurestsiv
- dwt_configurestsloadiv
- dwt_configmrxlut
- dwt_pgf_cal
Deleted:
- dwt_setgpiodirection
- dwt_setgpiovalue
- dwt_getgpiovalue
- dwt_configurefor64plen
- dwt_setsmarttxpower
- dwt_setlowpowerlistening
- dwt_lowpowerlistenisr
- dwt_setsnoozetime
- dwt_syncrxbufptrs
- dwt_rxreset
- dwt_calcbandwidthtempadj

d) the following default TX power/bandwidth configurations should be used:
Channel5: 
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0xFDFDFDFD      /* TX power. */
};
Channel9:
dwt_txconfig_t txconfig =
{
    0x34,           /* PG delay. */
    0xFEFEFEFE      /* TX power. */
};

=============================================================================
