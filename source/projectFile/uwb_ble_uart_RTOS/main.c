/*Nordic Include*/
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
/*Softdevice Include*/
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
/*RTOS Include*/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
/*BLE Include*/
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_nus_c.h"
#include "ble_conn_state.h"
#include "ble_db_discovery.h"
//#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "bsp_btn_ble.h"
/*BLE Peripheral*/
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
/*NRF Driver Include*/
#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"
#include "nrf_stack_guard.h"
/*Uart Include*/
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "app_uart.h"
#include "app_util_platform.h"
/*LOG Include*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
/*UWB Include*/
#include "deca_device_api.h"
#include <boards.h>
#include <portu.h>
#include <deca_spi.h>
#include <examples_defines.h>
#include <deca_regs.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <example_selection.h>


/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 8 below. */
extern dwt_txconfig_t txconfig_options;

#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "woosang_test"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define UWB_ROUND_INTERVAL         APP_TIMER_TICKS(30)   //1 : 60ms, 0.06               /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds).  */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UART_TX_BUF_SIZE                    256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                    256                                         /**< UART RX buffer size. */

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS      600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION       6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS     270                                     /**< Typical forward voltage drop of the diode . */
#define ADC_RES_10BIT                      1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
#define OSTIMER_WAIT_FOR_QUEUE             10                                      /**< Number of ticks to wait for the timer queue to be ready */

#define MAX_ANC_NUM                        3                                       /* Maximum number of anchors tag can be connected */
#define TAG_ID                             0x50                                    /* tag ID range : 0x50 ~ 0x99*/
#define ANCHOR_ID                          0xAA

/* Check ACK Message sent well  */
#define ACK_SUCCESS (1)
#define ACK_FAILED (0)


/*DS_TWR Parameters define*/
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define RNG_DELAY_MS 240
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_TX_TO_RESP_RX_DLY_UUS 700
#define RESP_RX_TO_FINAL_TX_DLY_UUS 700
#define RESP_RX_TIMEOUT_UUS 300
#define PRE_TIMEOUT 5

#define RANGING_DWTIME 7488018                 //30ms
#define START_TX_DWTIME 2496006                 //10ms


extern example_ptr example_pointer;
extern int unit_test_main(void);
extern void build_examples(void);
extern void TAG_ID_SEND(void);
extern int ACK_MSG_SEND(uint8_t session_id, uint8_t ANC_ID);
extern void test_run_info(unsigned char *data);
extern int ds_twr_init(void);
extern int rcm_rx(void);
extern int ds_twr_initiator(void);
extern void init_ds(void);

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
        ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

static nrf_saadc_value_t adc_buf[2];

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                  /**< BLE NUS service instance. Maximum number of total concurrent connections : 3 */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
//BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

//struct ranging_alloc{
//    uint8_t anc_ID;
//    uint8_t rng_round;
//};

static uint8_t anchor_ID;              /* Buffer for storing anchor number */
static uint8_t round_ID;                          /* Ranging round ID that can be used for tag' sleeping*/
static uint8_t sess_check;                       /* Session flag for diagnoising UWB Ranging */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) : 20byte
                                                                           that can be transmitted to the peer by the Nordic UART service module. */

static char const m_target_periph_name[] = "woosang_test";     /**< Name of the device we try to connect to. This name is searched in the scan report data*/

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

/* DS_TWR_Variables */

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

static uint8_t RCM_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'C', 'M', ANCHOR_ID, 0, 0};
//static uint8_t RCM_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'R', 'C', 'M'};
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;
/* Time-stamps of frames transmission/reception, expressed in device time units. */
static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;
static uint32_t ranging_time;
// DS-TWR Variable End

static TimerHandle_t m_battery_timer;                               /**< Definition of battery timer. */

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif
static TaskHandle_t uwb_thread;                                /**< Definition of Logger thread. */

static TickType_t xLastWakeTime;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

#define FPU_EXCEPTION_MASK 0x0000009F
void FPU_IRQHandler(void)
{
        uint32_t *fpscr = (uint32_t *)(FPU->FPCAR+0x40);
        (void)__get_FPSCR();

        *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}


void test_run_info(unsigned char *data)
{
    printf("%s\n", data);
}


void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    bsp_board_led_on(ADVERTISING_LED);
}

/**@brief Function for handling characters received by the Tag.
 */
static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
{
        ret_code_t ret_val;

        printf("Receiving data.");
        NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

        for (uint32_t i = 0; i < data_len; i++)
        {
                do
                {
                        ret_val = app_uart_put(p_data[i]);
                        if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
                        {
                                printf("<Error> app_uart_put failed for index 0x%04x.\n", i);
                                APP_ERROR_CHECK(ret_val);
                        }
                } while (ret_val == NRF_ERROR_BUSY);
        }
        if (p_data[data_len-1] == '\r')
        {
                while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
        static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
        static uint16_t index = 0;
        uint32_t ret_val;

        switch (p_event->evt_type)
        {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
                UNUSED_VARIABLE(app_uart_get(&data_array[index]));
                index++;

                if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
                {
                        printf("<info> Ready to send data over BLE NUS\n");
                        NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                        do
                        {
                                for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
                                {
                                        ret_val = ble_nus_c_string_send(&m_ble_nus_c[i], data_array, index);
//                                        if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES) )
//                                        {
//                                                APP_ERROR_CHECK(ret_val);
//                                        }
                                }
                        } while (ret_val == NRF_ERROR_RESOURCES);

                        index = 0;
                }
                break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
                printf("<Error> Communication error occurred while handling UART.\n");
                APP_ERROR_HANDLER(p_event->data.error_communication);
                break;

        case APP_UART_FIFO_ERROR:
                 printf("<Error> Error occurred in FIFO module used by UART.\n");
                APP_ERROR_HANDLER(p_event->data.error_code);
                break;

        default:
                break;
        }
}


///**@snippet [Handling events from the ble_nus_c module] */
//static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_c_evt)
//{
//        ret_code_t err_code;

//        switch (p_ble_nus_c_evt->evt_type)
//        {
//        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
//               printf("<info> NUS Service discovered on conn_handle 0x%x\n",
//                             p_ble_nus_c_evt->conn_handle);

//                err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_c_evt->conn_handle, &p_ble_nus_c_evt->handles);
//                APP_ERROR_CHECK(err_code);

//                printf("<info> Before enable the tx notification\n");
//                NRF_LOG_HEXDUMP_DEBUG(p_ble_nus_c, sizeof(ble_nus_c_t));
//                err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
//                APP_ERROR_CHECK(err_code);

//                 printf("<info> Connected to device with Nordic UART Service.\n\n");
//                break;

//        case BLE_NUS_C_EVT_NUS_TX_EVT:
//                ble_nus_chars_received_uart_print(p_ble_nus_c_evt->p_data, p_ble_nus_c_evt->data_len);
//                break;

//        case BLE_NUS_C_EVT_DISCONNECTED:
//                 printf("<info> Conn_handle %d is disconnected\n", p_ble_nus_c_evt->conn_handle);
//                 NRF_LOG_INFO("<info> Disconnected.\n");
//                 sess_check = 0;
//                 advertising_start();
//                break;
//        }
//}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH; // ATT payload
                 printf("<info> Data len is set to 0x%X(%d)\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len); // 20byte
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                 printf("<info> gatt_event: Data len is set to 0x%X (%d)\n", data_length, data_length);
                m_ble_nus_max_data_len = data_length;
        }
         printf("<info> ATT MTU exchange completed. central 0x%x peripheral 0x%x\n\n",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
        ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
        APP_ERROR_CHECK(err_code);

        //TAG_ID_SEND();
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
        uint32_t err_code;
        app_uart_comm_params_t const comm_params =
        {
                .rx_pin_no    = RX_PIN_NUMBER,
                .tx_pin_no    = TX_PIN_NUMBER,
                .rts_pin_no   = RTS_PIN_NUMBER,
                .cts_pin_no   = CTS_PIN_NUMBER,
                .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
                .use_parity   = false,
#if defined (UART_PRESENT)
                .baud_rate    = NRF_UART_BAUDRATE_115200
#else
                .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
        };

        APP_UART_FIFO_INIT(&comm_params,
                           UART_RX_BUF_SIZE,
                           UART_TX_BUF_SIZE,
                           uart_event_handle,
                           APP_IRQ_PRIORITY_LOWEST,
                           err_code);
        APP_ERROR_CHECK(err_code);
}

/**@brief  Function for sending TAG's indentifier through the UART module. */
void TAG_ID_SEND(void)
{
      ret_code_t send_check;
   
      uint8_t id_array[2] = {TAG_ID, 0x01};      
      do
        {
            uint16_t length = 2;

            send_check = ble_nus_data_send(&m_nus, id_array, &length, m_conn_handle);
            if ((send_check != NRF_ERROR_INVALID_STATE) &&
                (send_check != NRF_ERROR_RESOURCES) &&
                (send_check != NRF_ERROR_NOT_FOUND))
            {
                APP_ERROR_CHECK(send_check);
            }
        } while (send_check == NRF_ERROR_RESOURCES);

      APP_ERROR_CHECK(send_check);
      printf("TAG ID sent well!!\n\n");
}

/**@brief  Function for sending ACK message through the UART module. */
int ACK_MSG_SEND(uint8_t session_id, uint8_t anchor)
{
        ret_code_t send_check;

        uint8_t ack_array[4] = {TAG_ID, anchor, session_id, 0xAC}; 

        do
          {
              uint16_t length = 4;
              send_check = ble_nus_data_send(&m_nus, ack_array, &length, m_conn_handle);
              if ((send_check != NRF_ERROR_INVALID_STATE) &&
                  (send_check != NRF_ERROR_RESOURCES) &&
                  (send_check != NRF_ERROR_NOT_FOUND))
              {
                  APP_ERROR_CHECK(send_check);
              }
              //else
              //{
              //    printf("<Error> Sending ACK failed. \n\n");
              //    return ACK_FAILED;
              //}
          } while (send_check == NRF_ERROR_RESOURCES);

        APP_ERROR_CHECK(send_check);
        printf("<info> Sent ACK message Well! \n\n");

        return ACK_SUCCESS;
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint16_t r_data[p_evt->params.rx_data.length];
        uint8_t ack;

        printf("Received data from BLE NUS. Writing data on UART.\n\n\n");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
           r_data[i]=p_evt->params.rx_data.p_data[i];
        }

        uint8_t MSG_TYPE = r_data[2];

        switch(MSG_TYPE)
        {
            case 0x01: 
              if(r_data[0] == TAG_ID)
              { 
                 printf("@@Check Session can be established...@@\n\n");
                 anchor_ID = r_data[1];
                 ack = ACK_MSG_SEND(1, anchor_ID);
                 //if (ack == ACK_SUCCESS)
                 //{
                   round_ID = r_data[3];
                   sess_check = 1;
                   printf("Session Established. Anchor ID : %x, Round : %d\n\n", anchor_ID, round_ID);
                 //}
              }
              else
              {
                 printf("<0x01>TAG Num Invalid!\n\n");
              }
               break;
        
            case 0x02:
              if(r_data[0] == TAG_ID)
              {
                 if((r_data[3] != round_ID) && (r_data[3] != 0) )
                 {
                    printf("Session Round ID Updated!!\n\n");
                    sess_check = 0;
                    anchor_ID = r_data[1];
                    ack = ACK_MSG_SEND(2, anchor_ID);
                    if (ack == ACK_SUCCESS)
                    {
                      round_ID = r_data[3];
                      sess_check = 1;
                      printf("Session Updated. Anchor ID : %x, Round : %d\n\n", anchor_ID, round_ID);
                    }
                 }
                 else if(r_data[3] == 0){
                    printf("<Out of Range> Session will be discarded!!\n\n");
                    sd_ble_gap_disconnect(m_conn_handle,BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                    bsp_board_led_off(CONNECTED_LED);
                    m_conn_handle = BLE_CONN_HANDLE_INVALID;
                    sess_check = 0;
                    round_ID = 0;
                    advertising_start();
                 }
                 else 
                 {
                    printf("<Unknown Error>\n\n");
                 }
              }
              else
              {
                printf("<0x02>TAG Num Invalid!!\n\n");
              }
               break;

            case 0x03:
              if(r_data[0] == TAG_ID)
              {
                 printf("@@@ Warning Alarm : %d\n\n", r_data[3]);
              }
              else
              {
                 printf("<0x03>TAG Num Invalid!!\n\n");
              }
               break;   
        }
    }
    else if(p_evt->type == BLE_NUS_EVT_COMM_STARTED)
    {
        TAG_ID_SEND();
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;


    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) //!< Negotiation procedure failed.
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE); /* m_conn_handle = BLE_CONN_HANDLE_INVALID, Connection Interval Unacceptable. */
        sess_check = 0;
        round_ID = 0;
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY; // 
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
   ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            printf("Connected\n\n");
            bsp_board_led_on(CONNECTED_LED);
            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            printf("Disconnected\n\n");
            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            sess_check = 0;
            round_ID = 0;
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            sess_check = 0;
            round_ID = 0;
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            sess_check = 0;
            round_ID = 0;
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}



///**@brief Function for initializing the Nordic UART Service (NUS) client. */
//static void nus_c_init(void)
//{
//        ret_code_t err_code;
//        ble_nus_c_init_t init;

//        init.evt_handler = ble_nus_c_evt_handler;

//        for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
//        {
//                err_code = ble_nus_c_init(&m_ble_nus_c[i], &init);
//                APP_ERROR_CHECK(err_code);
//        }
//}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
     ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}



/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
        ret_code_t err_code;

        switch (event)
        {

        case BSP_EVENT_DISCONNECT:
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                sess_check = 0;
                round_ID = 0;
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                        APP_ERROR_CHECK(err_code);
                }
                break;

        case BSP_EVENT_WHITELIST_OFF:
//                if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
//                {
//                        err_code = ble_advertising_restart_without_whitelist(&m_advertising);
//                        if (err_code != NRF_ERROR_INVALID_STATE)
//                        {
//                                APP_ERROR_CHECK(err_code);
//                        }
//                }
                break;

        default:
                break;
        }
}



/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
        ret_code_t err_code = NRF_LOG_INIT(NULL);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;


    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS; // should be 2Mbps?
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED; // advertising types ** need closer looking at **
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


#if NRF_LOG_ENABLED
static void logger_thread(void * arg)
{
        UNUSED_PARAMETER(arg);

        while (1)
        {
                NRF_LOG_FLUSH();

                vTaskSuspend(NULL); // Suspend myself
        }
}
#endif 

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
        vTaskResume(m_logger_thread);
#endif
}


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
        ret_code_t err_code = nrf_drv_clock_init();
        APP_ERROR_CHECK(err_code);
}

//static void HIHIHI(void)
//{
//    xTaskNotify(uwb_thread, (1<<0), eSetBits);
//}

///**@brief Function for the Timer initialization.
// *
// * @details Initializes the timer module. This creates and starts application timers.
// */
//static void timers_init(void)
//{
//        // Initialize timer module.
//        ret_code_t err_code = app_timer_init();
//        APP_ERROR_CHECK(err_code);

//        // Create timers.
//        m_battery_timer = xTimerCreate("UWB",
//                                       UWB_ROUND_INTERVAL,
//                                       pdTRUE,
//                                       NULL,
//                                       HIHIHI);
//        /* Error checking */
//        if ( (NULL == m_battery_timer))
//        {
//                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//        }
//}

///**@brief   Function for starting application timers.
// * @details Timers are run after the scheduler has started.
// */
//static void uwb_timers_start(void)
//{
//        // Start application timers.
//        if (pdPASS != xTimerStart(m_battery_timer, 0))
//        {
//                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//        }
//}


/**@brief Function for application main entry.
 */
int main(void)
{
        bool erase_bonds;
        uart_init();
        log_init();
        clock_init();
        nrf_drv_clock_lfclk_request(NULL);
        APP_ERROR_CHECK(nrf_stack_guard_init());
      
        printf("UWB Tag Start.\n");
          
        if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 128, NULL, 2, &m_logger_thread))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }

        ble_stack_init();    // Configure and initialize the BLE stack.

        /* Initialize modules. */
        buttons_leds_init(&erase_bonds);
        gap_params_init();
        gatt_init();
        services_init();
        advertising_init();
        conn_params_init();
        
        /* UWB initialize */
        nrf52840_dk_spi_init();
        dw_irq_init();
        nrf_drv_gpiote_in_event_disable(DW3000_IRQn_Pin);
        nrf_delay_ms(2);
 
        /* UWB Start */
      
        if (pdPASS != xTaskCreate(ds_twr_init, "UWB", 2*1024, NULL, 1, &uwb_thread))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
  

        /* Create a FreeRTOS task for the BLE stack. */
        nrf_sdh_freertos_init(advertising_start, &erase_bonds);

        printf("UWB & BLE RTOS Start.\n");
        vTaskStartScheduler();
        for (;;)
        {
                APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        }
}


/**@brief A function which starts UWB DS TWR.
 * @note UWB Session shoud be started when the RCM message is received.
 */
int ds_twr_init(void)
{
    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2); 

    while (!dwt_checkidlerc()) 
    { };

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        while (1)
        { };
    }

    if(dwt_configure(&config)) /* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
    {
        while (1)
        { };
    }
    
    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    while (1)
    {
     //printf("Session Check: %d\n\n",sess_check);
     if(sess_check == 1)
     {
        //dwt_setrxaftertxdelay(0);
        //dwt_setrxtimeout(0);
        //dwt_setpreambledetecttimeout(0);
        
        uint32_t rcm_rx_time;

        rcm_rx_time = rcm_rx();

        //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        //dwt_setpreambledetecttimeout(PRE_TIMEOUT);
      
        printf("This is DS_TWR_INIT, Round ID : %d\n\n",round_ID);
        dwt_setreferencetrxtime(rcm_rx_time);

        ranging_time = (RANGING_DWTIME * round_ID) + START_TX_DWTIME - TX_ANT_DLY;

        dwt_setdelayedtrxtime(ranging_time); // Delay until ranging round time from RCM message.

        /* Start transmission, Block delay : 240ms */
        tx_poll_msg[ALL_MSG_SN_IDX] = 0;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg)+FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */
        
        //nrf_gpio_pin_toggle(LED_4);
        dwt_starttx(DWT_START_TX_DLY_REF | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        {};

        frame_seq_nb++;

        printf("DS TWR initiator start TX Check \n\n");	

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t frame_len;
            printf("DS TWR initiator start TX Complete\n\n");

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            printf("HIHIHI\n\n");
            printf("frame_len: %d \n\n", frame_len);
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                printf("Receive Resp Message Success\n\n");
                uint32_t final_tx_time;
                int ret;

                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                tx_final_msg[ALL_MSG_SN_IDX] = 0;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_final_msg)+FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */

                ret = dwt_starttx(DWT_START_TX_DELAYED);

                if (ret == DWT_SUCCESS)
                {
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                    { };

                    printf("DS_TWR Successed!\n\n\n");

                    /* Clear TXFRS event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                    frame_seq_nb++;

                }
            }
         }
        else
        {
            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(230));
        //Sleep(0);
     }
   }
}


/**@brief A function which receives RCM message from UWB Anchor.
 * @note RX event is occured as soon as BLE sent ACK message.
 */
int rcm_rx(void)
{
      printf("@@@@@@@@@@@rcm RX start@@@@@@@@@@@\n\n");

      uint32_t rcm_rx_ts;

      memset(rx_buffer,0,sizeof(rx_buffer));

      dwt_rxenable(DWT_START_RX_IMMEDIATE);

      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
      {
        //printf("rcm_rx failed, round ID : %d Session: %d\n\n",round_ID,sess_check);
      };

      xLastWakeTime = xTaskGetTickCount();
      
        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {

          uint16_t frame_len;


          /* A frame has been received, read it into the local buffer. */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
          if (frame_len <= sizeof(rx_buffer))
          {
              dwt_readrxdata(rx_buffer, frame_len, 0);

              rx_buffer[ALL_MSG_SN_IDX] = 0;
              if (memcmp(rx_buffer, RCM_msg, 8) == 0)
              {                  
                  rcm_rx_ts = dwt_readrxtimestamphi32(); /* Retrieve RCM reception timestamp.to fuction parameter */
              }

              /* Clear good RX frame event in the DW IC status register. */
              dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
          }
        }
        else
        {
            /* Clear RX error events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        return rcm_rx_ts;
}
