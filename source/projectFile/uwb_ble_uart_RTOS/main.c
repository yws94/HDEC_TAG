/*Nordic Include*/
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_delay.h"
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
/*PWM Include*/
#include "app_pwm.h"


/*IT1_BOARD_TAG*/
// Enable + PWR
#define   bUWB_PWR_EN   NRF_GPIO_PIN_MAP(0,22)
#define   bPWR_OFF      NRF_GPIO_PIN_MAP(0,20)
// LED + BTN
#define   bSTATE_LED    NRF_GPIO_PIN_MAP(0,15)
#define   bCH_CP_LED    NRF_GPIO_PIN_MAP(0,13)
#define   bBTN_SIG      NRF_GPIO_PIN_MAP(0,12)
#define   bBTN_SOS      NRF_GPIO_PIN_MAP(1,0)
// ADC
#define   bBATT_SENS    NRF_GPIO_PIN_MAP(0,31)
#define   bCH_COMP      NRF_GPIO_PIN_MAP(1,9)
// PWM
#define   bVibrator     NRF_GPIO_PIN_MAP(0,6)
#define   bBUZZER       NRF_GPIO_PIN_MAP(0,24)
//WATCHDOG
#define   bWDOn         NRF_GPIO_PIN_MAP(0,26)
#define   bWDI          NRF_GPIO_PIN_MAP(0,8)

/*SAADC define*/
#define SAMPLES_IN_BUFFER 5
//volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(2);
static nrf_saadc_value_t     m_buffer_pool[1][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

/*PWM define*/
APP_PWM_INSTANCE(PWM1,1);                 // Create the instance "PWM1" using TIMER1

//#define ADVERTISING_LED                 BSP_BOARD_LED_0                         /**< Is on when device is advertising. */
//#define CONNECTED_LED                   BSP_BOARD_LED_1                         /**< Is on when device has connected. */
//#define LEDBUTTON_LED                   BSP_BOARD_LED_2                         /**< LED to be toggled with the help of the LED Button Service. */
//#define LEDBUTTON_BUTTON                BSP_BUTTON_0                            /**< Button that will trigger the notification event with the LED Button Service */

//#define PERIPHERAL_LINK                 NRF_SDH_BLE_PERIPHERAL_LINK_COUNT     /**< Number of peripheral links which can connect to BLE Central. */
#define BOARDCHECK_INTERVAL         APP_TIMER_TICKS(500)//APP_TIMER_TICKS(30)   //1 : 60ms, 0.06               /**< Battery level measurement interval (ticks). This value corresponds to 120 seconds. */
                                        
#define DEVICE_NAME                     "testtest"//"woosang_test"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds).  */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(10000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

//#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
//#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
//#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
//#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
//#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
//#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
//#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
//#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

/* Queue & Semaphore parameters define */
#define QUEUE_LENGTH                        2
#define QUEUE_ITEM_SIZE                     sizeof(Session_Data_t)

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define UART_TX_BUF_SIZE                    256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                    256                                         /**< UART RX buffer size. */

//#define ADC_REF_VOLTAGE_IN_MILLIVOLTS      600                                     /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
//#define ADC_PRE_SCALING_COMPENSATION       6                                       /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
//#define DIODE_FWD_VOLT_DROP_MILLIVOLTS     270                                     /**< Typical forward voltage drop of the diode . */
//#define ADC_RES_10BIT                      1024                                    /**< Maximum digital value for 10-bit ADC conversion. */
#define OSTIMER_WAIT_FOR_QUEUE             10                                      /**< Number of ticks to wait for the timer queue to be ready */

#define MAX_ANC_NUM                        3                                       /* Maximum number of anchors tag can be connected */
#define TAG_ID                             0x70                                   /* tag ID range : 0x50 ~ 0x99 */
#define ANCHOR_ID                          0xAA                                   /* It'll be deleted when RCM message format negotiation is set */

/*UWB Ranging Parameters define*/
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_TX_TO_RESP_RX_DLY_UUS 700
#define RESP_RX_TO_FINAL_TX_DLY_UUS 700
#define RESP_RX_TIMEOUT_UUS 300
#define PRE_TIMEOUT 5

#define RANGING_DWTIME   249600*24              // 24ms **< Round delay offset time : 15ms = 3744009*/
#define START_TX_DWTIME  249600*5               //1m*5


/* Queue & Semaphore */
static QueueHandle_t xQueue; // Queue for BLE-UWB task interface

typedef struct
{
    uint8_t anchor_ID;
    uint8_t round_ID;
}Session_Data_t;

/* (extern)Funtions Declaration */
extern dwt_txconfig_t txconfig_options;
extern example_ptr example_pointer;
extern int unit_test_main(void);
extern void build_examples(void);
int ds_twr_init(void);
int rcm_rx(void);
void ACK_SESS_SEND(Session_Data_t sess_t);
void ACK_UP_SEND(Session_Data_t up_t);


/* BLE_Variables */
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

/* Board_Variables */
APP_TIMER_DEF(m_board_pwr_tmr);
APP_TIMER_DEF(m_pwr_timer_id);
static bool         PWR_OFF_enabled;
static bool         BTN_SOS_enabled;

BLE_NUS_C_ARRAY_DEF(m_ble_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
//BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

///**@brief Macro to convert the result of ADC conversion in millivolts.
// *
// * @param[in]  ADC_VALUE   ADC result.
// *
// * @retval     Result converted to millivolts.
// */
//#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE) \
//        ((((ADC_VALUE) *ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

//static nrf_saadc_value_t adc_buf[2];

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3; /**< Maximum length of data (in bytes) : 20byte
                                                                           that can be transmitted to the peer by the Nordic UART service module. */
static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static TimerHandle_t board_timer;                               /**< Definition of battery timer. */

static bool         PWM_enabled;

//static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
//{
//    {BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE},
//    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
//    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
//};

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
//BLE_Variables End


/* DS_TWR_Variables */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};
//static uint8_t RCM_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'C', 'M', ANCHOR_ID, 0, 0}; 
static uint8_t RCM_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'S', 'Y', 'N', 'C', 0xE1, ANCHOR_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21};
static uint8_t rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;

#define RX_BUF_LEN 30
static uint8_t rx_buffer[RX_BUF_LEN];

static uint32_t status_reg = 0;

static uint64_t poll_tx_ts;
static uint64_t resp_rx_ts;
static uint64_t final_tx_ts;
static uint32_t ranging_time; 
static uint8_t sess_check;
static uint8_t cnt;
static uint8_t Alert_check;
//static uint8_t cnt;
// DS-TWR Variable End

/* Defining Handler(thread) Name */
static TimerHandle_t m_battery_timer;                               /**< Definition of battery timer. */

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif
static TaskHandle_t uwb_thread;                                /**< Definition of Logger thread. */

static TickType_t xLastWakeTime;


#define FPU_EXCEPTION_MASK 0x0000009F
void FPU_IRQHandler(void)
{
        uint32_t *fpscr = (uint32_t *)(FPU->FPCAR+0x40);
        (void)__get_FPSCR();

        *fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
        app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


static void pwm_init(void)
{
    ret_code_t  err_code;

    /* 2-channel PWM, 200Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(245,24,6);
    
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;


    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,NULL);
    APP_ERROR_CHECK(err_code);   
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    //bsp_board_led_on(ADVERTISING_LED);
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

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params); // GAP Peripheral Preferred Connection Parameters
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the GATT library. */
/* @ params [in] p_gatt : GATT module event handler type, p_evt : GATT module event */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
        uint32_t data_length;
        if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
        {
                m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
                 printf("<info> Data len is set to 0x%X(%d)\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
        }
        else if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
        {
                data_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH - 4;
                 printf("<info> gatt_event: Data len is set to 0x%X (%d)\n", data_length, data_length);
                m_ble_nus_max_data_len = data_length;
        }
         printf("<info> ATT MTU exchange completed. central 0x%x peripheral 0x%x\n",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
        ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE); // 23 byte
        APP_ERROR_CHECK(err_code);

        err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH); //27 byte
        APP_ERROR_CHECK(err_code);
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




/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 *
 *    Case  1. Received Session Establishment Message : Send Ack_Session Message -> Obtaining Anchor ID & Round ID -> Set UWB session flag(Ranging Start)
 *          2. 1) Received Session Update Message : Reset all parameters related to UWB Session. -> Send Ack_Session_update Message -> Set all updated UWB Session parameters
 *             2) Received Session Discard Message : Disconnect BLE connection event -> Reset all parameters related to UWB Session
  *         3. Received BLE Alarming Message : Print Warning Message. 
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    Session_Data_t xSendSession;
    BaseType_t xStatus;
    const TickType_t xTicksToWait = pdMS_TO_TICKS(1); // minimum tick time

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint8_t r_data[p_evt->params.rx_data.length];
        ret_code_t  err_code;

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
           r_data[i]=p_evt->params.rx_data.p_data[i];
          
        }
        
        switch(r_data[0]){
        
        case 0x01: 
           printf("Session Established\n\n");
           xSendSession.anchor_ID = r_data[1];
           xSendSession.round_ID = r_data[2];
           sess_check = 1;
           xStatus = xQueueSendToBack(xQueue, &xSendSession, xTicksToWait);
           ACK_SESS_SEND(xSendSession);
           if(xStatus == pdPASS)
           {
              printf("Anchor ID : %x, Round : %d\n\n", xSendSession.anchor_ID, xSendSession.round_ID);
           }
           else
           {  
              printf("Could not send to the queue.\r\n");
           }

           
           break;
        
        case 0x02:
           if(r_data[1] == TAG_ID)
           {
              if ((r_data[3] != xSendSession.round_ID) && (r_data[3] != 0))
              {
                printf("Session Updated!!\n\n");
                xSendSession.round_ID = 0;
                sess_check = 0;
                xSendSession.anchor_ID = r_data[2];
                xSendSession.round_ID = r_data[3];
                ACK_UP_SEND(xSendSession);
                
                sess_check = 1;
                printf("Session Updated well. Anchor ID : %x, Round : %d\n\n", xSendSession.anchor_ID, xSendSession.round_ID);
              }

              else if(r_data[3] == 0)
              {
                printf("Session will be discarded!!\n\n");
                sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                m_conn_handle = BLE_CONN_HANDLE_INVALID;
                xSendSession.round_ID = 0;
                sess_check = 0;
              } 
           }
           else
           {
              printf("<0x02> TAG NUM Invalid!!\n\n");
           }
            
           break; 

        case 0x03:
           if(r_data[2] == TAG_ID)
           {              
              xSendSession.anchor_ID = r_data[1];
              PWM_enabled = true; 
              printf("@@@@ Warning! Intensity : %d @@@@\n\n", r_data[3]);
              if (r_data[3] == 0)
              {
                  Alert_check = 0;
                  //app_pwm_enable(&PWM1);
                  ////err_code = app_pwm_channel_duty_set(&PWM1,1,50);
                  ////APP_ERROR_CHECK(err_code);
                  //err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
                  //APP_ERROR_CHECK(err_code);     
                  //nrf_delay_ms(120);     
                  //app_pwm_disable(&PWM1);

                  //nrf_delay_ms(50);           
                  //app_pwm_enable(&PWM1);
                  //err_code = app_pwm_channel_duty_set(&PWM1,1,50);
                  //APP_ERROR_CHECK(err_code);
                  //err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
                  //APP_ERROR_CHECK(err_code); 
                  //nrf_delay_ms(50);     
                  //app_pwm_disable(&PWM1);
                  //nrf_delay_ms(50);                     
              }              
              else if (r_data[3] == 1)
              {
                  Alert_check = 1;
                  //nrf_delay_ms(50);           
                  //app_pwm_enable(&PWM1);
                  //err_code = app_pwm_channel_duty_set(&PWM1,1,50);
                  //APP_ERROR_CHECK(err_code);
                  //err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
                  //APP_ERROR_CHECK(err_code); 
                  //nrf_delay_ms(50);     
                  //app_pwm_disable(&PWM1);
                  //nrf_delay_ms(50);
              }
              else if (r_data[3] == 2)
              {
                  //nrf_delay_ms(50);           
                  //app_pwm_enable(&PWM1);
                  //err_code = app_pwm_channel_duty_set(&PWM1,1,50);
                  //APP_ERROR_CHECK(err_code);
                  //err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
                  //APP_ERROR_CHECK(err_code); 
                  //nrf_delay_ms(50);     
                  //app_pwm_disable(&PWM1);
                  //nrf_delay_ms(50);                             
              }              
           }
           else
           {
              printf("<0x03> TAG NUM Invalid!!\n\n");
              PWM_enabled = false;
           }
           break;   
        }//switch end
    }//if (p_evt->type == BLE_NUS_EVT_RX_DATA)
}

/**@brief  Function for sending ACK message through the UART module.
 */
/* Sending ACK message when received Round message in Session establishment process. */
void ACK_SESS_SEND(Session_Data_t sess_t)
{
        ret_code_t send_check;

        uint8_t data_array[4] = {0x01, TAG_ID, 0xAC, sess_t.round_ID};

        printf("<Session> Send ACK message. \n\n");                
        do
          {
              uint16_t length = 4;
              send_check = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
              if ((send_check != NRF_ERROR_INVALID_STATE) &&
                  (send_check != NRF_ERROR_RESOURCES) &&
                  (send_check != NRF_ERROR_NOT_FOUND))
              {
                  APP_ERROR_CHECK(send_check);
              }
          } while (send_check == NRF_ERROR_RESOURCES);

        APP_ERROR_CHECK(send_check);
        printf("<Session> Finished sending ACK message. \n\n");
}

/**@brief  Function for sending ACK message through the UART module. 
*/
/* Sending ACK message when received Session Update message. */
void ACK_UP_SEND(Session_Data_t up_t)
{
        ret_code_t send_check;

        uint8_t data_array[5] = {0x02, up_t.anchor_ID, TAG_ID, 0xAC, up_t.round_ID};

        printf("<Update> Send ACK message. \n\n");                
        do
          {
              uint16_t length = 5;
              send_check = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
              if ((send_check != NRF_ERROR_INVALID_STATE) &&
                  (send_check != NRF_ERROR_RESOURCES) &&
                  (send_check != NRF_ERROR_NOT_FOUND))
              {
                  APP_ERROR_CHECK(send_check);
              }
          } while (send_check == NRF_ERROR_RESOURCES);

        APP_ERROR_CHECK(send_check);
        printf("<Update> Finished sending ACK message. \n\n");
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

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) // Connection Parameters Module event type -> Negotiation procedure failed
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE); // Connection Interval Unacceptable
        sess_check = 0;
        //round_ID = 0;
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
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
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



/**@brief Function for handling the Connected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
//static void on_connected(const ble_gap_evt_t * const p_gap_evt)
//{
//    ret_code_t  err_code;
//    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

//    printf("Connection with link 0x%x established.\n\n", p_gap_evt->conn_handle);

//    // Assign connection handle to available instance of QWR module.
//    for (uint32_t i = 0; i < PERIPHERAL_LINK; i++)
//    {
//        if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID)
//        {
//            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], p_gap_evt->conn_handle);
//            APP_ERROR_CHECK(err_code);
//            break;
//        }
//    }

//    err_code = app_button_enable();
//    APP_ERROR_CHECK(err_code);

//    // Update LEDs
//    bsp_board_led_on(CONNECTED_LED);
//    if (periph_link_cnt == PERIPHERAL_LINK)
//    {
//        bsp_board_led_off(ADVERTISING_LED);
//    }
//    else
//    {
//        // Continue advertising. More connections can be established because the maximum link count has not been reached.
//        advertising_start();
//    }
//}


/**@brief Function for handling the Disconnected event.
 *
 * @param[in] p_gap_evt GAP event received from the BLE stack.
 */
//static void on_disconnected(ble_gap_evt_t const * const p_gap_evt)
//{
//    ret_code_t  err_code;
//    uint32_t    periph_link_cnt = ble_conn_state_peripheral_conn_count(); // Number of peripheral links.

//    printf("Connection 0x%x has been disconnected. Reason: 0x%X\n\n",
//                 p_gap_evt->conn_handle,
//                 p_gap_evt->params.disconnected.reason);

//    if (periph_link_cnt == 0)
//    {
//        bsp_board_led_off(CONNECTED_LED);
//        err_code = app_button_disable();
//        APP_ERROR_CHECK(err_code);
//    }

//    if (periph_link_cnt == (PERIPHERAL_LINK - 1))
//    {
//        // Advertising is not running when all connections are taken, and must therefore be started.
//        advertising_start();
//    }
//}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context) // ble_evt_t : Common BLE Event type
{
   ret_code_t err_code;

    switch (p_ble_evt->header.evt_id) // BLE Event header -> Value from a BLE_<module>_EVT series.
    {
        case BLE_GAP_EVT_CONNECTED:
            printf("Connected\n\n");
            //bsp_board_led_on(CONNECTED_LED);
            //bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //on_connected(&p_ble_evt->evt.gap_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            printf("Disconnected\n\n\n");
            //bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            sess_check = 0;
            Alert_check = 0;
            cnt = 0;
            //round_ID = 0;
            //on_disconnected(&p_ble_evt->evt.gap_evt);
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
            //round_ID = 0;
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            sess_check = 0;
            //round_ID = 0;
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

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
                //round_ID = 0;
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
    ble_advdata_t advdata;   //Advertising data
    ble_advdata_t srdata;    //Scan Response data


    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;                    /**< Include full device name in advertising data. */
    advdata.include_appearance = true;                                     /**< Determines if Appearance shall be included. */
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    // Build and set scan response data.
    memset(&srdata, 0, sizeof(srdata));

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters. -> Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED; /**< Connectable and scannable undirected
                                                                                        advertising events. */
    adv_params.p_peer_addr     = NULL;                                    /**< Address of a known peer. */
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;                     /**< Allow scan requests and connect requests from any device. */
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


/**@brief A function which is used for SAADC
 */
void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    printf("hi\n");
}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        int16_t    BATT;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
        
        BATT = p_event->data.done.p_buffer[0];
        printf("%d\n", BATT);
        
        // 1.6V~2.1V (0~100%)
        if (BATT > 568 )
        {
            int ch_comp = nrf_gpio_pin_read(bCH_COMP);
            if ( ch_comp == 1 )
            {
                nrf_gpio_pin_clear(bBATT_SENS);
            }
        }
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    //err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    //APP_ERROR_CHECK(err_code);

}

static void timer_event(void)
{
      uint8_t             BTN_SOS, BTN_PWR, PWR_CNT;
      uint32_t            err_code;
      //static bool         PWR_OFF_enabled = false;
      //static bool         BTN_SOS_enabled = false;
   
      BTN_SOS = nrf_gpio_pin_read(bBTN_SOS);   // SOS button
      BTN_PWR = nrf_gpio_pin_read(bBTN_SIG);  // Power button

      if(cnt == 2)
      {
          if(BTN_SOS == 1)
          {
              app_pwm_disable(&PWM1);
          }
          else if (BTN_SOS == 0)    //pressed
          {
              app_pwm_enable(&PWM1);
              //err_code = app_pwm_channel_duty_set(&PWM1,1,50);
              //APP_ERROR_CHECK(err_code);
              err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
              APP_ERROR_CHECK(err_code);     
              nrf_delay_ms(500);     
              app_pwm_disable(&PWM1);               
          }
      
          if (BTN_PWR == 0)
          {          
              if (PWR_OFF_enabled == false)
              {
                  nrf_gpio_pin_set(bPWR_OFF);   // bPWR_OFF High
                  nrf_gpio_pin_clear(bSTATE_LED); // bSTATE_LED ON

                  err_code = app_timer_start(m_pwr_timer_id, APP_TIMER_TICKS(200), NULL);
                  APP_ERROR_CHECK(err_code); 
              }
              else if (PWR_OFF_enabled == true)
              {
                  nrf_gpio_pin_toggle(bSTATE_LED);  // bSTATE_LED 1sec toggle 
                  err_code = app_timer_start(m_pwr_timer_id, APP_TIMER_TICKS(3000), NULL);
                  APP_ERROR_CHECK(err_code); 
              }
          }

          else if (BTN_PWR == 1)
          {
              if (PWR_OFF_enabled == false)
              {
                  nrf_gpio_pin_toggle(bSTATE_LED);  // bSTATE_LED 1sec toggle 
                  //nrf_gpio_pin_clear(bPWR_OFF);   // bPWR_OFF OFF
                  //nrf_gpio_pin_set(bSTATE_LED);   // bSTATE_LED OFF
                  //nrf_gpio_pin_clear(bUWB_PWR_EN);   // bUWB_PWR_EN OFF   
                  if (Alert_check == 1)
                  {        
                      app_pwm_enable(&PWM1);
                      //err_code = app_pwm_channel_duty_set(&PWM1,1,50);
                      //APP_ERROR_CHECK(err_code);
                      err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
                      APP_ERROR_CHECK(err_code);     
                      nrf_delay_ms(20);     
                      app_pwm_disable(&PWM1);  
                  }
                  else if (Alert_check == 0)
                  {
                      app_pwm_disable(&PWM1);
                  }     
              }
              else if (PWR_OFF_enabled == true)
              {
                  nrf_gpio_pin_toggle(bSTATE_LED);  // bSTATE_LED 1sec toggle
                  //if (Alert_check == 1)
                  //{        
                  //    //app_pwm_enable(&PWM1);
                  //    ////err_code = app_pwm_channel_duty_set(&PWM1,1,50);
                  //    ////APP_ERROR_CHECK(err_code);
                  //    //err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
                  //    //APP_ERROR_CHECK(err_code);     
                  //    //nrf_delay_ms(500);     
                  //    //app_pwm_disable(&PWM1);  
                  //}
                  //else if (Alert_check == 0)
                  //{
                  //    app_pwm_disable(&PWM1);
                  //}
              }
          } 
          cnt = 0;
      } // if cnt == 4
      else
      {
          if (BTN_PWR == 0)
          {          
              if (PWR_OFF_enabled == false)
              {
                  nrf_gpio_pin_set(bPWR_OFF);   // bPWR_OFF High
                  nrf_gpio_pin_clear(bSTATE_LED); // bSTATE_LED ON

                  err_code = app_timer_start(m_pwr_timer_id, APP_TIMER_TICKS(1000), NULL);
                  APP_ERROR_CHECK(err_code); 
              }
              else if (PWR_OFF_enabled == true)
              {
                  nrf_gpio_pin_toggle(bSTATE_LED);  // bSTATE_LED 1sec toggle 
                  err_code = app_timer_start(m_pwr_timer_id, APP_TIMER_TICKS(3000), NULL);
                  APP_ERROR_CHECK(err_code); 
              }
          }

          else if (BTN_PWR == 1)
          {
              if (PWR_OFF_enabled == false)
              {
                  if (Alert_check == 1)
                  {        
                      app_pwm_enable(&PWM1);
                      //err_code = app_pwm_channel_duty_set(&PWM1,1,50);
                      //APP_ERROR_CHECK(err_code);
                      err_code = app_pwm_channel_duty_set(&PWM1,0,37.5);
                      APP_ERROR_CHECK(err_code);     
                      nrf_delay_ms(20);     
                      app_pwm_disable(&PWM1);  
                  }
                  else if (Alert_check == 0)
                  {
                      app_pwm_disable(&PWM1);
                  }     
              }
          } 
          cnt++;
      } // else 
}

static void btn_pwr_timeout_handler(void)
{
    if (PWR_OFF_enabled == true)
    {
          nrf_gpio_pin_clear(bPWR_OFF);   // bPWR_OFF OFF
          nrf_gpio_pin_set(bSTATE_LED);   // bSTATE_LED OFF
          //nrf_gpio_pin_clear(bUWB_PWR_EN);   // bUWB_PWR_EN OFF        
          PWR_OFF_enabled = false;
    }
    else if (PWR_OFF_enabled == false)
    { 
          nrf_gpio_pin_set(bPWR_OFF);   // bPWR_OFF On
          nrf_gpio_pin_clear(bSTATE_LED);   // bSTATE_LED On
          //nrf_gpio_pin_set(bUWB_PWR_EN);   // bUWB_PWR_EN On       
          PWR_OFF_enabled = true;
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
        // Initialize timer module.
        ret_code_t err_code = app_timer_init();
        APP_ERROR_CHECK(err_code);

        // Create timers.
        board_timer = xTimerCreate("IT1_board",
                                       BOARDCHECK_INTERVAL,
                                       pdTRUE,
                                       NULL,
                                       timer_event);

        err_code = app_timer_create(&m_pwr_timer_id,
                                    APP_TIMER_MODE_SINGLE_SHOT,
                                    btn_pwr_timeout_handler);
        APP_ERROR_CHECK(err_code);
                                    
        /* Error checking */
        if ( (NULL == board_timer))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }        
}

/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void board_timers_start(void)
{
        // Start application timers.
        if (pdPASS != xTimerStart(board_timer, OSTIMER_WAIT_FOR_QUEUE))
        {
                APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }      
}

/**@brief Function for application main entry.
 */
int main(void)
{
      bool erase_bonds;
      //uart_init();
      log_init();
      clock_init();
      nrf_drv_clock_lfclk_request(NULL);
      //APP_ERROR_CHECK(nrf_stack_guard_init());
    
      printf("UWB Tag Start.\n");
        
      if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 128, NULL, 2, &m_logger_thread))
      {
              APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
      }

      /* Configure and initialize the BLE stack. */
      ble_stack_init();

      /* Initialize modules. */
      //buttons_leds_init(&erase_bonds);
      IT1_board_init();
      gap_params_init();
      gatt_init();
      services_init();
      advertising_init();
      conn_params_init();
      //saadc_init();
      //saadc_sampling_event_init();
      //saadc_sampling_event_enable();
      pwm_init();
  
      
      /* UWB initialize */
      nrf52840_dk_spi_init();
      //dw_irq_init();
      //nrf_drv_gpiote_in_event_disable(DW3000_IRQn_Pin);
      nrf_delay_ms(2);

      // Timer for board check
      timers_init();
      board_timers_start();


      /* UWB Task start */
      if (pdPASS != xTaskCreate(ds_twr_init, "UWB", 1024, NULL, 1, &uwb_thread))
      {
              APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
      }

      /* Create a FreeRTOS task for the BLE stack. */
      nrf_sdh_freertos_init(advertising_start, &erase_bonds);

      /* Create queue to store round_ID that is accessed by BLE & UWB tasks */
      xQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);

      printf("UWB & BLE RTOS Start.\n");
      vTaskStartScheduler();
      for (;;)
      {
              APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
      }
}

void IT1_board_init(void)
{
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,20));
    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0,20));         // bPWR_OFF HIGH 
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,22));      // bUWB-PWR_EN
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,15));      //  bSTATE_LED 

    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0,15));   
    nrf_gpio_pin_set(bUWB_PWR_EN);   // bUWB_PWR_EN On     

    nrf_gpio_cfg_input(bCH_COMP, NRF_GPIO_PIN_PULLUP);    // bCH_COMP (LOW: Charging, HIGH: Charged)
    nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(1,0), NRF_GPIO_PIN_PULLUP);     // BTN_SOS
    nrf_gpio_cfg_input(NRF_GPIO_PIN_MAP(0,12), NRF_GPIO_PIN_PULLUP);    // BTN_SIG
}

int ds_twr_init(void)
{
    uint32_t ulNotifiedValue;

    Session_Data_t xReceivedSession;
    BaseType_t xStatus;
    const TickType_t xTicksToWait = pdMS_TO_TICKS(1); 

    nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0,22)); 

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

    if(dwt_configure(&config)) 
    {
        while (1)
        { };
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    //dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    //dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    while(1) 
    {
       // xLastWakeTime = xTaskGetTickCount();
        //vTaskDelayUntil(&xLastWakeTime,  pdMS_TO_TICKS(240));
        //nrf_gpio_pin_toggle(NRF_GPIO_PIN_MAP(0,24)); 
      if(sess_check == 1)
      {
        xStatus = xQueueReceive(xQueue, &xReceivedSession, xTicksToWait);
        if(xStatus == pdPASS)
        {
            printf("Round ID : %d\n\n", xReceivedSession.round_ID);
        }

        //const TickType_t xWakePeriod = 240*portTICK_PERIOD_MS - 15*(xReceivedSession.round_ID) -10*portTICK_PERIOD_MS;
        //vTaskResume(uwb_thread);
        uint32_t rcm_rx_time;
        do{
           rcm_rx_time=rcm_rx();
        }while((rcm_rx_time == 0) || (rcm_rx_time == 1));

        dwt_setreferencetrxtime(rcm_rx_time);
        ranging_time = RANGING_DWTIME * xReceivedSession.round_ID + START_TX_DWTIME - TX_ANT_DLY;
        dwt_setdelayedtrxtime(ranging_time);
         
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_poll_msg)+FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging. */

        dwt_starttx(DWT_START_TX_DLY_REF | DWT_RESPONSE_EXPECTED);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
        { };	

        frame_seq_nb++;

        if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
        {
            uint32_t frame_len;

            /* Clear good RX frame event and TX frame sent in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_TXFRS_BIT_MASK);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & FRAME_LEN_MAX_EX;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            /* Check that the frame is the expected response from the companion "DS TWR responder" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;
            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32_t final_tx_time;
                int ret;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 11 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 12 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message. See NOTE 9 below. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0); /* Zero offset in TX buffer. */
                dwt_writetxfctrl(sizeof(tx_final_msg)+FCS_LEN, 0, 1); /* Zero offset in TX buffer, ranging bit set. */

                ret = dwt_starttx(DWT_START_TX_DELAYED);

                /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 13 below. */
                if (ret == DWT_SUCCESS)
                {
                    /* Poll DW IC until TX frame sent event set. See NOTE 10 below. */
                    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
                    { };

                    printf("DS_TWR Successed!\n\n\n");
                    //nrf_gpio_pin_toggle(NRF_GPIO_PIN_MAP(0,24));

                    /* Clear TXFRS event. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
                             
                    /* Increment frame sequence number after transmission of the final message (modulo 256). */
                    frame_seq_nb++;
                }
            }
        }
        else
        {
            /* Clear RX error/timeout events in the DW IC status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_TXFRS_BIT_MASK);
        }
	//nrf_gpio_pin_toggle(NRF_GPIO_PIN_MAP(0,24));
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(216));
     }// if sess_check == 0
  }// while(1)
}

int rcm_rx(void)
{
      //printf("@@@@@@@@@@@rcm RX start@@@@@@@@@@@\n\n");
      
      uint32_t rcm_rx_ts;

      memset(rx_buffer,0,sizeof(rx_buffer));

      dwt_rxenable(DWT_START_RX_IMMEDIATE);

      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
      {
          //if (sess_check == 0)
          //{
              //dwt_forcetrxoff();
              //vTaskSuspend(uwb_thread);
          //}
      };
      
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
                  rcm_rx_ts = dwt_readrxtimestamphi32(); /* Retrieve RCM reception timestamp to fuction parameter */
                  xLastWakeTime = xTaskGetTickCount();
              }
              else
              {
                  rcm_rx_ts = 0;
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