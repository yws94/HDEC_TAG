/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <deca_device_api.h>
#include "port.h"
#include "deca_spi.h"

//static
//spi_handle_t spi_handler = {
///* below will be configured in the port_init_dw_chip() */
//  .spi_inst       = 0,
//  .frequency_slow = 0, 
//  .frequency_fast = 0,
//  .spi_config     = 0,
//
//  .csPin          = DW3000_CS_Pin,
//  .lock           = DW_HAL_NODE_UNLOCKED
//};

static spi_handle_t pgSpiHandler = { 0 };
static volatile bool spi_xfer_done;
static uint8_t spi_init_stat = 0; // use 1 for slow, use 2 for fast;

static uint8_t idatabuf[DATALEN1]={0}; //Never define this inside the Spi read/write
static uint8_t itempbuf[DATALEN1]={0}; //As that will use the stack from the Task, which are not such long!!!!
                                     //You will face a crashes which are not expected!

/****************************************************************************//**
 *
 *                              DW3000 SPI section
 *
 *******************************************************************************/

/* @fn    nrf52840_dk_spi_init
 * Initialise nRF52840-DK SPI
 * */
void nrf52840_dk_spi_init(void)
{
    nrf_drv_spi_t   *spi_inst;
    nrf_drv_spi_config_t  *spi_config;

    pgSpiHandler.frequency_slow = NRF_SPIM_FREQ_4M;
    pgSpiHandler.frequency_fast = NRF_SPIM_FREQ_32M;
    pgSpiHandler.csPin = DW3000_CS_Pin;
    pgSpiHandler.lock = DW_HAL_NODE_UNLOCKED;

    spi_inst = &pgSpiHandler.spi_inst;
    spi_config = &pgSpiHandler.spi_config;
    spi_inst->inst_idx = SPI3_INSTANCE_INDEX;
    spi_inst->use_easy_dma = SPI3_USE_EASY_DMA;
    spi_inst->u.spim.p_reg = NRF_SPIM3;
    spi_inst->u.spim.drv_inst_idx = NRFX_SPIM3_INST_IDX;

    spi_config->sck_pin = DW3000_CLK_Pin;
    spi_config->mosi_pin = DW3000_MOSI_Pin;
    spi_config->miso_pin = DW3000_MISO_Pin;
    spi_config->ss_pin = DW3000_CS_Pin;
    spi_config->irq_priority = (APP_IRQ_PRIORITY_MID - 2);
    spi_config->orc = 0xFF;
    spi_config->frequency = pgSpiHandler.frequency_slow;
    spi_config->mode = NRF_DRV_SPI_MODE_0;
    spi_config->bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

    port_set_dw_ic_spi_slowrate();
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
static int openspi(nrf_drv_spi_t *p_instance)
{
    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_enable(p_spi);
    return 0;
} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
static int closespi(nrf_drv_spi_t *p_instance)
{
    NRF_SPIM_Type * p_spi = p_instance->u.spim.p_reg;
    nrf_spim_disable(p_spi);
    return 0;
} // end closespi()

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                              void *p_context)
{
    UNUSED_PARAMETER(p_event);
    UNUSED_PARAMETER(p_context);
    spi_xfer_done = true;
}

/* @fn      port_set_dw_ic_spi_slowrate
 * @brief   set 2MHz
 * */
void port_set_dw_ic_spi_slowrate(void)
{
    //set_SPI_master();
    if(spi_init_stat == 1)
    {
        return;
    }
    else
    {
        if(spi_init_stat == 2)
        {
            nrf_drv_spi_uninit(&pgSpiHandler.spi_inst);
        }

//        pgSpiHandler.spi_config.frequency = spi_handler.frequency_slow;

        APP_ERROR_CHECK(nrf_drv_spi_init(&pgSpiHandler.spi_inst,
                                         &pgSpiHandler.spi_config,
                                         spi_event_handler,
                                         NULL) );

        spi_init_stat = 1;

        nrf_delay_ms(2);
    }
}

/* @fn      port_set_dw_ic_spi_fastrate
 * @brief   set 16MHz
 * */
void port_set_dw_ic_spi_fastrate(void)
{
    //set_SPI_master();
    if(spi_init_stat == 2)
    {
        return;
    }
    else
    {
        if(spi_init_stat == 1)
        {
            nrf_drv_spi_uninit(&pgSpiHandler.spi_inst);
        }
//        pgSpiHandler.spi_config.frequency = spi_handler.frequency_fast;

        APP_ERROR_CHECK( nrf_drv_spi_init(&pgSpiHandler.spi_inst,
                                          &pgSpiHandler.spi_config,
                                          spi_event_handler,
                                          NULL) );

        spi_init_stat = 2;
        nrf_delay_ms(2);
    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte which is written last
 * returns 0 for success, or -1 for error
 */
int writetospiwithcrc(uint16_t      headerLength,
                      const uint8_t *headerBuffer,
                      uint16_t      bodyLength,
                      const uint8_t *bodyBuffer,
                      uint8_t       crc8)
{
    uint8_t* p1;
    uint32_t idatalength = headerLength + bodyLength + sizeof(crc8);

    if (idatalength > DATALEN1)
    {
        return NRF_ERROR_NO_MEM;
    }

    while(pgSpiHandler.lock);

    __HAL_LOCK(pgSpiHandler);

    openspi(&pgSpiHandler.spi_inst);

    p1 = idatabuf;
    memcpy(p1, headerBuffer, headerLength);
    p1 += headerLength;
    memcpy(p1, bodyBuffer, bodyLength);
    p1 += bodyLength;
    memcpy(p1, &crc8, 1);

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&pgSpiHandler.spi_inst, idatabuf, idatalength, itempbuf, idatalength);
    while(!spi_xfer_done);

    closespi(&pgSpiHandler.spi_inst);

     __HAL_UNLOCK(pgSpiHandler);

    return 0;
} // end writetospiwithcrc()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16_t       headerLength,
               const uint8_t  *headerBuffer,
               uint16_t       bodyLength,
               const uint8_t  *bodyBuffer)
{
    uint8_t* p1;
    uint32_t idatalength = headerLength + bodyLength;

    if (idatalength > DATALEN1)
    {
        return NRF_ERROR_NO_MEM;
    }

    while(pgSpiHandler.lock);

    __HAL_LOCK(pgSpiHandler);

    openspi(&pgSpiHandler.spi_inst);

    p1 = idatabuf;
    memcpy(p1, headerBuffer, headerLength);
    p1 += headerLength;
    memcpy(p1, bodyBuffer, bodyLength);

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&pgSpiHandler.spi_inst, idatabuf, idatalength, itempbuf, idatalength);
    while(!spi_xfer_done);

    closespi(&pgSpiHandler.spi_inst);

     __HAL_UNLOCK(pgSpiHandler);

    return 0;
} // end writetospi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
int readfromspi(uint16_t  headerLength,
                uint8_t   *headerBuffer,
                uint16_t  readLength,
                uint8_t   *readBuffer)
{
    uint8_t* p1;
    uint32_t idatalength = headerLength + readLength;

    if (idatalength > DATALEN1)
    {
        return NRF_ERROR_NO_MEM;
    }

    while(pgSpiHandler.lock);

    __HAL_LOCK(pgSpiHandler);

    openspi(&pgSpiHandler.spi_inst);

    p1 = idatabuf;
    memcpy(p1, headerBuffer, headerLength);

    p1 += headerLength;
    memset(p1, 0x00, readLength);

    idatalength = headerLength + readLength;

    spi_xfer_done = false;
    nrf_drv_spi_transfer(&pgSpiHandler.spi_inst, idatabuf, idatalength, itempbuf, idatalength);
    while(!spi_xfer_done);

    p1 = itempbuf + headerLength;
    memcpy(readBuffer, p1, readLength);

    closespi(&pgSpiHandler.spi_inst);

    __HAL_UNLOCK(pgSpiHandler);

    return 0;
} // end readfromspi()

/****************************************************************************//**
 *
 *                              END OF DW3000 SPI section
 *
 *******************************************************************************/