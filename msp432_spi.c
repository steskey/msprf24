#include <msp432.h>
#include "msp432_spi.h"
#include "nrf_userconfig.h"
#include "driverlib.h"


/* USCI 16-bit transfer functions rely on the Little-Endian architecture and use
 * an internal uint8_t * pointer to manipulate the individual 8-bit segments of a
 * 16-bit integer.
 */

void spi_init()
{

    /* SPI Master Configuration Parameter */
    const eUSCI_SPI_MasterConfig spiMasterConfig =
    {
            EUSCI_B_SPI_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
          24000000,                                   // SMCLK 3Mhz
            500000,                                    // SPICLK = 500khz
            EUSCI_B_SPI_MSB_FIRST,                     // MSB First
            EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,    // Phase
            EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // Low polarity
            EUSCI_B_SPI_3PIN                           // 3Wire SPI Mode
    };

    /* Selecting P3.5(SCLK) P3.6(MOSI) and P3.7(MISO) in SPI mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
            GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_B2_SPI_BASE, &spiMasterConfig);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_B2_SPI_BASE);
}

uint8_t spi_transfer(uint8_t inb)
{
    /* Wait until previous transmission is over */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_TXIFG));
    // load byte into transmit buffer and send
    EUSCI_B2->TXBUF = inb;
    /* Wait until current transmission is complete */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_RXIFG));
    // return received data byte
    return EUSCI_B2->RXBUF;
}

uint16_t spi_transfer16(uint16_t inw)
{
    uint16_t retw;
    uint8_t *retw8 = (uint8_t *)&retw, *inw8 = (uint8_t *)&inw;

    /* Wait until previous transmission is over */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_TXIFG));
    // load 1st byte into transmit buffer and send
    EUSCI_B2->TXBUF = inw8[1];
    /* Wait until current transmission is complete */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_RXIFG));
    // load received data into return variable upper byte
    retw8[1] = EUSCI_B2->RXBUF;
    // load 2nd byte into transmit buffer and send
    EUSCI_B2->TXBUF = inw8[0];
    /* Wait until current transmission is complete */
    while (!(EUSCI_B2->IFG&EUSCI_A_IFG_RXIFG));
    // load received data into return variable lower byte
    retw8[0] = EUSCI_B2->RXBUF;
    return retw;
}

