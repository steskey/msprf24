// AUTHOR: Chris van Zomeren

/* DriverLib Includes */
#include "driverlib.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "msprf24.h"



void clockInit48MHzXTL(void) {  // sets the clock module to use the external 48 MHz crystal

    /* Configuring pins for peripheral/crystal usage */
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ,
            GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    CS_setExternalClockSourceFrequency(32000,48000000); // enables getMCLK, getSMCLK to know externally set frequencies

    /* Starting HFXT in non-bypass mode without a timeout. Before we start
     * we have to change VCORE to 1 to support the 48MHz frequency */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);
    CS_startHFXT(false);  // false means that there are no timeouts set, will return when stable

    /* Initializing MCLK to HFXT (effectively 48MHz) */
    MAP_CS_initClockSignal(CS_MCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);
    MAP_CS_initClockSignal(CS_SMCLK, CS_HFXTCLK_SELECT, CS_CLOCK_DIVIDER_2);
}



int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();

    clockInit48MHzXTL();
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P2, GPIO_PIN5, GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN5);
    MAP_Interrupt_enableInterrupt(INT_PORT2);
    MAP_Interrupt_enableMaster();

    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1);

    rf_crc = RF24_EN_CRC | RF24_CRCO; // CRC enabled, 16-bit
    rf_addr_width      = 5;
    rf_speed_power     = RF24_SPEED_1MBPS | RF24_POWER_0DBM;
    rf_channel         = 42;

    uint8_t buf[32] = {0};
    uint8_t num_retransmits = 0;

    msprf24_init();

    while(!msprf24_is_alive()) {}

    msprf24_set_pipe_packetsize(0, 32);
    msprf24_open_pipe(0, 1);

    msprf24_standby();

    //char *const rx_addr = "\xde\xad\xbe\xef";
    //char *const rx_addr = "\x15\x77\x42\xAF\x95";
    char *const rx_addr = "BEANS";

    w_tx_addr(rx_addr);
    w_rx_addr(0, rx_addr);

    while(1){
            int i;
            for(i = 0; i < 3; i++) __delay_cycles(400000);
            if(buf[0]=='0'){buf[0] = '1';buf[1] = '0';}
            else {buf[0] = '0';buf[1] = '1';}
            w_tx_payload(32, buf);
            msprf24_activate_tx();

            MAP_PCM_gotoLPM4();

            if(buf[0]=='0')
            {
                P2OUT &= ~BIT0;  // Red LED on
            }
            else
            {
                P2OUT |= BIT0; // Red LED off
            }

            if (rf_irq & RF24_IRQ_FLAGGED) {
                msprf24_get_irq_reason();
                if (rf_irq & RF24_IRQ_TXFAILED){
                    P2OUT &= ~BIT1; // Green LED off
                }
                else
                {
                    P2OUT |= BIT1; // Green LED on
                }

                msprf24_irq_clear(rf_irq);
                num_retransmits = msprf24_get_last_retransmits();
            }

            //__delay_cycles(400000);
            //MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1);
        }
}
