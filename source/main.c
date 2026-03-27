/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 4 - SPI Master
*              for ModusToolbox.
*
* Related Document: See README.md 
*
*
*******************************************************************************
* (c) 2020-2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "SpiMaster.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Number of elements in the transmit buffer */
/* There are three elements - one for head, one for command and one for tail */
#define NUMBER_OF_ELEMENTS  (3UL)
#define SIZE_OF_ELEMENT     (1UL)
#define SIZE_OF_PACKET      (NUMBER_OF_ELEMENTS * SIZE_OF_ELEMENT)


/*******************************************************************************
 * Function Name: main
 *******************************************************************************
 * Summary:
 * Entry function for the application. This function sets up the SPI Master.
 * SPI master sends commands to the slave to turn LED ON or OFF, every
 * second using high level APIs.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  int
 *
 ******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_scb_spi_status_t status;
    uint32_t cmd_send = CYBSP_LED_STATE_OFF;

    /* Buffer to hold command packet to be sent to the slave by the master */
    uint8_t  txBuffer[NUMBER_OF_ELEMENTS];

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the SPI Master */
    result = initMaster();
    /* Initialization failed. Stop program execution */
    if(result != INIT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    for(;;)
    {
        /* Toggle the current LED state */
        cmd_send = (cmd_send == CYBSP_LED_STATE_OFF) ? CYBSP_LED_STATE_ON :
                                                       CYBSP_LED_STATE_OFF;

        /* Form the command packet */
        txBuffer[PACKET_SOP_POS] = PACKET_SOP;
        txBuffer[PACKET_CMD_POS] = cmd_send;
        txBuffer[PACKET_EOP_POS] = PACKET_EOP;

        /* Send the packet to the slave */
        status = sendPacket(txBuffer, SIZE_OF_PACKET);
        if(status != CY_SCB_SPI_SUCCESS)
        {
            CY_ASSERT(0);
        }

        /* Delay between commands */
        Cy_SysLib_Delay(1000);
    }
}

/* [] END OF FILE */
