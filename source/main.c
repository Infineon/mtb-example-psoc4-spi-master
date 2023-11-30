/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the PSoC 4 - SPI Master
 *             for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2020-2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
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
