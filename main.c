/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code of TCPWM Dead-Time Mode example
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This main achieve the PWM output with dead time mode. PWM line pin and
* PWM compl_pin output the 1KHz wave with left side dead time 250[us]
* Return: int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t rslt;

    /* Initialize the device and board peripherals */
    rslt = cybsp_init();
    if (rslt != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize PWM                                                  */
    /* The dead-Time is set at 250 clock cycles with 1MHz source clock */
    /* It means the dead-time is 250us                                 */
    Cy_TCPWM_PWM_Init(TCPWM_PWM_HW, TCPWM_PWM_NUM, &TCPWM_PWM_config);

    /* Enable PWM */
    Cy_TCPWM_PWM_Enable(TCPWM_PWM_HW, TCPWM_PWM_NUM);

    /* Trigger to start PWM */
    Cy_TCPWM_TriggerStart_Single(TCPWM_PWM_HW, TCPWM_PWM_NUM);

    for (;;)
    {
    }
}

/* [] END OF FILE */