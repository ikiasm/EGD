/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <proc/p32mx795f512l.h>

#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
static int tickActualIny = 0;
static int tickActualRpm = 0;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/
void pulsoIny1();
void pulsoIny2();
void pulsoIny3();
void pulsoIny4();

void pulsoBanco1();
void pulsoBanco2();

void tick10usIny();
void tick50usRpm();

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    
    TRISCbits.TRISC15 = 0;

}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
//            bool appInitialized = true;
//       
//        
//            if (appInitialized)
//            {
//            
//                appData.state = APP_STATE_SERVICE_TASKS;
//            }
            
            //DRV_OC0_Start();
            //DRV_OC1_Start();
            DRV_TMR0_Start();
            DRV_TMR1_Start();
            DRV_TMR2_Start();
            
            appData.state = APP_STATE_SERVICE_TASKS;
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {           
            
            pulsoBanco1();
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

/*
 * pulsoInyX()
 * funciones listas para recibir tipo de dato (int periodo, int duty, int duracion)
 */

void pulsoBanco1()
{
    static int stage = 0;
    static int tiempoIny = 0;
    int duty ,aux, periodo, duracion, dutyNeg, dutyPos;
    switch(stage)
    {
        case 0:
            tickActualIny = 0;
            //LATCbits.LATC15 = 1;
            stage++;
            break;
        case 1:
            aux = 0;
            duracion = 57;  //valor multiplicado por 10us
            periodo = 800; //periodo para 50kHz con clock de 40MHz
            dutyPos = 32;
            dutyNeg = (100 - dutyPos);
            duty = dutyPos; //dutyNeg;
            aux = (duty*periodo)/100;
            DRV_TMR1_PeriodValueSet(periodo);
            DRV_OC0_Duty(aux);
            DRV_OC0_Start();
            if(tickActualIny > duracion)
            {
                stage++;
                tickActualIny = 0;
                tiempoIny = tiempoIny + duracion;
            }            
            break;
        case 2:
            aux = 0;
            duracion = 55;
            periodo = 800;
            dutyPos = 83;
            dutyNeg = (100 - dutyPos);
            duty = dutyPos; //dutyNeg;
            aux = (duty*periodo)/100;
            DRV_OC0_Duty(aux);
            if(tickActualIny > duracion)
            {
                stage++;
                tickActualIny = 0;
                tiempoIny = tiempoIny + duracion;
            }
            break;
        case 3:
            aux = 0;
            duracion = 54;
            periodo = 2000;//periodo para 20kHz con clock de 40MHz
            dutyPos = 90;//90;
            dutyNeg = (100 - dutyPos);
            duty = dutyPos; //dutyNeg;
            DRV_TMR1_PeriodValueSet(periodo);
            aux = (duty*periodo)/100;
            DRV_OC0_Duty(aux);
            if(tickActualIny > duracion)
            {
                stage++;
                tickActualIny = 0;
                DRV_TMR1_PeriodValueSet(800);//importante para que tenga tiempo 
                tiempoIny = tiempoIny + duracion;//de setear el periodo y no se cuelgue
            }                                
            break;
        case 4:
            //Posiblemente haya que sacar este case
            //Corresponderia al apagado del Transitor de referencia
            DRV_OC0_Stop();
            duracion = 20;
            TRISDbits.TRISD1 = 0;
            LATDbits.LATD1 = 1;
            if(tickActualIny > duracion)
            {
                stage++; 
                tickActualIny = 0;
                LATDbits.LATD1 = 0;
                tiempoIny = tiempoIny + duracion;
                LATCbits.LATC15 = 0;
                stage = 0;
            }
            break;
        case 5://case para realizar pruebas, no iria en el definitorio.
            if(tickActualIny > (3160 - tiempoIny))
            {
                stage = 0;
                tickActualIny = 0;
                tiempoIny = 0;
            }
            break;
        default:
            break;
    }
}

void pulsoBanco2()
{
    
}
void pulsoIny1()
{
   
}
void pulsoIny2()
{
    
}
void pulsoIny3()
{
    
}
void pulsoIny4()
{
    
}

void tick10usIny()
{
    tickActualIny++;
}

void tick50usRpm()
{
    tickActualRpm++;
}

void prueba()
{
    //DRV_OC1_Start();

    LATCbits.LATC15 = 1;
    LATCbits.LATC15 = 0;
}
