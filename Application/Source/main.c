/*******************************************************************************
*
* E M B E D D E D   W I Z A R D   P R O J E C T
*
*                                                Copyright (c) TARA Systems GmbH
*                                    written by Paul Banach and Manfred Schweyer
*
********************************************************************************
*
* This software is delivered "as is" and shows the usage of other software
* components. It is provided as an example software which is intended to be
* modified and extended according to particular requirements.
*
* TARA Systems hereby disclaims all warranties and conditions with regard to the
* software, including all implied warranties and conditions of merchantability
* and non-infringement of any third party IPR or other rights which may result
* from the use or the inability to use the software.
*
********************************************************************************
*
* DESCRIPTION:
*   This file implements the main.c module for running Embedded Wizard
*   generated GUI applications on a dedicated target with or without the usage
*   of an operating system.
*
*   Important: Please be aware that every Embedded Wizard GUI application
*   requires the execution in a single GUI task!
*   If you are working with an operating system and your software is using
*   several threads/tasks, please take care to access your GUI application
*   only within the context of your GUI thread/task. Use operating system
*   services to exchange data or events between the GUI thread/task and other
*   worker threads/tasks.
*
*   For more information concerning the integration of an Embedded Wizard
*   generated GUI application into your main application, please see
*   https://doc.embedded-wizard.de/main-loop
*
*******************************************************************************/

#include "ewmain.h"
#include "ewrte.h"
#include "ew_bsp_system.h"
#include "ew_bsp_console.h"
#include "BSP_HeartRate.h"
#include "BSP_Clock.h"

#if EW_USE_FREE_RTOS == 1

  #include "cmsis_os.h"

  #define semtstSTACK_SIZE    configMINIMAL_STACK_SIZE * 10

  static void GuiThread( const void* arg );

#endif


#if EW_USE_FREE_RTOS == 0

/*******************************************************************************
* FUNCTION:
*   main
*
* DESCRIPTION:
*   The main function for running Embedded Wizard generated GUI applications on
*   a dedicated target without using any operating system (bare metal).
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
int main( void )
{
  /* initialize system */
  EwBspSystemInit();

  /* initialize console interface for debug messages */
  EwBspConsoleInit();

  /* initialize Embedded Wizard application */
  if ( EwInit() == 0 )
    return 0;
    
  clock_init();
  heartRate_init();

  EwPrintSystemInfo();

  /* process the Embedded Wizard main loop */
  while( EwProcess())
    ;

  /* de-initialize Embedded Wizard application */
  EwDone();

  /* terminate the system */
  EwBspSystemDone();

  return 0;
}

#endif

#if EW_USE_FREE_RTOS == 1

/*******************************************************************************
* FUNCTION:
*   main
*
* DESCRIPTION:
*   The main function for running Embedded Wizard generated GUI applications on
*   a dedicated target using the FreeRTOS operating system.
*
* ARGUMENTS:
*   None
*
* RETURN VALUE:
*   None
*
*******************************************************************************/
int main( void )
{
  /* initialize system */
  EwBspSystemInit();

  /* initialize console interface for debug messages */
  EwBspConsoleInit();

  /* create thread that drives the Embedded Wizard GUI application... */
  EwPrint( "Create UI thread...                          " );
  osThreadDef( EwThreadHandle, GuiThread, osPriorityNormal, 0, semtstSTACK_SIZE );
  osThreadCreate( osThread( EwThreadHandle ), (void*)0 );
  EwPrint( "[OK]\n" );

  /* ...and start scheduler */
  osKernelStart();

  /* terminate the system */
  EwBspSystemDone();

  return 0;
}


/*******************************************************************************
* FUNCTION:
*   GuiThread
*
* DESCRIPTION:
*   The EwThread processes the Embeded Wizard application.
*
* ARGUMENTS:
*   arg - not used.
*
* RETURN VALUE:
*   None.
*
*******************************************************************************/
static void GuiThread( const void* arg )
{
  /* initialize Embedded Wizard application */
  if ( EwInit() == 0 )
    return;

  EwPrintSystemInfo();

  /* process the Embedded Wizard main loop */
  while( EwProcess())
    ;

  /* de-initialize Embedded Wizard application */
  EwDone();
}

#endif

/* msy, mli */
