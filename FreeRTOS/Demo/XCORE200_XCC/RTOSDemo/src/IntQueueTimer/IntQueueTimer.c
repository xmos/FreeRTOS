// Copyright (c) 2019, XMOS Ltd, All rights reserved

#include "FreeRTOS.h"
#include "task.h"

#include "IntQueue.h"

static void _hwtimer_get_trigger_time( hwtimer_t t, uint32_t *time )
{
	asm volatile("getd %0, res[%1]" : "=r" (*time): "r" (t));
}

static xcore_c_error_t hwtimer_get_trigger_time( hwtimer_t t, uint32_t *time )
{
	RETURN_EXCEPTION_OR_ERROR( _hwtimer_get_trigger_time( t, time ) );
}

DEFINE_RTOS_INTERRUPT_CALLBACK( pxIntQueueTimerISR, pvData )
{
	static int xCount0, xCount1;
	hwtimer_t xTimer = ( hwtimer_t ) pvData;
	uint32_t ulNow;
	int xYieldRequired = pdFALSE;

	hwtimer_get_time( xTimer, &ulNow );
	hwtimer_get_trigger_time( xTimer, &ulNow );
	ulNow += configCPU_CLOCK_HZ / 50;

	if( ++xCount0 == 2 )
	{
		xCount0 = 0;
		taskENTER_CRITICAL_FROM_ISR();
		if( xFirstTimerHandler() != pdFALSE )
		{
			xYieldRequired = pdTRUE;
		}
		taskEXIT_CRITICAL_FROM_ISR( 0 );
	}

	if( ++xCount1 == 3 )
	{
		xCount1 = 0;
		taskENTER_CRITICAL_FROM_ISR();
		if( xSecondTimerHandler() != pdFALSE )
		{
			xYieldRequired = pdTRUE;
		}
		taskEXIT_CRITICAL_FROM_ISR( 0 );
	}

	hwtimer_change_trigger_time( xTimer, ulNow );

	portYIELD_FROM_ISR( xYieldRequired );
}

void vInitialiseTimerForIntQueueTest( void )
{
uint32_t ulNow;
uint32_t ulState;
hwtimer_t xIntQueueTimer;

	/*
	 * Disable interrupts here so we stay on the same core
	 */
	ulState = portDISABLE_INTERRUPTS();
	{
		hwtimer_alloc( &xIntQueueTimer );
		hwtimer_get_time( xIntQueueTimer, &ulNow );
		ulNow += configCPU_CLOCK_HZ / 50;
		hwtimer_setup_interrupt_callback( xIntQueueTimer, ulNow, ( void * ) xIntQueueTimer, RTOS_INTERRUPT_CALLBACK( pxIntQueueTimerISR ) );
		hwtimer_enable_trigger( xIntQueueTimer );
	}
	portRESTORE_INTERRUPTS( ulState );
}
