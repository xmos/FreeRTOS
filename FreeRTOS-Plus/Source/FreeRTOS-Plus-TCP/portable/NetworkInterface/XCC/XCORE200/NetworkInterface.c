/*
FreeRTOS+TCP V2.0.11
Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 http://aws.amazon.com/freertos
 http://www.FreeRTOS.org
*/


/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_DNS.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"

#include "FreeRTOS_TCP_port.h"

#if ipconfigZERO_COPY_RX_DRIVER == 0 || ipconfigZERO_COPY_TX_DRIVER == 0
#error The xcore TCP/IP driver requires both RX and TX zero copy
#endif

#define PRINT_MAC_ADDR( A ) debug_printf("%x:%x:%x:%x:%x:%x\n", A[0], A[1], A[2], A[3], A[4], A[5])

#define ETH_BUF_SIZE (ipconfigNETWORK_MTU + ipSIZE_OF_ETH_HEADER)

/* Ethernet device */
static soc_peripheral_t xEthDev;

#if TCPPORT_FREE_TX_FRAMES_ASAP == 1
    portTIMER_CALLBACK_ATTRIBUTE
    static void vDeferredTransmit( void *dummy1, uint32_t dummy2 )
    {
        NetworkBufferDescriptor_t *pxNetworkBuffer;
        uint8_t *pucBuf;
        soc_dma_ring_buf_t *tx_ring_buf = soc_peripheral_tx_dma_ring_buf(xEthDev);

        (void) dummy1;
        (void) dummy2;

        /* Free any tx buffers that have already been processed */
        while( ( pucBuf = soc_dma_ring_tx_buf_get( tx_ring_buf, NULL, NULL ) ) != NULL )
        {
            pxNetworkBuffer = pxPacketBuffer_to_NetworkBuffer(pucBuf);
            configASSERT( pxNetworkBuffer != NULL );
            configASSERT( pxNetworkBuffer->pucEthernetBuffer == pucBuf );
            vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
        }
    }
#endif

portTIMER_CALLBACK_ATTRIBUTE
static void vDeferredReceive( uint8_t *frame_buffer, int frame_length )
{
    NetworkBufferDescriptor_t *pxNetworkBuffer;
    soc_dma_ring_buf_t *rx_ring_buf = soc_peripheral_rx_dma_ring_buf(xEthDev);

    /* Allocate a new network buffer for the DMA engine to
    replace the one it just gave us. */
    pxNetworkBuffer = pxGetNetworkBufferWithDescriptor(ETH_BUF_SIZE, 0);

    if( pxNetworkBuffer != NULL )
    {
        IPStackEvent_t xRxEvent = { eNetworkRxEvent, NULL };

        soc_dma_ring_rx_buf_set(rx_ring_buf, pxNetworkBuffer->pucEthernetBuffer, ETH_BUF_SIZE);
        soc_peripheral_hub_dma_request(xEthDev, SOC_DMA_RX_REQUEST);

        /* Retrieve the network buffer from the packet buffer that the
        DMA received the new frame into */
        pxNetworkBuffer = pxPacketBuffer_to_NetworkBuffer(frame_buffer);
        configASSERT( pxNetworkBuffer != NULL );
        configASSERT( pxNetworkBuffer->xDataLength >= ETH_BUF_SIZE );
        configASSERT( pxNetworkBuffer->pucEthernetBuffer == frame_buffer );
        pxNetworkBuffer->xDataLength = frame_length;

        xRxEvent.pvData = ( void * ) pxNetworkBuffer;

        /* Data was received and stored.  Send a message to the IP
        task to let it know. */
        if( xSendEventStructToIPTask( &xRxEvent, ( TickType_t ) 0 ) == pdFAIL )
        {
            vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );
            iptraceETHERNET_RX_EVENT_LOST();
            debug_printf("eth data lost 1\n", frame_length);
        }

        iptraceNETWORK_INTERFACE_RECEIVE();
    }
    else
    {
        /* There is not a new network buffer available to give the DMA
        engine to replace the one it just gave us. So we'll just give this
        one back and drop the frame. */

        iptraceETHERNET_RX_EVENT_LOST();
        debug_printf("eth data lost 2\n", frame_length);

        /* Give the buffer back to the DMA engine */
        soc_dma_ring_rx_buf_set(rx_ring_buf, frame_buffer, ETH_BUF_SIZE);
        soc_peripheral_hub_dma_request(xEthDev, SOC_DMA_RX_REQUEST);
    }

    #if TCPPORT_FREE_TX_FRAMES_ASAP == 1
        /* Free any tx buffers that have already been processed.
        This is just in case xTimerPendFunctionCallFromISR ever
        fails in the TX complete case. */
        vDeferredTransmit(NULL, 0);
    #endif
}

RTOS_IRQ_ISR_ATTR
void xEth_ISR(soc_peripheral_t device)
{
    BaseType_t xYieldRequired = pdFALSE;
    uint32_t status;

    status = soc_peripheral_interrupt_status(device);

    if (status & SOC_PERIPHERAL_ISR_DMA_RX_DONE_BM) {
        soc_dma_ring_buf_t *rx_ring_buf;
        uint8_t *frame_buffer;
        int frame_length;

        rx_ring_buf = soc_peripheral_rx_dma_ring_buf(device);

        while( ( frame_buffer = soc_dma_ring_rx_buf_get(rx_ring_buf, &frame_length) ) != NULL )
        {
            BaseType_t xResult;
            xResult = xTimerPendFunctionCallFromISR(
                    (PendedFunction_t) vDeferredReceive,
                    frame_buffer,
                    frame_length,
                    &xYieldRequired );

            if (xResult != pdPASS)
            {
                iptraceETHERNET_RX_EVENT_LOST();
                debug_printf("eth data lost 3\n", frame_length);

                /* Give the buffer back to the DMA engine */
                soc_dma_ring_rx_buf_set(rx_ring_buf, frame_buffer, ETH_BUF_SIZE);
                soc_peripheral_hub_dma_request(device, SOC_DMA_RX_REQUEST);
            }
        }
    }

    #if TCPPORT_FREE_TX_FRAMES_ASAP == 1
        if (status & SOC_PERIPHERAL_ISR_DMA_TX_DONE_BM) {
            xTimerPendFunctionCallFromISR( (PendedFunction_t) vDeferredTransmit, NULL, 0, &xYieldRequired );
        }
    #endif

    portEND_SWITCHING_ISR( xYieldRequired );
}

BaseType_t xNetworkInterfaceInitialise( void )
{
BaseType_t xReturn, xStatus;
int i;
uint8_t ucMACAddrRTOS[ ipMAC_ADDRESS_LENGTH_BYTES ];
uint8_t ucMACAddrHW[ ipMAC_ADDRESS_LENGTH_BYTES ];

    /* Initalize static items */
    do
    {
        xStatus = xInit_RNG();

        if( xStatus == pdFAIL )
        {
            break;
        }

        if( xEthDev == NULL )
        {
            xEthDev = ethernet_driver_init(
                    BITSTREAM_ETHERNET_DEVICE_A,          /* Initializing ethernet device A */
                    TCPPORT_ETH_DMA_RX_DESCRIPTOR_COUNT,  /* The number of DMA RX buffer descriptors */
                    0,                                    /* Do not allocate any RX buffers yet */
                    TCPPORT_ETH_DMA_TX_DESCRIPTOR_COUNT,  /* The number of DMA TX buffer descriptors */
                    NULL,                                 /* Nothing associated with this device */
                    TCPPORT_ETH_DMA_ISR_CORE_ID,          /* The core that will be interrupted on DMA events */
                    (rtos_irq_isr_t) xEth_ISR);           /* The ISR to handle this device's interrupts */
        }

        for (i = 0; i < TCPPORT_ETH_DMA_RX_DESCRIPTOR_COUNT; i++) {
            NetworkBufferDescriptor_t *bufdesc = pxGetNetworkBufferWithDescriptor(ETH_BUF_SIZE, 0);
            configASSERT(bufdesc != NULL);
            soc_dma_ring_rx_buf_set(soc_peripheral_rx_dma_ring_buf(xEthDev), bufdesc->pucEthernetBuffer, ETH_BUF_SIZE);
        }

    } while( 0 );

    /* Initialize any other parameters */
    if( xStatus != pdFAIL )
    {
        memcpy( ucMACAddrRTOS, FreeRTOS_GetMACAddress(), ( size_t ) ipMAC_ADDRESS_LENGTH_BYTES );
        BaseType_t xMAC_addr_init;
        uint8_t unsetMAC[ipMAC_ADDRESS_LENGTH_BYTES] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        /* If the MAC addr was set to 00:00:00:00:00:00 within FreeRTOS,
         * then we will try to use hardware MAC addr */
        xMAC_addr_init = memcmp( ucMACAddrRTOS, unsetMAC, ( size_t ) ipMAC_ADDRESS_LENGTH_BYTES );

        /* If a MAC addr was specified in FreeRTOSIPConfig.h then
         * set it on the hardware */
        if( xMAC_addr_init == 0 )
        {
            /* Get hardware MAC addr and set FreeRTOS MAC addr */
            ethernet_get_mac_addr( xEthDev, 0, ucMACAddrHW );
            FreeRTOS_UpdateMACAddress( ucMACAddrHW );
        }
        else
        {
            /* Set hardware MAC */
            ethernet_set_mac_addr( xEthDev, 0, ucMACAddrRTOS );
            memcpy( ucMACAddrHW, ucMACAddrRTOS, ( size_t ) ipMAC_ADDRESS_LENGTH_BYTES );
        }
    }

    /* Verify parameters are set up */
    if( xStatus != pdFAIL )
    {
        memcpy( ucMACAddrRTOS, FreeRTOS_GetMACAddress(), ( size_t ) ipMAC_ADDRESS_LENGTH_BYTES );
        ethernet_get_mac_addr( xEthDev, 0, ucMACAddrHW );

        BaseType_t xMACAddrAgree;

        xMACAddrAgree = memcmp( ucMACAddrRTOS, ucMACAddrHW, ( size_t ) ipMAC_ADDRESS_LENGTH_BYTES );

        if( xMACAddrAgree != 0 )
        {
            xStatus = pdFAIL;
        }
        else
        {
            debug_printf("My PHY'S MAC address is ");
            PRINT_MAC_ADDR(ucMACAddrHW);
            debug_printf("My FreeRTOS MAC address is ");
            PRINT_MAC_ADDR(ucMACAddrRTOS);
        }
    }

    /* Verify that link is present */
    if( xStatus != pdFAIL )
    {
        /* TODO: Add a timeout? */
        while( xGetPhyLinkStatus() == pdFAIL );
        debug_printf("Network is up\n");
    }

    /* Wait until network is initialized by eth_dev */
    if( xStatus == pdFAIL )
    {
        xReturn = pdFAIL;
        debug_printf("Network Init Fail\n");
    }
    else
    {
        xReturn = pdPASS;
        debug_printf("Network Init Pass\n");
    }

    return xReturn;
}

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxNetworkBuffer, BaseType_t xReleaseAfterSend )
{
    /* xReleaseAfterSend should never be false when
     * ipconfigZERO_COPY_RX_DRIVER is set to 1. */
    configASSERT( xReleaseAfterSend != pdFALSE );

    /* Send the packet */
    ethernet_driver_send_packet( TCPPORT_ETH_DEV, pxNetworkBuffer->pucEthernetBuffer, pxNetworkBuffer->xDataLength );

    /* N.B. The network buffer and its descriptor will be freed
    after the DMA has completed the transfer. */

    /* Call the standard trace macro to log the send event. */
    iptraceNETWORK_INTERFACE_TRANSMIT();
	
    #if TCPPORT_FREE_TX_FRAMES_ASAP == 0
    {
        uint8_t *pucBuf;
        soc_dma_ring_buf_t *tx_ring_buf;

        /* Ethernet buffer will be freed once it has been sent */
        pxNetworkBuffer->pucEthernetBuffer = NULL;
        vReleaseNetworkBufferAndDescriptor( pxNetworkBuffer );

        tx_ring_buf = soc_peripheral_tx_dma_ring_buf( TCPPORT_ETH_DEV );

        /* Free any tx buffers that have already been processed */
        while( ( pucBuf = soc_dma_ring_tx_buf_get( tx_ring_buf, NULL, NULL ) ) != NULL )
        {
            vReleaseNetworkBuffer( pucBuf );
        }
    }
    #endif

    return pdTRUE;
}

void vNetworkInterfaceAllocateRAMToBuffers( NetworkBufferDescriptor_t pxNetworkBuffers[ ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS ] )
{
    /* Only required if using BufferAllocation_1.c, which uses
     * preallocated static network buffers */
}

BaseType_t xGetPhyLinkStatus( void )
{
BaseType_t xReturn;

    if( ethernet_driver_smi_get_link_state( TCPPORT_ETH_DEV, TCPPORT_DEV_PHYS_ADDR ) == pdPASS )
    {
        xReturn = pdPASS;
    }
    else
    {
        xReturn = pdFAIL;
    }

    return xReturn;
}

