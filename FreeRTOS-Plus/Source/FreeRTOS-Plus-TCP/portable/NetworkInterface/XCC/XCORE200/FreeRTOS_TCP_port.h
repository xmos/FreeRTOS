// Copyright (c) 2019, XMOS Ltd, All rights reserved

#ifndef FREERTOS_TCP_PORT_H_
#define FREERTOS_TCP_PORT_H_

#ifndef ipconfigUSE_ETHERNET
#define ipconfigUSE_ETHERNET 0
#endif

#ifndef ipconfigUSE_WIFI
#define ipconfigUSE_WIFI 0
#endif

#if !((ipconfigUSE_ETHERNET != 0) ^ (ipconfigUSE_WIFI != 0))
#define ipconfigUSE_WIFI 1 /* Only doing this for syntax highlighting */
#error Exactly one of either ipconfigUSE_ETHERNET or ipconfigUSE_WIFI must be defined true when using FreeRTOS Plus TCP
#endif

#if ipconfigUSE_ETHERNET

#include "soc.h"
#include "bitstream_devices.h"
#include "ethernet_driver.h"

#define TCPPORT_ETH_DEV                         (bitstream_ethernet_devices[BITSTREAM_ETHERNET_DEVICE_A])
#define TCPPORT_DEV_PHYS_ADDR                   (0)

#define TCPPORT_ETH_DMA_RX_DESCRIPTOR_COUNT     (12)
#define TCPPORT_ETH_DMA_TX_DESCRIPTOR_COUNT     (12)
#define TCPPORT_ETH_DMA_ISR_CORE_ID             (0)

/*
 * If this is set to 1 then transmitted network buffers and
 * their descriptors will be freed immediately after they are
 * sent by the DMA engine, rather than on the next time a frame
 * is sent.
 */
#define TCPPORT_FREE_TX_FRAMES_ASAP             (1)

#endif

#if ipconfigUSE_WIFI

#include "sl_wfx.h"
#include "sl_wfx_host.h"

#endif

#define TCPPORT_INITIAL_RANDOM_SEED             (0xF94B291A)

BaseType_t xInit_RNG( void );

#endif /* FREERTOS_TCP_PORT_H_ */
