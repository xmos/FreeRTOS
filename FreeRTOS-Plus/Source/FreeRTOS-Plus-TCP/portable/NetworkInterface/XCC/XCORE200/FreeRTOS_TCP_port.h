// Copyright (c) 2019, XMOS Ltd, All rights reserved

#ifndef FREERTOS_TCP_PORT_H_
#define FREERTOS_TCP_PORT_H_

#include "soc.h"
#include "bitstream_devices.h"
#include "ethernet_driver.h"

#define TCPPORT_ETH_DEV                         (bitstream_ethernet_devices[BITSTREAM_ETHERNET_DEVICE_A])
#define TCPPORT_DEV_PHYS_ADDR                   (0)

#define TCPPORT_INITIAL_RANDOM_SEED             (0xF94B291A)

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

BaseType_t xInit_RNG( void );

#endif /* FREERTOS_TCP_PORT_H_ */
