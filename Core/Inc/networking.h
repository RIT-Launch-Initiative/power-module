/**
 * File for all networking tasks
 * @author Aaron Chan
 */

#ifndef POWER_MODULE_NETWORKING_H
#define POWER_MODULE_NETWORKING_H

#include "sched.h"
#include "init/init.h"
#include "sched/macros.h"

#include "device/platforms/stm32/HAL_SPIDevice.h"
#include "device/platforms/stm32/HAL_GPIODevice.h"
#include "device/peripherals/LED/LED.h"

#include "device/peripherals/wiznet/wiznet.h"
#include "net/packet/Packet.h"
#include "net/stack/IPv4UDP/IPv4UDPStack.h"
#include "net/stack/IPv4UDP/IPv4UDPSocket.h"

#include "networking.h"
#include "device/platforms/stm32/swdebug.h"

Wiznet *w5500 = nullptr;
HALSPIDevice* wiz_spi = nullptr;
HALGPIODevice* wiz_cs = nullptr;
HALGPIODevice *wiz_rst = nullptr;
HALGPIODevice *wiz_led_gpio = nullptr;
LED *wiz_led = nullptr;

IPv4UDPStack *stack = nullptr;
IPv4UDPSocket *sock = nullptr;



RetType wiz_spi_poll_task(void *) {
    RESUME();
    CALL(wiz_spi->poll());
    RESET();
    return RET_SUCCESS;
}

RetType wizSendTestTask(void *) {
    RESUME();
    static IPv4UDPSocket::addr_t addr;
    addr.ip[0] = 239;
    addr.ip[1] = 255;
    addr.ip[2] = 255;
    addr.ip[3] = 255;
    addr.port = 8000;

    static uint8_t buff[7] = {'L', 'a', 'u', 'n', 'c', 'h', '!'};
    RetType ret = CALL(sock->send(buff, 7, &addr));

    SLEEP(2000);

    RESET();
    return RET_SUCCESS;
}

RetType netStackInitTask(void *) {
    RESUME();
    sched_start(wiz_spi_poll_task, {});

    static uint8_t ip_addr[4] = {192, 168, 1, 10};
    static uint8_t subnet_mask[4] = {255, 255, 255, 0};
    static uint8_t gateway_addr[4] = {192, 168, 1, 1};
    static uint8_t mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static IPv4UDPSocket::addr_t addr;

    static Packet packet = alloc::Packet<IPv4UDPSocket::MTU_NO_HEADERS - IPv4UDPSocket::HEADERS_SIZE, IPv4UDPSocket::HEADERS_SIZE>();

    static Wiznet wiznet(*wiz_spi, *wiz_cs, *wiz_rst, *wiz_led_gpio, stack->get_eth(), packet);
    w5500 = &wiznet;

    static IPv4UDPStack iPv4UdpStack(10, 10, 10, 69, 255, 255, 255, 0, wiznet);
    stack = &iPv4UdpStack;

    sock = stack->get_socket();
    addr.ip[0] = addr.ip[1] = addr.ip[2] = addr.ip[3] = 0;
    addr.port = 8000;
    sock->bind(addr); // TODO: Error handling

    ipv4::IPv4Addr_t temp_addr;
    ipv4::IPv4Address(239, 255, 255, 255, &temp_addr);
    stack->add_multicast(temp_addr);

    RetType ret = CALL(wiznet.init());
    if (ret != RET_SUCCESS) {
        swprint("#RED#W5500 init failed\n");
        goto netStackInitDone;
    } else {
		swprint("#GRN#W5500 init OK\n");
    }

    if (RET_SUCCESS != stack->init()) {
        swprint("#RED#Net stack init failed");
        goto netStackInitDone;
    } else {
		swprint("#GRN#Net stack init OK\n");
    }

    swprint("Successfully initialized network interface\n");
    sched_start(wizSendTestTask, {});

    netStackInitDone:
    RESET();
    return RET_ERROR; // Kill task
}
#endif //POWER_MODULE_NETWORKING_H
