///**
// * File for all networking tasks
// * @author Aaron Chan
// */
//
//#include "networking.h"
//#include "device/platforms/stm32/swdebug.h"
//
//HALSPIDevice *wiz_spi = nullptr;
//HALGPIODevice *wiz_cs = nullptr;
//HALGPIODevice *wiz_rst = nullptr;
//HALGPIODevice *wizLEDGPIO = nullptr;
//LED *wiz_led = nullptr;
//
//
//RetType wiz_spi_poll_task(void *) {
//    RESUME();
//    CALL(wiz_spi->poll());
//    RESET();
//    return RET_SUCCESS;
//}
//
////RetType wizRecvTestTask(void *) {
////    RESUME();
////    static Packet packet = alloc::Packet<IPv4UDPSocket::MTU_NO_HEADERS - IPv4UDPSocket::HEADERS_SIZE, IPv4UDPSocket::HEADERS_SIZE>();
////    static uint8_t *buff;
////
////    RetType ret = CALL(w5500->recv_data(stack->get_eth(), packet));
////    buff = packet.raw();
////
////    RESET();
////    return RET_SUCCESS;
////}
//
//RetType wizSendTestTask(void *) {
//    RESUME();
//    static IPv4UDPSocket::addr_t addr;
//    addr.ip[0] = 10;
//    addr.ip[1] = 10;
//    addr.ip[2] = 10;
//    addr.ip[3] = 96;
//    addr.port = 8000;
//
//    static uint8_t buff[7] = {'L', 'a', 'u', 'n', 'c', 'h', '!'};
//    RetType ret = CALL(sock->send(buff, 7, &addr));
//
//    RESET();
//    return RET_SUCCESS;
//}
//
//RetType netStackInitTask(void *) {
//    RESUME();
//
//    static Wiznet wiznet(*wiz_spi, *wiz_cs, *wiz_rst, *wiz_led);
//    w5500 = &wiznet;
//
//    static IPv4UDPStack iPv4UdpStack{10, 10, 10, 69, \
//                              255, 255, 255, 0,
//                                     *w5500};
//    stack = &iPv4UdpStack;
//
//    static uint8_t ip_addr[4] = {192, 168, 1, 10};
//    static uint8_t subnet_mask[4] = {255, 255, 255, 0};
//    static uint8_t gateway_addr[4] = {192, 168, 1, 1};
//    static uint8_t mac_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//    static IPv4UDPSocket::addr_t addr;
//
//    sock = stack->get_socket();
//    addr.ip[0] = addr.ip[1] = addr.ip[2] = addr.ip[3] = 0;
//    addr.port = 8000;
//    sock->bind(addr); // TODO: Error handling
//
//    ipv4::IPv4Addr_t temp_addr;
//    ipv4::IPv4Address(10, 10, 10, 69, &temp_addr);
//    stack->add_multicast(temp_addr);
//
//
//    swprint("Initializing W5500\n");
//    RetType ret = CALL(wiznet.init(mac_addr));
//    if (RET_SUCCESS != ret) {
//		swprint("#RED#W5500 init failed\n");
//        goto netStackInitDone;
//    } else {
//		swprint("#GRN#W5500 init OK\n");
//    }
//
//
//    swprint("Initializing network stack\n");
//    ret = stack->init();
//    if (RET_SUCCESS != ret) {
//    	swprint("#RED#Net stack init failed");
//        goto netStackInitDone;
//    } else {
//		swprint("#GRN#Net stack init OK\n");
//    }
//
//    swprint("Successfully initialized network interface\n");
//    sched_start(wizSendTestTask, {});
//
//    netStackInitDone:
//    RESET();
//    return RET_ERROR; // Kill task
//}