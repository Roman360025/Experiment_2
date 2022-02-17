#include <stdio.h>
#include <string.h>

#include "periph/gpio.h"
#include "lptimer.h"
#include "thread.h"

#include "net/lora.h"
#include "net/netdev/lora.h"
#include "sx127x_internal.h"
#include "sx127x_netdev.h"
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/* For UNWD-RANGE board */
#ifndef SX127X_PARAM_PASELECT
#define SX127X_PARAM_PASELECT   (SX127X_PA_RFO)
#endif

#define SX127X_LORA_MSG_QUEUE   (16U)
#define SX127X_STACKSIZE        (2*THREAD_STACKSIZE_DEFAULT)

int power = -1;
int sf = 7;


typedef struct {
    bool (*appdata_received_cb)(uint8_t *buf, size_t buflen);

    netdev_t *device;       /**< Pointer to the radio PHY structure */
    gpio_t pps_pin;
    
    char isr_stack[SX127X_STACKSIZE];
    kernel_pid_t isr_pid;
    
    char slot_stack[SX127X_STACKSIZE];
    kernel_pid_t slot_pid;
    

    
    lptimer_t slot_timer;
    /* TODO: add sender control structures (received IDs, own ID, etc...) */



} sender_t;


#define MSG_TYPE_ISR            (0x3456)

static sx127x_params_t sx127x_params = {
    .nss_pin = SX127X_SPI_NSS,
    .spi = SX127X_SPI,

    .dio0_pin = SX127X_DIO0,
    .dio1_pin = SX127X_DIO1,
    .dio2_pin = SX127X_DIO2,
    .dio3_pin = SX127X_DIO3,
    .dio4_pin = SX127X_DIO4,
    .dio5_pin = SX127X_DIO5,
    .reset_pin = SX127X_RESET,
   
    .rfswitch_pin = SX127X_RFSWITCH,
    .rfswitch_active_level = 1,
    .paselect = SX127X_PARAM_PASELECT
};
static sx127x_t sx127x;


void *isr_thread(void *arg){
    (void)arg;

    static msg_t _msg_q[SX127X_LORA_MSG_QUEUE];
    msg_init_queue(_msg_q, SX127X_LORA_MSG_QUEUE);

    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == MSG_TYPE_ISR) {
            netdev_t *dev = msg.content.ptr;
            dev->driver->isr(dev);
        }
        else {
//            puts("[LoRa] unexpected msg type");
        }
    }
}



static void sx127x_handler(netdev_t *dev, netdev_event_t event, void *arg)
{
    sender_t *sender = (sender_t *)arg;
    
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;
        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;
        if (msg_send(&msg, sender->isr_pid) <= 0) {
//            puts("gnrc_netdev: possibly lost interrupt.");
        }
        return;
    }
    
    switch (event) {
        case NETDEV_EVENT_RX_COMPLETE: {
            int len;
            netdev_lora_rx_info_t packet_info;
            uint8_t message[255];
    
            len = sender->device->driver->recv(dev, NULL, 0, &packet_info);
            if (len < 0) {
//                printf("RX: bad message, aborting\n");
                break;
            }
            
            sender->device->driver->recv(dev, message, len, &packet_info);
//            printf("RX: %d bytes, | RSSI: %d dBm | SNR: %d dB\n", (int)len, packet_info.rssi, (int)packet_info.snr);
            /* TODO: здесь вызываем обработчик пакета (точнее, кидаем сообщение в другой поток?) */
            break;
        }
        case NETDEV_EVENT_CRC_ERROR:
//            puts("[LoRa] RX CRC failed");
            break;

        case NETDEV_EVENT_TX_COMPLETE:
            printf("[LoRa] transmission done.\r\n");
            
            /* TODO: переключаемся в режим приема правильно */
            uint8_t state = NETOPT_STATE_IDLE;
            sender->device->driver->set(dev, NETOPT_STATE, &state, sizeof(uint8_t));
            
            break;

        case NETDEV_EVENT_RX_TIMEOUT:
//            puts("[LoRa] RX timeout");
            break;

        case NETDEV_EVENT_TX_TIMEOUT:
            /* this should not happen, re-init SX127X here */
//            puts("[LoRa] TX timeout");
            sender->device->driver->init(dev);
            break;
            
        case NETDEV_EVENT_CAD_DONE:
//            puts("[LoRa] CAD done\n");
            break;
            
        case NETDEV_EVENT_CAD_DETECTED:
//            puts("[LoRa] CAD detected\n");
            break;
            
        case NETDEV_EVENT_VALID_HEADER:
//            puts("[LoRa] header received, switch to RX state");
            break;

        default:
//            printf("[LoRa] received event #%d\n", (int) event);
            break;
    }
}




void do_send(sender_t *sender, int *power, int *sf)
{
    int8_t buffer[10];

    buffer[0] = 15;
    buffer[1] = 112;

    if (*power != 15) {
        for (int i = 2; i < 10; ++i)
        {
            buffer[i] = *power;
        }
    }
    else {
        buffer[2] = 15;
        for (int i = 3; i < 10; ++i)
        {
            buffer[i] = *sf;
        }
    }



    iolist_t data = {
    .iol_base = buffer,
    .iol_len = 10,
    };

    if (sender->device->driver->send(sender->device, &data) < 0) {
//        puts("[LoRa] cannot send, device busy");
    }
    else
    {
    }
}


void *slot_thread(void *arg){
       sender_t *sender = (sender_t *)arg;

    while (1) {

        sender->device->driver->set(sender->device, NETOPT_TX_POWER, &power, sizeof(int16_t));



        for (int i = 0; i < 10; ++i)
        {
            lptimer_sleep(5000);
            gpio_set(UNWD_GPIO_1);
            gpio_set(LED0_PIN);
            lptimer_sleep(10);
            gpio_clear(UNWD_GPIO_1);
            gpio_clear(LED0_PIN);
            lptimer_sleep(90);
            gpio_toggle(LED0_PIN);
            do_send(sender, &power, &sf);
        }

        power++;


        if (power == 15) {
            if (sf == 12) {
                printf("DDDne\r\n");
                lptimer_sleep(500);
                printf("DDDone\r\n");
                sf = 7;
                }
            else {
                sf++;
            }

            power = -1;
            printf("PPPower %d\r\n", power);
            lptimer_sleep(500);
            printf("PPPower %d\r\n", power);

            power = 15;
            do_send(sender, &power, &sf);
            lptimer_sleep(5000);
            do_send(sender, &power, &sf);
            lptimer_sleep(5000);
            do_send(sender, &power, &sf);
            lptimer_sleep(5000);

            power = -1;


            switch (sf) {
            case 7:
                sf = LORA_SF7;
                break;
            case 8:
                sf = LORA_SF8;
                break;
            case 9:
                sf = LORA_SF9;
                break;
            case 10:
                sf = LORA_SF10;
                break;
            case 11:
                sf = LORA_SF11;
                break;
            case 12:
                sf = LORA_SF12;
                break;
                }

        sender->device->driver->set(sender->device, NETOPT_SPREADING_FACTOR, &sf, sizeof(uint8_t));



        }
        else {
            printf("PPPower %d\r\n", power);
            lptimer_sleep(500);
            printf("PPPower %d\r\n", power);
            }
        }
    }

int sender_init(sender_t *sender, netdev_t *device){
    device->driver = &sx127x_driver;
    sender->device = device;

    sender->isr_pid = thread_create(sender->isr_stack, sizeof(sender->isr_stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, isr_thread, sender,
                              "SX127x handler thread");

    if (sender->isr_pid <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of SX127X ISR thread failed");
        return false;
    }

    sender->slot_pid = thread_create(sender->slot_stack, sizeof(sender->slot_stack), THREAD_PRIORITY_MAIN - 2,
                              THREAD_CREATE_STACKTEST, slot_thread, sender,
                              "sender uplink thread");

    if (sender->isr_pid <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of SX127X ISR thread failed");
        return false;
    }

    if(sender->device->driver->init(sender->device) < 0){
        puts("Netdev driver initialisation failed");
    }

    sender->device->event_callback = sx127x_handler;
    sender->device->event_callback_arg = sender;

    /* Конфигурируем трансивер, выставляем значимые параметры */
    const netopt_enable_t enable = true;
    const netopt_enable_t disable = false;

    puts("[LoRa] reconfigure transceiver\n");
    /* Configure to sleep */
    uint8_t state = NETOPT_STATE_SLEEP;
    sender->device->driver->set(sender->device, NETOPT_STATE, &state, sizeof(uint8_t));

    uint16_t modem = NETDEV_TYPE_LORA;
    sender->device->driver->set(sender->device, NETOPT_DEVICE_TYPE, &modem, sizeof(uint16_t));

    uint8_t sf_init = LORA_SF7;
    sender->device->driver->set(sender->device, NETOPT_SPREADING_FACTOR, &sf_init, sizeof(uint8_t));
    uint8_t bw = LORA_BW_125_KHZ;
    sender->device->driver->set(sender->device, NETOPT_BANDWIDTH, &bw, sizeof(uint8_t));
    uint8_t cr = LORA_CR_4_5;
    sender->device->driver->set(sender->device, NETOPT_CODING_RATE, &cr, sizeof(uint8_t));

    uint8_t hop_period = 0;
    sender->device->driver->set(sender->device, NETOPT_CHANNEL_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    sender->device->driver->set(sender->device, NETOPT_CHANNEL_HOP, &disable, sizeof(disable));
    sender->device->driver->set(sender->device, NETOPT_SINGLE_RECEIVE, &disable, sizeof(disable));
    sender->device->driver->set(sender->device, NETOPT_INTEGRITY_CHECK, &disable, sizeof(enable));
    sender->device->driver->set(sender->device, NETOPT_FIXED_HEADER, &disable, sizeof(disable));
    sender->device->driver->set(sender->device, NETOPT_IQ_INVERT, &disable, sizeof(disable));

    int16_t power_init = -1;
    sender->device->driver->set(sender->device, NETOPT_TX_POWER, &power_init, sizeof(int16_t));

    uint16_t preamble_len = 8;
    sender->device->driver->set(sender->device, NETOPT_PREAMBLE_LENGTH, &preamble_len, sizeof(uint16_t));

    uint32_t tx_timeout = 30000;
    sender->device->driver->set(sender->device, NETOPT_TX_TIMEOUT, &tx_timeout, sizeof(uint32_t));

    uint32_t rx_timeout = 0;
    sender->device->driver->set(sender->device, NETOPT_RX_TIMEOUT, &rx_timeout, sizeof(uint32_t));

    uint32_t frequency = 869000000;
    sender->device->driver->set(sender->device, NETOPT_CHANNEL_FREQUENCY, &frequency, sizeof(uint32_t));

    state = NETOPT_STATE_IDLE;
    sender->device->driver->set(sender->device, NETOPT_STATE, &state, sizeof(uint8_t));


    return 0;
}

sender_t sender;

int main(void){
    sx127x.params = sx127x_params;
    sender_init(&sender, (netdev_t*) &sx127x);

    gpio_init(UNWD_GPIO_1, GPIO_OUT);
    gpio_clear(UNWD_GPIO_1);
    gpio_clear(LED0_PIN);

    while (1)
    {
        scanf("%d", &power);
        scanf("%d", &sf);

        puts("");
        printf("Power: %d\n", power);
        puts("");

        sender.device->driver->set(sender.device, NETOPT_TX_POWER, &power, sizeof(int16_t));

        puts("");
        printf("SF: %d\n", sf);
        puts("");

        switch (sf)
            {
            case 7:
                sf = LORA_SF7;
                break;
            case 8:
                sf = LORA_SF8;
                break;
            case 9:
                sf = LORA_SF9;
                break;
            case 10:
                sf = LORA_SF10;
                break;
            case 11:
                sf = LORA_SF11;
                break;
            case 12:
                sf = LORA_SF12;
                break;
            }

        sender.device->driver->set(sender.device, NETOPT_SPREADING_FACTOR, &sf, sizeof(uint8_t));
    }
    
    }
 