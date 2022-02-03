#include <stdio.h>
#include <string.h>

#include "lptimer.h"
#include "thread.h"

#include "net/lora.h"
#include "net/netdev/lora.h"
#include "sx127x_internal.h"
#include "sx127x_netdev.h"

/* For UNWD-RANGE board */
#ifndef SX127X_PARAM_PASELECT
#define SX127X_PARAM_PASELECT   (SX127X_PA_RFO)
#endif

#define SX127X_LORA_MSG_QUEUE   (16U)
#define SX127X_STACKSIZE        (2*THREAD_STACKSIZE_DEFAULT)
#define MSG_TYPE_ISR            (0x3456)
static char isr_stack[SX127X_STACKSIZE];
static kernel_pid_t isr_pid;

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
volatile int sf = 7;
volatile int count_of_message = 0;
volatile int power = 0;
volatile bool flag_sf = false;

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
            puts("[LoRa] unexpected msg type");
        }
    }
}

static void sx127x_handler(netdev_t *dev, netdev_event_t event, void *arg)
{
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;
        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;
        if (msg_send(&msg, isr_pid) <= 0) {
            puts("gnrc_netdev: possibly lost interrupt.");
        }
        return;
    }
    
    //ls_ed_t *ls = (ls_ed_t *)arg;
    (void)arg;
    
    switch (event) {
        case NETDEV_EVENT_RX_COMPLETE: {

            int len;
            netdev_lora_rx_info_t packet_info;
            int8_t message[255];
    
            len = dev->driver->recv(dev, NULL, 0, &packet_info);
            if (len < 0) {
                printf("RX: bad message, aborting\r\n");
                break;
            }
            else {
                dev->driver->recv(dev, message, len, &packet_info);

                if (message[0] == 15 && message[1] == 112 && len == 10)
                {
                    if (power != message[2] && message[2] != 15) {
                    printf("\r\n");
                    printf("Power: %d\r\n", message[2]);
                    power = message[2];
                    count_of_message = 0;
                }

                printf("Number: %d | RSSI: %d dBm | SNR: %d dB\r\n", ++count_of_message, packet_info.rssi, (int)packet_info.snr);

                if (message[2] == 15)
                {

                    flag_sf = true;

                }

                }

            }
            

            
            // dev->driver->recv(dev, message, len, &packet_info);
            // printf("RX: %d bytes, | RSSI: %d dBm | SNR: %d dB\n", (int)len, packet_info.rssi, (int)packet_info.snr);
            
            break;
        }
        case NETDEV_EVENT_CRC_ERROR:
            //puts("[LoRa] RX CRC failed");
            break;

        case NETDEV_EVENT_TX_COMPLETE:
            puts("[LoRa] transmission done.");
            
            /* TODO: переключаемся в режим приема правильно */
            uint8_t state = NETOPT_STATE_IDLE;
            dev->driver->set(dev, NETOPT_STATE, &state, sizeof(uint8_t));
            
            break;

        case NETDEV_EVENT_RX_TIMEOUT:
            puts("[LoRa] RX timeout");
            break;

        case NETDEV_EVENT_TX_TIMEOUT:
            /* this should not happen, re-init SX127X here */
            //puts("[LoRa] TX timeout");
            dev->driver->init(dev);
            break;
            
        case NETDEV_EVENT_CAD_DONE:
            //puts("[LoRa] CAD done\n");
            break;
            
        case NETDEV_EVENT_CAD_DETECTED:
            //puts("[LoRa] CAD detected\n");
            break;
            
        case NETDEV_EVENT_VALID_HEADER:
            //puts("[LoRa] header received, switch to RX state");
            break;

        default:
            //printf("[LoRa] received event #%d\n", (int) event);
            break;
    }
}

int main(void){
    /* Создаем netdev со всей его "обвязкой" */
    sx127x.params = sx127x_params;
    
    netdev_t *device = (netdev_t*) &sx127x;
    device->driver = &sx127x_driver;
    
    isr_pid = thread_create(isr_stack, sizeof(isr_stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, isr_thread, NULL,
                              "SX127x handler thread");

    if (isr_pid <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of SX127X ISR thread failed");
        return false;
    }
    
    if(device->driver->init(device) < 0){
        puts("Netdev driver initialisation failed");
    }
    
    device->event_callback = sx127x_handler;
    device->event_callback_arg = NULL; /* На самом деле можно передать указатель на какую-нибудь служебную структуру */
    
    /* Конфигурируем трансивер, выставляем значимые параметры */
    const netopt_enable_t enable = true;
    const netopt_enable_t disable = false;
    
    puts("[LoRa] reconfigure transceiver\n");
    /* Configure to sleep */
    uint8_t state = NETOPT_STATE_SLEEP;
    device->driver->set(device, NETOPT_STATE, &state, sizeof(uint8_t));
    
    uint16_t modem = NETDEV_TYPE_LORA;
    device->driver->set(device, NETOPT_DEVICE_TYPE, &modem, sizeof(uint16_t));

    uint8_t sf = LORA_SF7;
    device->driver->set(device, NETOPT_SPREADING_FACTOR, &sf, sizeof(uint8_t));
    uint8_t bw = LORA_BW_125_KHZ;
    device->driver->set(device, NETOPT_BANDWIDTH, &bw, sizeof(uint8_t));
    uint8_t cr = LORA_CR_4_5;
    device->driver->set(device, NETOPT_CODING_RATE, &cr, sizeof(uint8_t));
    
    uint8_t hop_period = 0;
    device->driver->set(device, NETOPT_CHANNEL_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    device->driver->set(device, NETOPT_CHANNEL_HOP, &disable, sizeof(disable));
    device->driver->set(device, NETOPT_SINGLE_RECEIVE, &disable, sizeof(disable));
    device->driver->set(device, NETOPT_INTEGRITY_CHECK, &disable, sizeof(enable));
    device->driver->set(device, NETOPT_FIXED_HEADER, &disable, sizeof(disable));
    device->driver->set(device, NETOPT_IQ_INVERT, &disable, sizeof(disable));
    
    int16_t power = 20;

    device->driver->set(device, NETOPT_TX_POWER, &power, sizeof(int16_t));
    
    uint16_t preamble_len = 8;
    device->driver->set(device, NETOPT_PREAMBLE_LENGTH, &preamble_len, sizeof(uint16_t));
    
    uint32_t tx_timeout = 30000;
    device->driver->set(device, NETOPT_TX_TIMEOUT, &tx_timeout, sizeof(uint32_t));
    
    uint32_t rx_timeout = 0;
    device->driver->set(device, NETOPT_RX_TIMEOUT, &rx_timeout, sizeof(uint32_t));

    uint32_t frequency = 869000000;
    device->driver->set(device, NETOPT_CHANNEL_FREQUENCY, &frequency, sizeof(uint32_t));
    
    state = NETOPT_STATE_IDLE;
    device->driver->set(device, NETOPT_STATE, &state, sizeof(uint8_t));

    while(1){

//        puts("Power and SF:");

//        scanf("%d", &sf);

        if (flag_sf) {
            sf++;

            puts("");
            printf("SF: %d\n", sf);
            puts("");

            switch (sf) {
            case 0:
                count_of_message = 0;
                break;
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



        flag_sf = false;
        device->driver->set(device, NETOPT_SPREADING_FACTOR, &sf, sizeof(uint8_t));
        }



    }
    
}
