#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include <esp_log.h>
#include "mbedtls/md.h"

#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

#include "lora.h"
#include "lora_crc.h"
#include "openlora.h"

#include "esp_random.h"
#include "backoff_algorithm.h"


static net_if_buffer_descriptor_t net_if_buffer_descriptors[NUMBER_OF_NET_IF_DESCRIPTORS];
static openlora_t openlora;
static const char *OPEN_LORA_TAG = "lora_tx";

void ol_init_net_if_buffers(void){
    for (int i=0; i<NUMBER_OF_NET_IF_DESCRIPTORS; i++) {
        net_if_buffer_descriptors[i].puc_link_buffer = NULL;
        net_if_buffer_descriptors[i].data_lenght = 0;
    }
}

void ol_link_layer_task(void *arg);

void ol_init(uint8_t nwk_id, uint8_t addr) {
    ol_init_net_if_buffers();

    openlora.nwk_id = nwk_id;
    openlora.node_addr = addr;
    openlora.mac_seq_number = 0;
    memset(openlora.neigh_seq_number, 0xFF, 256);

    xTaskCreate(ol_link_layer_task, "OL Link Layer Task", 2048, NULL, 4, NULL);
}

net_if_buffer_descriptor_t *ol_get_net_if_buffer(uint8_t size){
    for (int i=0; i<NUMBER_OF_NET_IF_DESCRIPTORS; i++) {
        if (net_if_buffer_descriptors[i].puc_link_buffer == NULL) {
            //if (size <= LORA_MAX_PAYLOAD_SIZE) {
                net_if_buffer_descriptors[i].puc_link_buffer = pvPortMalloc(size);
                net_if_buffer_descriptors[i].data_lenght = size;
                return &net_if_buffer_descriptors[i];
            //}
        }
    }
    return NULL;
}

void ol_release_net_if_buffer(net_if_buffer_descriptor_t *buffer){
    //taskENTER_CRITICAL();
    buffer->data_lenght = 0;
    vPortFree(buffer->puc_link_buffer);
    buffer->puc_link_buffer = NULL;
    //taskEXIT_CRITICAL();
}


uint32_t ol_send_link_frame(uint8_t dst_addr, net_if_buffer_descriptor_t *net_if_buffer, uint32_t timeout){
    link_layer_header_t *link_frame = (link_layer_header_t *)net_if_buffer->puc_link_buffer;

    link_frame->frame_type = DATA_FRAME;
    link_frame->network_id = openlora.nwk_id;
    link_frame->seq_number = openlora.mac_seq_number;
    link_frame->dst_addr   = dst_addr;
    link_frame->src_addr   = openlora.node_addr;
    link_frame->payload_size = net_if_buffer->data_lenght - sizeof(link_layer_header_t) - sizeof(link_layer_trailer_t);

    link_layer_trailer_t *link_trailer = (link_layer_trailer_t *)&net_if_buffer->puc_link_buffer[sizeof(link_layer_header_t)+link_frame->payload_size];

    link_trailer->crc = usLORACRC16(net_if_buffer->puc_link_buffer, sizeof(link_layer_header_t) + link_frame->payload_size);

    uint32_t link_retries = 0;
    do {
        /* Variables used for the backoff algorithm */
        //BackoffAlgorithmStatus_t retryStatus = BackoffAlgorithmSuccess;
        BackoffAlgorithmContext_t retryParams;
        uint16_t nextRetryBackoff = 0;

        /* Initialize reconnect attempts and interval. */
        BackoffAlgorithm_InitializeParams( &retryParams,
                                        RETRY_BACKOFF_BASE_MS,
                                        RETRY_MAX_BACKOFF_DELAY_MS,
                                        BACKOFF_RETRY_MAX_ATTEMPTS );

        //srand(xTaskGetTickCount());

        uint32_t backoff_retries = 0;
        do {
            if (BackoffAlgorithm_GetNextBackoff( &retryParams, esp_random(), &nextRetryBackoff ) == BackoffAlgorithmSuccess) {
                ESP_LOGI(OPEN_LORA_TAG, "Backoff Time %d", nextRetryBackoff);
                vTaskDelay(nextRetryBackoff);

                if (lora_cca() == pdTRUE) {
                    // wait for channel availability
                    break;
                }
                backoff_retries++;
            }else {
                // BackoffAlgorithmRetriesExhausted
                ESP_LOGI(OPEN_LORA_TAG, "Fail to get a backoff time!");
                return pdFALSE;
            }
        }while(backoff_retries < LINK_RETRIES);

        if (backoff_retries >= LINK_RETRIES) {
            // couldn´t get the channel
            ESP_LOGI(OPEN_LORA_TAG, "Fail to get channel free to transmit!");
            return pdFALSE;
        }

        ESP_LOGI(OPEN_LORA_TAG, "Transmitting an ol frame: %s", net_if_buffer->puc_link_buffer);
        uint32_t ret = lora_send_frame(net_if_buffer->puc_link_buffer, net_if_buffer->data_lenght, timeout);
        if (ret == pdTRUE) {
            // wait for the ack
            ESP_LOGI(OPEN_LORA_TAG, "Waiting ACK");
            lora_receive();    // put into receive mode
            if (lora_received(LINK_ACK_TIMEOUT) == pdTRUE) {
                int len = lora_read_frame_size();
                if (len >= (sizeof(link_layer_header_t) + sizeof(link_layer_trailer_t))) {

                    net_if_buffer_descriptor_t *net_if_ack_buffer = ol_get_net_if_buffer((uint8_t)len);
                    int x = lora_read_frame(net_if_ack_buffer->puc_link_buffer, net_if_ack_buffer->data_lenght);
                    if (x){
                        uint16_t crc = usLORACRC16(net_if_ack_buffer->puc_link_buffer, (net_if_ack_buffer->data_lenght - sizeof(link_layer_trailer_t)));
                        //ESP_LOGI(OPEN_LORA_TAG, "calc ack: %x - packet acc: %x", crc, ack_packet.mac_ack_packet.crc);
                        link_layer_header_t *link_ack_frame = (link_layer_header_t *)net_if_ack_buffer->puc_link_buffer;
                        link_layer_trailer_t *link_ack_trailer = (link_layer_trailer_t *)&net_if_ack_buffer->puc_link_buffer[sizeof(link_layer_header_t)];
                        if (link_ack_trailer->crc == crc) {
                            if ((link_ack_frame->frame_type == ACK_FRAME) && (link_ack_frame->dst_addr == openlora.node_addr)){
                                if ((link_ack_frame->seq_number == link_frame->seq_number) && (link_ack_frame->network_id == openlora.nwk_id)){
                                    openlora.mac_seq_number++;
                                    if (openlora.mac_seq_number >= 0xFE) {
                                        openlora.mac_seq_number = 0;
                                    }
                                    ol_release_net_if_buffer(net_if_ack_buffer);
                                    return pdTRUE;
                                }
                            }
                        }else {
                            ESP_LOGI(OPEN_LORA_TAG, "ACK packet with wrong CRC");
                        }
                    }
                    ol_release_net_if_buffer(net_if_ack_buffer);
                }
            }else {
                ESP_LOGI(OPEN_LORA_TAG, "ACK packet timeout!");
            }
        }
        link_retries++;
    }while(link_retries < LINK_RETRIES);

    openlora.mac_seq_number++;
    if (openlora.mac_seq_number >= 0xFE) {
        openlora.mac_seq_number = 0;
    }

    return pdFALSE;
}

uint32_t ol_send_link_ack(net_if_buffer_descriptor_t *net_if_buffer, uint32_t timeout) {
    net_if_buffer_descriptor_t *net_if_ack_buffer = ol_get_net_if_buffer(sizeof(link_layer_header_t)+sizeof(link_layer_trailer_t));

    link_layer_header_t *link_frame = (link_layer_header_t *)net_if_buffer->puc_link_buffer;
    link_layer_header_t *link_ack_packet = (link_layer_header_t *)net_if_ack_buffer->puc_link_buffer;

    link_ack_packet->frame_type = ACK_FRAME;
    link_ack_packet->network_id = link_frame->network_id;
    link_ack_packet->dst_addr = link_frame->src_addr;
    link_ack_packet->src_addr = openlora.node_addr;
    link_ack_packet->seq_number = link_frame->seq_number;
    link_ack_packet->payload_size = 0;
    link_layer_trailer_t *link_ack_trailer = (link_layer_trailer_t *)&net_if_ack_buffer->puc_link_buffer[sizeof(link_layer_header_t)];
    link_ack_trailer->crc = usLORACRC16(net_if_ack_buffer->puc_link_buffer, sizeof(link_layer_header_t));

    uint32_t ret = lora_send_frame(net_if_ack_buffer->puc_link_buffer, net_if_ack_buffer->data_lenght, timeout);
    ol_release_net_if_buffer(net_if_ack_buffer);
    return ret;
}

/* Define the size of the item to be held by queue 1 and queue 2 respectively.
The values used here are just for demonstration purposes. */
#define LINK_LAYER_QUEUE    16

/* Binary semaphores have an effective length of 1. */
#define BINARY_SEMAPHORE_LENGTH	1

/* The combined length of the two queues and binary semaphore that will be
added to the queue set. */
#define COMBINED_LENGTH ( LINK_LAYER_QUEUE + BINARY_SEMAPHORE_LENGTH )

static QueueHandle_t tx_link_layer_queue;
static QueueHandle_t rx_link_layer_queue;

BaseType_t ol_to_link_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout) {
    return xQueueSendToBack(tx_link_layer_queue, &buffer, timeout);
}

BaseType_t ol_from_link_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout) {
    return xQueueReceive(rx_link_layer_queue, buffer, timeout);
}

void ol_link_layer_task(void *arg) {
    // esperar pacotes das camadas superiores
    // chegou um pacote do radio
    // transmitir um pacote (cca, csma/ca, retentativas, ack, ...)
    static const char *TAG = "ol_link_layer";

    /* Create the queue set large enough to hold an event for every space in
    every queue and semaphore that is to be added to the set. */
    static QueueSetHandle_t link_event;
    QueueSetMemberHandle_t xActivatedMember;

    link_event = xQueueCreateSet( COMBINED_LENGTH );

    /* Create the queues and semaphores that will be contained in the set. */
    tx_link_layer_queue = xQueueCreate( LINK_LAYER_QUEUE, sizeof(net_if_buffer_descriptor_t *));
    rx_link_layer_queue = xQueueCreate( LINK_LAYER_QUEUE, sizeof(net_if_buffer_descriptor_t *));

    /* Check everything was created. */
    configASSERT( link_event );
    configASSERT( tx_link_layer_queue );
    configASSERT( rx_link_layer_queue );

    /* Add the queues and semaphores to the set.  Reading from these queues and
    semaphore can only be performed after a call to xQueueSelectFromSet() has
    returned the queue or semaphore handle from this point on. */
    xQueueAddToSet( tx_link_layer_queue, link_event );
    SemaphoreHandle_t sem_radio = lora_get_received_sem();
    xQueueAddToSet( sem_radio, link_event );

    while(1) {
        /* Enter in reception mode */
        lora_receive();
        /* Block to wait for something to be available from the queues or
        semaphore that have been added to the set. */
        xActivatedMember = xQueueSelectFromSet( link_event, portMAX_DELAY);

        if( xActivatedMember == tx_link_layer_queue ){
            // Transmitt a packet
            net_if_buffer_descriptor_t *frame;
            xQueueReceive( xActivatedMember, &frame, portMAX_DELAY);

            (void)ol_send_link_frame(OL_BORDER_ROUTER_ADDR, frame, portMAX_DELAY);
            ol_release_net_if_buffer(frame);

        }else if( xActivatedMember == sem_radio ){
            // Receive a packet
            if (lora_received(10) == pdTRUE) {
                int len = lora_read_frame_size();
                net_if_buffer_descriptor_t *net_if_buffer = ol_get_net_if_buffer((uint8_t)len);
                int x = lora_read_frame(net_if_buffer->puc_link_buffer, net_if_buffer->data_lenght);
                if (x) {
                    link_layer_header_t *link_frame_header = (link_layer_header_t *)net_if_buffer->puc_link_buffer;
                    link_layer_trailer_t *link_frame_trailer = (link_layer_trailer_t *)&net_if_buffer->puc_link_buffer[sizeof(link_layer_header_t)+link_frame_header->payload_size];
                    uint16_t crc = usLORACRC16(net_if_buffer->puc_link_buffer, (net_if_buffer->data_lenght - sizeof(link_layer_trailer_t)));
                    if (link_frame_trailer->crc == crc) {
                        if ((link_frame_header->frame_type == DATA_FRAME) && (link_frame_header->network_id == openlora.nwk_id) && (link_frame_header->dst_addr == openlora.node_addr)) {
                            (void)ol_send_link_ack(net_if_buffer, LINK_ACK_TIMEOUT);
                            if (openlora.neigh_seq_number[link_frame_header->src_addr] != link_frame_header->seq_number) {
                                openlora.neigh_seq_number[link_frame_header->src_addr] = link_frame_header->seq_number;
                                if (xQueueSendToBack(rx_link_layer_queue, &net_if_buffer, 100) == pdTRUE) {
                                    ESP_LOGI(OPEN_LORA_TAG, "link frame sent to the upper layer");
                                }else {
                                    ESP_LOGI(OPEN_LORA_TAG, "full upper layer queue");
                                    ol_release_net_if_buffer(net_if_buffer);
                                }
                            }
                        }else {
                            ESP_LOGI(OPEN_LORA_TAG, "not the link frame destination");
                            ol_release_net_if_buffer(net_if_buffer);
                        }
                    }else {
                        ol_release_net_if_buffer(net_if_buffer);
                        ESP_LOGI(OPEN_LORA_TAG, "link frame with wrong CRC");
                    }
                }else {
                    ol_release_net_if_buffer(net_if_buffer);
                    ESP_LOGI(OPEN_LORA_TAG, "zero size link frame");
                }
            }
        }

    }
}

