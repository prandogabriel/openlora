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

static net_if_buffer_descriptor_t net_if_buffer_descriptors[NUMBER_OF_NET_IF_DESCRIPTORS];
static openlora_t openlora;


void ol_init_net_if_buffers(void){
    for (int i=0; i<NUMBER_OF_NET_IF_DESCRIPTORS; i++) {
        net_if_buffer_descriptors[i].puc_link_buffer = NULL;
        net_if_buffer_descriptors[i].data_lenght = 0;
    }
}


void ol_init(uint8_t nwk_id, uint8_t addr) {
    ol_init_net_if_buffers();

    openlora.nwk_id = nwk_id;
    openlora.node_addr = addr;
    openlora.mac_seq_number = 0;
    memset(openlora.neigh_seq_number, 0xFF, 256);
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
    //taskEXIT_CRITICAL();
}



uint32_t ol_send_link_packet(uint8_t dst_addr, net_if_buffer_descriptor_t *net_if_buffer, uint32_t timeout){
    //QueueSetMemberHandle_t teste = xQueueSelectFromSet(link_event, 0);
    return 0;
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


void link_layer(void *arg) {
    // esperar pacotes das camadas superiores
    // chegou um pacote do radio
    // transmitir um pacote (cca, csma/ca, retentativas, ack, ...)

    /* Create the queue set large enough to hold an event for every space in
    every queue and semaphore that is to be added to the set. */
    static QueueSetHandle_t link_event;
    QueueSetMemberHandle_t xActivatedMember;

    link_event = xQueueCreateSet( COMBINED_LENGTH );

    /* Create the queues and semaphores that will be contained in the set. */
    tx_link_layer_queue = xQueueCreate( LINK_LAYER_QUEUE, sizeof(net_if_buffer_descriptor_t));

    /* Check everything was created. */
    configASSERT( link_event );
    configASSERT( tx_link_layer_queue );

    /* Add the queues and semaphores to the set.  Reading from these queues and
    semaphore can only be performed after a call to xQueueSelectFromSet() has
    returned the queue or semaphore handle from this point on. */
    xQueueAddToSet( tx_link_layer_queue, link_event );
    SemaphoreHandle_t sem_radio = lora_get_received_sem();
    xQueueAddToSet( sem_radio, link_event );


    while(1) {
        /* Block to wait for something to be available from the queues or
        semaphore that have been added to the set.  Don't block longer than
        200ms. */
        xActivatedMember = xQueueSelectFromSet( link_event, portMAX_DELAY);

        if( xActivatedMember == tx_link_layer_queue ){
            // Transmitt a packet

        }else if( xActivatedMember == sem_radio ){
            // Receive a packet
        }

    }
}

