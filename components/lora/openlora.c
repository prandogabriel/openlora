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
// #include <openssl/sha.h>

static net_if_buffer_descriptor_t net_if_buffer_descriptors[NUMBER_OF_NET_IF_DESCRIPTORS];
static openlora_t openlora;
static transport_layer_t *transp_list_head = NULL;
static transport_layer_t *transp_list_tail = NULL;
static const char *OPEN_LORA_TAG = "lora_tx";

/* Define the size of the item to be held by queue 1 and queue 2 respectively.
The values used here are just for demonstration purposes. */
#define LINK_LAYER_QUEUE_LENGTH 16

/* Binary semaphores have an effective length of 1. */
#define BINARY_SEMAPHORE_LENGTH 1

#define TRANSP_LAYER_QUEUE_TX_LENGTH 16
#define TRANSP_LAYER_QUEUE_RX_LENGTH 16

/* The combined length of the two queues and binary semaphore that will be
added to the queue set. */
#define COMBINED_LENGTH_LINK (LINK_LAYER_QUEUE_LENGTH + BINARY_SEMAPHORE_LENGTH)
#define COMBINED_LENGTH_TRANSP (TRANSP_LAYER_QUEUE_TX_LENGTH + LINK_LAYER_QUEUE_LENGTH + BINARY_SEMAPHORE_LENGTH)

static SemaphoreHandle_t ol_available_network_buffer = NULL;
static SemaphoreHandle_t ol_network_buffer_mutex = NULL;

static QueueHandle_t tx_link_layer_queue;
static QueueHandle_t rx_link_layer_queue;

static SemaphoreHandle_t link_layer_tx_ready_signal;

static QueueHandle_t tx_transp_layer_queue;
static QueueHandle_t rx_transp_layer_queue;

BaseType_t ol_init_net_if_buffers(void)
{
    for (int i = 0; i < NUMBER_OF_NET_IF_DESCRIPTORS; i++)
    {
        net_if_buffer_descriptors[i].puc_link_buffer = NULL;
        net_if_buffer_descriptors[i].packet_ack = NULL;
        net_if_buffer_descriptors[i].data_length = 0;
        net_if_buffer_descriptors[i].dst_addr = 0xFF;
    }
    ol_available_network_buffer = xSemaphoreCreateCounting(NUMBER_OF_NET_IF_DESCRIPTORS, NUMBER_OF_NET_IF_DESCRIPTORS);
    ol_network_buffer_mutex = xSemaphoreCreateMutex();
    if ((ol_available_network_buffer != NULL) && (ol_network_buffer_mutex != NULL))
    {
        return pdPASS;
    }
    return pdFAIL;
}

void ol_link_layer_task(void *arg);
void ol_transport_layer_task(void *arg);

BaseType_t ol_init(uint8_t nwk_id, uint8_t addr)
{
    if (ol_init_net_if_buffers() == pdFAIL)
    {
        return pdFAIL;
    }

    openlora.nwk_id = nwk_id;
    openlora.node_addr = addr;
    openlora.mac_seq_number = 0;
    memset(openlora.neigh_seq_number, 0xFF, 256);

    TaskHandle_t link_task_handle;
    if (xTaskCreate(ol_link_layer_task, "OL Link Layer Task", 2048, NULL, 4, &link_task_handle) != pdPASS)
    {
        vSemaphoreDelete(ol_available_network_buffer);
        vSemaphoreDelete(ol_network_buffer_mutex);
        return pdFAIL;
    }
    if (xTaskCreate(ol_transport_layer_task, "OL Transport Layer Task", 2048, NULL, 3, NULL) != pdPASS)
    {
        vSemaphoreDelete(ol_available_network_buffer);
        vSemaphoreDelete(ol_network_buffer_mutex);
        vTaskDelete(link_task_handle);
        return pdFAIL;
    }
    return pdPASS;
}

net_if_buffer_descriptor_t *ol_get_net_if_buffer(uint8_t size, uint32_t timeout)
{
    if (xSemaphoreTake(ol_available_network_buffer, timeout) == pdTRUE)
    {
        if (xSemaphoreTake(ol_network_buffer_mutex, timeout) == pdTRUE)
        {
            for (int i = 0; i < NUMBER_OF_NET_IF_DESCRIPTORS; i++)
            {
                if (net_if_buffer_descriptors[i].puc_link_buffer == NULL)
                {
                    net_if_buffer_descriptors[i].puc_link_buffer = pvPortMalloc(size);
                    net_if_buffer_descriptors[i].data_length = size;
                    xSemaphoreGive(ol_network_buffer_mutex);
                    return &net_if_buffer_descriptors[i];
                }
            }
            xSemaphoreGive(ol_network_buffer_mutex);
        }
        else
        {
            xSemaphoreGive(ol_available_network_buffer);
        }
    }
    return NULL;
}

BaseType_t ol_release_net_if_buffer(net_if_buffer_descriptor_t *buffer)
{
    // taskENTER_CRITICAL();
    if (xSemaphoreTake(ol_network_buffer_mutex, RELEASE_NET_IF_BUFFER_TIMEOUT) == pdTRUE)
    {
        buffer->data_length = 0;
        buffer->dst_addr = 0xFF;
        vPortFree(buffer->puc_link_buffer);
        if (buffer->packet_ack != NULL)
        {
            buffer->packet_ack = NULL;
        }
        buffer->puc_link_buffer = NULL;
        xSemaphoreGive(ol_available_network_buffer);
        xSemaphoreGive(ol_network_buffer_mutex);
        return pdPASS;
    }
    return pdFAIL;
    // taskEXIT_CRITICAL();
}

BaseType_t ol_get_number_of_free_net_if_buffer(void)
{
    return uxSemaphoreGetCount(ol_available_network_buffer);
}

uint32_t ol_send_link_frame(uint8_t dst_addr, net_if_buffer_descriptor_t *net_if_buffer, uint32_t timeout)
{
    link_layer_header_t *link_frame = (link_layer_header_t *)net_if_buffer->puc_link_buffer;

    link_frame->frame_type = DATA_FRAME;
    link_frame->network_id = openlora.nwk_id;
    link_frame->seq_number = openlora.mac_seq_number;
    link_frame->dst_addr = dst_addr;
    link_frame->src_addr = openlora.node_addr;
    link_frame->payload_size = net_if_buffer->data_length - sizeof(link_layer_header_t) - sizeof(link_layer_trailer_t);

    link_layer_trailer_t *link_trailer = (link_layer_trailer_t *)&net_if_buffer->puc_link_buffer[sizeof(link_layer_header_t) + link_frame->payload_size];

    link_trailer->crc = usLORACRC16(net_if_buffer->puc_link_buffer, sizeof(link_layer_header_t) + link_frame->payload_size);

    uint32_t link_retries = 0;
    do
    {
        /* Variables used for the backoff algorithm */
        // BackoffAlgorithmStatus_t retryStatus = BackoffAlgorithmSuccess;
        BackoffAlgorithmContext_t retryParams;
        uint16_t nextRetryBackoff = 0;

        /* Initialize reconnect attempts and interval. */
        BackoffAlgorithm_InitializeParams(&retryParams,
                                          RETRY_BACKOFF_BASE_MS,
                                          RETRY_MAX_BACKOFF_DELAY_MS,
                                          BACKOFF_RETRY_MAX_ATTEMPTS);

        // srand(xTaskGetTickCount());

        uint32_t backoff_retries = 0;
        do
        {
            if (BackoffAlgorithm_GetNextBackoff(&retryParams, esp_random(), &nextRetryBackoff) == BackoffAlgorithmSuccess)
            {
                ESP_LOGI(OPEN_LORA_TAG, "Backoff Time %d", nextRetryBackoff);
                vTaskDelay(nextRetryBackoff);

                if (lora_cca() == pdTRUE)
                {
                    // wait for channel availability
                    break;
                }
                backoff_retries++;
            }
            else
            {
                // BackoffAlgorithmRetriesExhausted
                ESP_LOGI(OPEN_LORA_TAG, "Fail to get a backoff time!");
                return pdFALSE;
            }
        } while (backoff_retries < LINK_RETRIES);

        if (backoff_retries >= LINK_RETRIES)
        {
            // couldn´t get the channel
            ESP_LOGI(OPEN_LORA_TAG, "Fail to get channel free to transmit!");
            return pdFALSE;
        }

        // ESP_LOGI(OPEN_LORA_TAG, "Transmitting an ol frame with %d bytes: %s", net_if_buffer->data_length, &net_if_buffer->puc_link_buffer[sizeof(link_layer_header_t)+sizeof(transport_layer_header_t)]);
        ESP_LOGI(OPEN_LORA_TAG, "Transmitting an ol frame with %d bytes", net_if_buffer->data_length);
        uint32_t ret = lora_send_frame(net_if_buffer->puc_link_buffer, net_if_buffer->data_length, timeout);
        if (ret == pdTRUE)
        {
            // wait for the ack
            ESP_LOGI(OPEN_LORA_TAG, "Waiting ACK");
            lora_receive(); // put into receive mode
            if (lora_received(LINK_ACK_TIMEOUT) == pdTRUE)
            {
                int len = lora_read_frame_size();
                if (len >= (sizeof(link_layer_header_t) + sizeof(link_layer_trailer_t)))
                {
                    /* todo: analisar ol_get_net_if_buffer para o pacote de ack */
                    net_if_buffer_descriptor_t *net_if_ack_buffer = ol_get_net_if_buffer((uint8_t)len, MAX_NET_IF_DESCRIPTORS_WAIT_TIME_MS);
                    if (net_if_ack_buffer != NULL)
                    {
                        int x = lora_read_frame(net_if_ack_buffer->puc_link_buffer, net_if_ack_buffer->data_length);
                        if (x)
                        {
                            uint16_t crc = usLORACRC16(net_if_ack_buffer->puc_link_buffer, (net_if_ack_buffer->data_length - sizeof(link_layer_trailer_t)));
                            // ESP_LOGI(OPEN_LORA_TAG, "calc ack: %x - packet acc: %x", crc, ack_packet.mac_ack_packet.crc);
                            link_layer_header_t *link_ack_frame = (link_layer_header_t *)net_if_ack_buffer->puc_link_buffer;
                            link_layer_trailer_t *link_ack_trailer = (link_layer_trailer_t *)&net_if_ack_buffer->puc_link_buffer[sizeof(link_layer_header_t)];
                            if (link_ack_trailer->crc == crc)
                            {
                                if ((link_ack_frame->frame_type == ACK_FRAME) && (link_ack_frame->dst_addr == openlora.node_addr))
                                {
                                    if ((link_ack_frame->seq_number == link_frame->seq_number) && (link_ack_frame->network_id == openlora.nwk_id))
                                    {
                                        openlora.mac_seq_number++;
                                        if (openlora.mac_seq_number >= 0xFE)
                                        {
                                            openlora.mac_seq_number = 0;
                                        }
                                        // todo: analyze ol_release_net_if_buffer fail
                                        (void)ol_release_net_if_buffer(net_if_ack_buffer);
                                        return pdTRUE;
                                    }
                                }
                            }
                            else
                            {
                                ESP_LOGI(OPEN_LORA_TAG, "ACK packet with wrong CRC");
                            }
                        }
                        (void)ol_release_net_if_buffer(net_if_ack_buffer);
                    }
                    else
                    {
                        ESP_LOGI(OPEN_LORA_TAG, "No free network buffer descritor to send ACK packet");
                    }
                }
            }
            else
            {
                ESP_LOGI(OPEN_LORA_TAG, "ACK packet timeout!");
            }
        }
        else
        {
            ESP_LOGI(OPEN_LORA_TAG, "Fail to sent the frame to the radio");
        }
        link_retries++;
    } while (link_retries < LINK_RETRIES);

    openlora.mac_seq_number++;
    if (openlora.mac_seq_number >= 0xFE)
    {
        openlora.mac_seq_number = 0;
    }

    return pdFALSE;
}

uint32_t ol_send_link_ack(net_if_buffer_descriptor_t *net_if_buffer, uint32_t timeout)
{
    /* todo: analisar o ol_get_net_if_buffer para o ol_send_link_ack */
    net_if_buffer_descriptor_t *net_if_ack_buffer = ol_get_net_if_buffer(sizeof(link_layer_header_t) + sizeof(link_layer_trailer_t), MAX_NET_IF_DESCRIPTORS_WAIT_TIME_MS);

    if (net_if_ack_buffer != NULL)
    {
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

        uint32_t ret = lora_send_frame(net_if_ack_buffer->puc_link_buffer, net_if_ack_buffer->data_length, timeout);
        // todo: analyze ol_release_net_if_buffer fail
        (void)ol_release_net_if_buffer(net_if_ack_buffer);
        return ret;
    }
    return pdFAIL;
}

void ol_receive_link_frame(uint32_t timeout)
{
    if (lora_received(timeout) == pdTRUE)
    {
        int len = lora_read_frame_size();
        /* todo: analisar o ol_get_net_if_buffer em ol_receive_link_frame */
        net_if_buffer_descriptor_t *net_if_buffer = ol_get_net_if_buffer((uint8_t)len, MAX_NET_IF_DESCRIPTORS_WAIT_TIME_MS);
        if (net_if_buffer != NULL)
        {
            int x = lora_read_frame(net_if_buffer->puc_link_buffer, net_if_buffer->data_length);
            if (x)
            {
                link_layer_header_t *link_frame_header = (link_layer_header_t *)net_if_buffer->puc_link_buffer;
                link_layer_trailer_t *link_frame_trailer = (link_layer_trailer_t *)&net_if_buffer->puc_link_buffer[sizeof(link_layer_header_t) + link_frame_header->payload_size];
                uint16_t crc = usLORACRC16(net_if_buffer->puc_link_buffer, (net_if_buffer->data_length - sizeof(link_layer_trailer_t)));
                if (link_frame_trailer->crc == crc)
                {
                    if ((link_frame_header->frame_type == DATA_FRAME) && (link_frame_header->network_id == openlora.nwk_id) && (link_frame_header->dst_addr == openlora.node_addr))
                    {
                        if (ol_send_link_ack(net_if_buffer, LINK_ACK_TIMEOUT) == pdTRUE)
                        {
                            if (openlora.neigh_seq_number[link_frame_header->src_addr] != link_frame_header->seq_number)
                            {
                                openlora.neigh_seq_number[link_frame_header->src_addr] = link_frame_header->seq_number;
                                if (xQueueSendToBack(rx_link_layer_queue, &net_if_buffer, timeout) == pdTRUE)
                                {
                                    ESP_LOGI(OPEN_LORA_TAG, "link frame sent to the upper layer");
                                }
                                else
                                {
                                    // todo: analyze ol_release_net_if_buffer fail
                                    (void)ol_release_net_if_buffer(net_if_buffer);
                                    ESP_LOGI(OPEN_LORA_TAG, "full upper layer queue");
                                }
                            }
                            else
                            {
                                // todo: analyze ol_release_net_if_buffer fail
                                (void)ol_release_net_if_buffer(net_if_buffer);
                                ESP_LOGI(OPEN_LORA_TAG, "Do not proccess a repeated frame!");
                            }
                        }
                        else
                        {
                            // todo: analyze ol_release_net_if_buffer fail
                            (void)ol_release_net_if_buffer(net_if_buffer);
                            ESP_LOGI(OPEN_LORA_TAG, "No free buffer descriptor to send ack packet");
                        }
                    }
                    else
                    {
                        /* todo: Testar se o pacote é broadcast */
                        // todo: analyze ol_release_net_if_buffer fail
                        (void)ol_release_net_if_buffer(net_if_buffer);
                        ESP_LOGI(OPEN_LORA_TAG, "not the link frame destination");
                    }
                }
                else
                {
                    // todo: analyze ol_release_net_if_buffer fail
                    (void)ol_release_net_if_buffer(net_if_buffer);
                    ESP_LOGI(OPEN_LORA_TAG, "link frame with wrong CRC");
                }
            }
            else
            {
                // todo: analyze ol_release_net_if_buffer fail
                (void)ol_release_net_if_buffer(net_if_buffer);
                ESP_LOGI(OPEN_LORA_TAG, "zero size link frame");
            }
        }
        else
        {
            ESP_LOGI(OPEN_LORA_TAG, "No free network buffer descriptor!");
        }
    }
}

BaseType_t ol_to_link_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout)
{
    return xQueueSendToBack(tx_link_layer_queue, &buffer, timeout);
}

BaseType_t ol_from_link_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout)
{
    return xQueueReceive(rx_link_layer_queue, buffer, timeout);
}

void ol_link_layer_task(void *arg)
{
    // esperar pacotes das camadas superiores
    // chegou um pacote do radio
    // transmitir um pacote (cca, csma/ca, retentativas, ack, ...)
    static const char *TAG = "ol_link_layer";
    bool is_queue_full = false;

    /* Create the queue set large enough to hold an event for every space in
    every queue and semaphore that is to be added to the set. */
    static QueueSetHandle_t link_event;
    QueueSetMemberHandle_t xActivatedMember;

    link_event = xQueueCreateSet(COMBINED_LENGTH_LINK);

    /* Create the queues and semaphores that will be contained in the set. */
    tx_link_layer_queue = xQueueCreate(LINK_LAYER_QUEUE_LENGTH, sizeof(net_if_buffer_descriptor_t *));
    rx_link_layer_queue = xQueueCreate(LINK_LAYER_QUEUE_LENGTH, sizeof(net_if_buffer_descriptor_t *));
    link_layer_tx_ready_signal = xSemaphoreCreateBinary();

    /* Check everything was created. */
    configASSERT(link_event);
    configASSERT(tx_link_layer_queue);
    configASSERT(rx_link_layer_queue);
    configASSERT(link_layer_tx_ready_signal);

    /* Add the queues and semaphores to the set.  Reading from these queues and
    semaphore can only be performed after a call to xQueueSelectFromSet() has
    returned the queue or semaphore handle from this point on. */
    xQueueAddToSet(tx_link_layer_queue, link_event);
    SemaphoreHandle_t sem_radio = lora_get_received_sem();
    xQueueAddToSet(sem_radio, link_event);

    while (1)
    {
        /* Enter in reception mode */
        lora_receive();
        /* Block to wait for something to be available from the queues or
        semaphore that have been added to the set. */
        xActivatedMember = xQueueSelectFromSet(link_event, portMAX_DELAY);

        if (xActivatedMember == tx_link_layer_queue)
        {
            // Transmitt a frame
            net_if_buffer_descriptor_t *frame;
            BaseType_t space = uxQueueSpacesAvailable(tx_link_layer_queue);
            ESP_LOGI(TAG, "Available space in the link layer RX queue: %d", space);
            if (space == 0)
            {
                is_queue_full = true;
            }
            xQueueReceive(xActivatedMember, &frame, portMAX_DELAY);

            uint8_t len = 0;
            if (ol_send_link_frame(frame->dst_addr, frame, portMAX_DELAY) != pdTRUE)
            {
                if (frame->packet_ack != NULL)
                {
                    xQueueSendToBack(frame->packet_ack, &len, 10);
                }
                ESP_LOGI(TAG, "Fail to sent the frame to the radio!");
            }
            else
            {
                if (frame->packet_ack != NULL)
                {
                    uint8_t len = frame->data_length - sizeof(link_layer_header_t) - sizeof(link_layer_trailer_t) - sizeof(transport_layer_header_t);
                    xQueueSendToBack(frame->packet_ack, &len, 10);
                }
            }
            // todo: analyze ol_release_net_if_buffer fail
            (void)ol_release_net_if_buffer(frame);
            if (is_queue_full && (uxQueueSpacesAvailable(tx_link_layer_queue) >= (LINK_LAYER_QUEUE_LENGTH / 2)))
            {
                is_queue_full = false;
                ESP_LOGI(TAG, "Re-enabled the send of network buffers to the link layer!");
                xSemaphoreGive(link_layer_tx_ready_signal);
            }
        }
        else if (xActivatedMember == sem_radio)
        {
            // Receive a frame
            ol_receive_link_frame(10);
        }
    }
}

void ol_transp_include_client_or_server(transport_layer_t *client_server)
{
    if (transp_list_tail != NULL)
    {
        /* Insert server/client into list */
        transp_list_tail->next = client_server;
        client_server->previous = transp_list_tail;
        transp_list_tail = client_server;
        transp_list_tail->next = NULL;
    }
    else
    {
        /* Init server/client list */
        transp_list_head = client_server;
        transp_list_tail = client_server;
        client_server->next = NULL;
        client_server->previous = NULL;
    }
}

void ol_transp_remove_client_or_server(transport_layer_t *client_server)
{
    if (client_server == transp_list_head)
    {
        if (client_server == transp_list_tail)
        {
            transp_list_head = NULL;
            transp_list_tail = NULL;
        }
        else
        {
            transp_list_head = client_server->next;
            transp_list_head->previous = NULL;
        }
    }
    else
    {
        if (client_server == transp_list_tail)
        {
            transp_list_tail = client_server->previous;
            transp_list_tail->next = NULL;
        }
        else
        {
            client_server->next->previous = client_server->previous;
            client_server->previous->next = client_server->next;
        }
    }
}

BaseType_t ol_transp_layer_receive_packet(net_if_buffer_descriptor_t *packet)
{
    // analyze the transport layer header
    // analisar o protocolo
    // varrer a lista de objetos da camada de transporte
    if (packet != NULL)
    {
        transport_layer_t *server_client = transp_list_head;
        while (server_client != NULL)
        {
            // is there a src port listening/waiting for this destination port?
            /*ESP_LOGI(OPEN_LORA_TAG, "Received packet:\n\r");
            for(int i=0; i<packet->data_length; i++){
                ESP_LOGI(OPEN_LORA_TAG, "%d ", packet->puc_link_buffer[i]);
            }*/
            // link_layer_header_t *link_frame_header = (link_layer_header_t *)packet->puc_link_buffer;
            transport_layer_header_t *transp_packet_header = (transport_layer_header_t *)&packet->puc_link_buffer[sizeof(link_layer_header_t)];
            if (server_client->src_port == transp_packet_header->dst_port)
            {
                server_client->payload_size = transp_packet_header->payload_size;
                server_client->sender_port = transp_packet_header->src_port;
                // server_client->sender_addr = link_frame_header->src_addr;
                ESP_LOGI(OPEN_LORA_TAG, "Wake up destination task!");
                xQueueSendToBack(server_client->transp_wakeup, &packet, 0);
                return pdPASS;
            }
            server_client = server_client->next;
        }
        ESP_LOGI(OPEN_LORA_TAG, "No upper layer waiting for this packet!");
        // todo: analyze ol_release_net_if_buffer fail
        (void)ol_release_net_if_buffer(packet);
    }
    else
    {
        ESP_LOGI(OPEN_LORA_TAG, "Can't get the network buffer from the lower layer!");
    }
    return pdFAIL;
}

BaseType_t ol_to_transport_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout)
{
    return xQueueSendToBack(tx_transp_layer_queue, &buffer, timeout);
}

BaseType_t ol_from_transport_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout)
{
    return xQueueReceive(rx_transp_layer_queue, buffer, timeout);
}

void ol_transport_layer_task(void *arg)
{
    // esperar pacotes das camadas superiores (em geral, aplicação)
    // encaminha pacotes da camada de rede e/ou enlace para as tasks usando a camada de transporte
    // opcional transmitir um pacote (retentativas, ack, ...)
    static const char *TAG = "ol_transport_layer";

    /* Create the queue set large enough to hold an event for every space in
    every queue and semaphore that is to be added to the set. */
    static QueueSetHandle_t transp_event;
    QueueSetMemberHandle_t xActivatedMember;

    transp_event = xQueueCreateSet(COMBINED_LENGTH_TRANSP);

    /* Create the queues and semaphores that will be contained in the set. */
    tx_transp_layer_queue = xQueueCreate(TRANSP_LAYER_QUEUE_TX_LENGTH, sizeof(net_if_buffer_descriptor_t *));
    rx_transp_layer_queue = xQueueCreate(TRANSP_LAYER_QUEUE_RX_LENGTH, sizeof(net_if_buffer_descriptor_t *));

    /* Check everything was created. */
    configASSERT(transp_event);
    configASSERT(tx_transp_layer_queue);
    configASSERT(rx_transp_layer_queue);

    /* Add the queues and semaphores to the set.  Reading from these queues and
    semaphore can only be performed after a call to xQueueSelectFromSet() has
    returned the queue or semaphore handle from this point on. */
    xQueueAddToSet(tx_transp_layer_queue, transp_event);
    xQueueAddToSet(rx_link_layer_queue, transp_event);
    xQueueAddToSet(link_layer_tx_ready_signal, transp_event);
    while (1)
    {
        /* Block to wait for something to be available from the queues or
        semaphore that have been added to the set. */
        xActivatedMember = xQueueSelectFromSet(transp_event, portMAX_DELAY);

        if (xActivatedMember == tx_transp_layer_queue)
        {
            // Transmit a transport segment or datagram to link layer
            net_if_buffer_descriptor_t *datagram = NULL;
            xQueueReceive(xActivatedMember, &datagram, 0);

            // the transport layer header where added by the transport layer interface
            // send to lower layer
            if (datagram != NULL)
            {
                if (ol_to_link_layer(datagram, 10) != pdTRUE)
                {
                    xQueueRemoveFromSet(xActivatedMember, transp_event);
                    ESP_LOGI(TAG, "Stop to send network buffers to the link layer!");
                    xQueueSendToFront(tx_transp_layer_queue, datagram, 10);
                }
            }
        }
        else if (xActivatedMember == rx_link_layer_queue)
        {
            // from link layer
            net_if_buffer_descriptor_t *packet = NULL;
            xQueueReceive(xActivatedMember, &packet, 0);
            (void)ol_transp_layer_receive_packet(packet);
        }
        else if (xActivatedMember == link_layer_tx_ready_signal)
        {
            xSemaphoreTake(xActivatedMember, 0);
            ESP_LOGI(TAG, "Re-enabled the send of network buffers to the link layer!");
            xQueueAddToSet(tx_transp_layer_queue, transp_event);
        }
    }
}

int ol_transp_open(transport_layer_t *client_server)
{
    ol_transp_include_client_or_server(client_server);
    // Create a queue
    client_server->transp_wakeup = xQueueCreate(1, sizeof(net_if_buffer_descriptor_t *));
    if (client_server->protocol == TRANSP_STREAM)
    {
        client_server->transp_ack = xQueueCreate(1, sizeof(uint8_t));
        if ((client_server->transp_wakeup != NULL) && (client_server->transp_ack != NULL))
        {
            return pdPASS;
        }
    }
    else
    {
        if (client_server->transp_wakeup != NULL)
        {
            return pdPASS;
        }
    }
    return pdFAIL;
}

int ol_transp_close(transport_layer_t *server_client)
{
    ol_transp_remove_client_or_server(server_client);
    // Delete the server/client semaphore
    vQueueDelete(server_client->transp_wakeup);
    vQueueDelete(server_client->transp_ack);
    return 0;
}

int ol_transp_recv(transport_layer_t *server_client, uint8_t *buffer, TickType_t timeout)
{
    // Wait for the semaphore
    net_if_buffer_descriptor_t *datagram = NULL;
    if (xQueueReceive(server_client->transp_wakeup, &datagram, timeout) == pdTRUE)
    {
        // something was receive
        transport_layer_header_t *transp_header = (transport_layer_header_t *)&datagram->puc_link_buffer[sizeof(link_layer_header_t)];
        uint8_t *payload = &datagram->puc_link_buffer[sizeof(link_layer_header_t) + sizeof(transport_layer_header_t)];
        memcpy(buffer, payload, transp_header->payload_size);
        int len = transp_header->payload_size;
        ol_release_net_if_buffer(datagram);
        return len;
    }
    return pdFAIL;
}

int ol_transp_send(transport_layer_t *server_client, uint8_t *buffer, uint16_t length, TickType_t timeout)
{
    // send a transport layer segment/datagram
    // todo: verificar a quantidade de net if buffers antes de tentar enviar
    int ret = 0;

    if (server_client->protocol == TRANSP_DATAGRAM)
    {
        if (length <= OL_TRANSPORT_MAX_PAYLOAD_SIZE)
        {
            int retries = 3;
            do
            {
                BaseType_t free_net_buffers = ol_get_number_of_free_net_if_buffer();
                if (free_net_buffers >= MIN_NUMBER_OF_NET_IF_DESCRIPTORS)
                {
                    net_if_buffer_descriptor_t *datagram = ol_get_net_if_buffer(sizeof(link_layer_header_t) + sizeof(link_layer_trailer_t) + sizeof(transport_layer_header_t) + length, timeout);
                    transport_layer_header_t *transp_header = (transport_layer_header_t *)&datagram->puc_link_buffer[sizeof(link_layer_header_t)];
                    uint8_t *payload = (uint8_t *)&datagram->puc_link_buffer[sizeof(link_layer_header_t) + sizeof(transport_layer_header_t)];
                    transp_header->src_port = server_client->src_port;
                    transp_header->dst_port = server_client->dst_port;
                    transp_header->payload_size = length;
                    transp_header->protocol = server_client->protocol;
                    datagram->dst_addr = server_client->dst_addr;
                    memcpy(payload, buffer, length);
                    /*
                    ESP_LOGI(OPEN_LORA_TAG, "Transmitted packet:\n\r");
                    for(int i=0; i<21; i++){
                        ESP_LOGI(OPEN_LORA_TAG, "%d ", datagram->puc_link_buffer[i]);
                    }
                    */
                    ol_to_transport_layer(datagram, timeout);
                    ret = length;
                    break;
                }
                else
                {
                    retries--;
                    vTaskDelay(timeout / 3);
                }
            } while (retries > 0);
        }
    }
    else if (server_client->protocol == TRANSP_STREAM)
    {
        uint16_t bytes_left = length;
        while (bytes_left > 0)
        {
            uint16_t bytes_count;
            if (bytes_left > OL_TRANSPORT_MAX_PAYLOAD_SIZE)
            {
                bytes_count = OL_TRANSPORT_MAX_PAYLOAD_SIZE;
            }
            else
            {
                bytes_count = bytes_left;
            }
            int retries = 3;
            do
            {
                BaseType_t free_net_buffers = ol_get_number_of_free_net_if_buffer();
                if (free_net_buffers >= MIN_NUMBER_OF_NET_IF_DESCRIPTORS)
                {
                    net_if_buffer_descriptor_t *segment = ol_get_net_if_buffer(sizeof(link_layer_header_t) + sizeof(link_layer_trailer_t) + sizeof(transport_layer_header_t) + bytes_count, timeout);
                    transport_layer_header_t *transp_header = (transport_layer_header_t *)&segment->puc_link_buffer[sizeof(link_layer_header_t)];
                    uint8_t *payload = (uint8_t *)&segment->puc_link_buffer[sizeof(link_layer_header_t) + sizeof(transport_layer_header_t)];
                    transp_header->src_port = server_client->src_port;
                    transp_header->dst_port = server_client->dst_port;
                    transp_header->payload_size = bytes_count;
                    transp_header->protocol = server_client->protocol;
                    segment->dst_addr = server_client->dst_addr;
                    memcpy(payload, buffer, bytes_count);
                    /*
                    ESP_LOGI(OPEN_LORA_TAG, "Transmitted packet:\n\r");
                    for(int i=0; i<21; i++){
                        ESP_LOGI(OPEN_LORA_TAG, "%d ", datagram->puc_link_buffer[i]);
                    }
                    */
                    segment->packet_ack = server_client->transp_ack;
                    ol_to_transport_layer(segment, timeout);

                    // Wait for confirmation of the delivered package or error
                    uint8_t len = 0;
                    xQueueReceive(server_client->transp_ack, &len, timeout);
                    if (len == bytes_count)
                    {
                        // increments the return total length
                        ret += bytes_count;
                        // decrements the bytes left
                        bytes_left -= bytes_count;
                        // increments the payload buffer pointer
                        buffer += bytes_count;
                    }
                    break;
                }
                else
                {
                    retries--;
                    vTaskDelay(timeout / 3);
                }
            } while (retries > 0);
        }
    }
    return ret;
}

// int calculate_sha256(const uint8_t *data, size_t length, uint8_t *digest)
// {
//     SHA256_CTX sha256;
//     if (!SHA256_Init(&sha256))
//     {
//         return -1; // Falha na inicialização do contexto SHA-256
//     }

//     if (!SHA256_Update(&sha256, data, length))
//     {
//         return -1; // Falha na atualização do contexto SHA-256
//     }

//     if (!SHA256_Final(digest, &sha256))
//     {
//         return -1; // Falha na finalização do contexto SHA-256
//     }

//     return 0; // Sucesso
// }

BaseType_t ol_app_send_file(uint8_t *file_content)
{
    static const char *TAG = "ol_app_send_file";
    transport_layer_t client;
    client.protocol = TRANSP_STREAM;
    client.src_port = OL_TRANSPORT_CLIENT_PORT_INIT + 1;
    client.dst_port = 1;
    client.dst_addr = OL_BORDER_ROUTER_ADDR;

    int ret = ol_transp_open(&client);

    if (ret == pdFAIL)
    {
        ESP_LOGI(TAG, "Error on transp open");
        return pdFAIL;
    }

    size_t file_size = strlen((const char *)file_content);

    // Calcular o hash SHA-256 do conteúdo do arquivo
    uint8_t sha256_digest[SHA256_DIGEST_LENGTH] = "asf";
    // if (calculate_sha256((const uint8_t *)file_content, file_size, sha256_digest) != 0)
    // {
    //     ESP_LOGI(TAG, "Erro ao calcular o hash SHA-256 do arquivo");
    //     ol_transp_close(&client);
    //     return -1;
    // }

    size_t header_size = sizeof(app_layer_header_t);

    // Enviar o novo buffer (com cabeçalho) em chunks
    const uint8_t *current_position = file_content;
    size_t remaining_size = file_size;

    while (remaining_size > 0)
    {
        size_t bytes_to_send = (remaining_size > FILE_BUFFER_SIZE) ? FILE_BUFFER_SIZE : remaining_size;

        // Criar um novo buffer para incluir o cabeçalho e o conteúdo do arquivo
        size_t total_size = header_size + bytes_to_send;
        uint8_t buffer[total_size];

        // Adicionar o cabeçalho ao novo buffer
        app_layer_header_t *header = (app_layer_header_t *)buffer;

        header->message_type = FILE_CONTENT_TYPE;
        
        header->content_length = FILE_BUFFER_SIZE;
        // Copiar parte do conteúdo do arquivo após o cabeçalho
        memcpy(buffer, file_content, total_size);

        // Enviar o chunk usando a função de transporte existente
        int sent_bytes = ol_transp_send(&client, (uint8_t *)current_position, bytes_to_send, portMAX_DELAY);

        if (sent_bytes <= 0)
        {
            ESP_LOGI(TAG, "Erro ao enviar o chunk do arquivo");
            ol_transp_close(&client);
            return -1;
        }

        // Atualizar a posição atual e o tamanho restante
        current_position += sent_bytes;
        remaining_size -= sent_bytes;

        // Atualizar o cabeçalho para o próximo chunk
        header->content_length = (remaining_size > FILE_BUFFER_SIZE) ? FILE_BUFFER_SIZE : remaining_size;
        memcpy(buffer + header_size, current_position, header->content_length);

        vTaskDelay(10); // Adicionar um pequeno atraso entre os chunks se necessário
    }

    // Enviar o hash SHA-256 (cabeçalho + hash)
    app_layer_header_t *header_sha256 = NULL;

    header_sha256->message_type = SHA256_DIGEST_TYPE;
    header_sha256->content_length = SHA256_DIGEST_LENGTH;
    
    uint8_t *buffer_sha256 = NULL;

    // Enviar o cabeçalho
    ol_transp_send(&client, buffer_sha256, sizeof(header_sha256) + SHA256_DIGEST_LENGTH, portMAX_DELAY);

    ol_transp_close(&client); // Fechar a conexão após o envio do arquivo e do hash
    return 0;                 // Sucesso
}