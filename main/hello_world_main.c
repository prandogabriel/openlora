/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "lora.h"
#include "openlora.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include "u8g2_esp32_hal.h"
#include <u8g2.h>


#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sdmmc_cmd.h"

#include "driver/uart.h"
#include "string.h"

#define TRANSMITTER 1
#define RECEIVER    2
#define MODE TRANSMITTER


#define BOARD_V1        1
#define BOARD_V2        2
#define BOARD_V2_16     3
#define BOARD_TTGO_BEAM 4

#define BOARD   BOARD_V2


#define PRINT_DEBUG 0

#define VERBOSE     1

#define HAVE_LORA   1
#define HAVE_SDCARD 0
#define HAVE_OLED   0

// Utilizado para debug em campo
#define HAVE_LED    0


#if (BOARD == BOARD_V1)
    // SDA - GPIO21
    #define PIN_SDA 4
    // SCL - GPIO22
    #define PIN_SCL 15
    #define PIN_RST 16
#else
    // SDA - GPIO21
    #define PIN_SDA 21
    // SCL - GPIO22
    #define PIN_SCL 22
#endif

#if (BOARD == BOARD_TTGO_BEAM)
    #define LED_1                                       14
    //#define LED_2                                       4
#elif (BOARD == BOARD_V1)
    #define LED_1                                       2
    #define LED_2                                       12
#elif (BOARD == BOARD_V2_16)
    #define LED_1                                       2
    //#define LED_2                                       12
#else
    #define LED_1                                       22
    #define LED_2                                       4
#endif

static const char *TAG = "ssd1306";
volatile char global_counter[6] = {'T', 'e', 's', 't', 'e', '\0'};

void task_test_SSD1306i2c(void *ignore) {
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
    #if (BOARD == BOARD_V1)
    u8g2_esp32_hal.reset   = PIN_RST;
    #endif    
	u8g2_esp32_hal_init(u8g2_esp32_hal);


	u8g2_t u8g2; // a structure which will contain all the data for one display
	u8g2_Setup_ssd1306_i2c_128x32_univision_f(
		&u8g2,
		U8G2_R0,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	ESP_LOGI(TAG, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

	ESP_LOGI(TAG, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	ESP_LOGI(TAG, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(&u8g2);
	ESP_LOGI(TAG, "u8g2_DrawBox");
	u8g2_DrawBox(&u8g2, 0, 26, 80,6);
	u8g2_DrawFrame(&u8g2, 0,26,100,6);

	ESP_LOGI(TAG, "u8g2_SetFont");
    	u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
	ESP_LOGI(TAG, "u8g2_DrawStr");
    	u8g2_DrawStr(&u8g2, 2,17,"Hi nkolban!");
	ESP_LOGI(TAG, "u8g2_SendBuffer");
	u8g2_SendBuffer(&u8g2);

	ESP_LOGI(TAG, "All done!");

	char string[18];
	//int i = 0;

	u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);

    uint32_t cnt=0;
	while(1){
		vTaskDelay(2000);
		u8g2_ClearBuffer(&u8g2);
		sprintf(string, "Recebendo %ld", cnt++);
	    u8g2_DrawStr(&u8g2, 1,17,string);
		//ESP_LOGI(TAG, "u8g2_SendBuffer");
		u8g2_SendBuffer(&u8g2);
	}

}

const char *file;
void lora_transmit_task(void *param) {
    (void)param;
    // static const char *TAG = "lora_tx";
    while(1) {
        (void)ol_app_send_file(file);
        
        ESP_LOGI(TAG, "Next iteration" );
        
        vTaskDelay(1000);
    }
    // transport_layer_t client;
    // client.protocol = TRANSP_STREAM;
    // client.src_port = OL_TRANSPORT_CLIENT_PORT_INIT+1;
	// client.dst_port = 1;
    // client.dst_addr = OL_BORDER_ROUTER_ADDR;
    // int ret = ol_transp_open(&client);
    // if (ret == pdFAIL) {
    //     ESP_LOGI(TAG, "It was not possible to connect to the %d port", client.dst_port );
    //     vTaskSuspend(NULL);
    // }
    // while(1) {
    //     #if 1
    //     uint8_t *pfile = file;
    //     int full_len = strlen(file);
    //     uint16_t len = 0;
    //     while(*file){
    //         if (full_len >= 1024){
    //             len = 1024;
    //         }else {
    //             len = full_len;
    //         }
    //         int sent = ol_transp_send(&client, pfile, len, portMAX_DELAY);
    //         if (sent == len){
    //             pfile += sent;
    //             full_len -= sent;
    //             ESP_LOGI(TAG, "Sent %d bytes to the transport layer task.", sent);
    //         }else {
    //             ESP_LOGI(TAG, "Fail to sent the streaming data!");
    //             break;
    //         }
    //         vTaskDelay(10);
    //     }
    //     #else
    //     char buffer[16];
    //     uint16_t cnt = 0;
    
    //     int len = sprintf(buffer, "Teste: %d\n", cnt++);
    //     ESP_LOGI(TAG, "Destination addr: %d, Source port: %d, Destination port: %d", client.dst_addr, client.src_port, client.dst_port);
    //     ol_transp_send(&client, (uint8_t *)buffer, len, portMAX_DELAY);
    //     #endif
    //     /*
    //     net_if_buffer_descriptor_t *packet  = ol_get_net_if_buffer(sizeof(link_layer_header_t)+sizeof(link_layer_trailer_t)+len, 100);
    //     if (packet != NULL) {
    //         memcpy(&packet->puc_link_buffer[sizeof(link_layer_header_t)], buffer, len+1);
    //         ol_to_link_layer(packet, portMAX_DELAY);
    //     }
    //     */
    //     #if 0
    //     int len = sprintf(buffer, "Teste: %d\n", cnt++);
    //     if (lora_send_frame((uint8_t *)buffer, len, 1000) == pdTRUE) {
    //         ESP_LOGI(TAG, "Frame transmitted!");
    //     }else {
    //         ESP_LOGI(TAG, "Failure on transmitting frame!");
    //     }
    //     #endif

    //     vTaskDelay(1000);
    // }
}

void lora_receive_task(void *param) {
    (void)param;
    static const char *TAG = "lora_rx";
    char buffer[256];
    transport_layer_t server;
    server.protocol = TRANSP_STREAM;
    server.src_port = 1;
	server.dst_port = OL_TRANSPORT_CLIENT_PORT_INIT+1;
    int ret = ol_transp_open(&server);
    if (ret == pdFAIL) {
        ESP_LOGI(TAG, "It was not possible to listen the %d port", server.src_port );
        vTaskSuspend(NULL);
    }
    while(1) {
        memset(buffer, 0, 16);
        int len = ol_transp_recv(&server, (uint8_t *)buffer, portMAX_DELAY);
        if (len > 0){
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Size: %d - %s", len, buffer);
        }else {
            ESP_LOGI(TAG, "Reception timeout!");
        }
        /*
        lora_receive();
        lora_received(portMAX_DELAY);
        memset(buffer, 0, 16);
        int len = lora_read_frame_size();
        lora_read_frame((uint8_t *)buffer, 16);
        ESP_LOGI(TAG, "Size: %d - %s", len, buffer);
        */
    }
}

void lora_receive_task_2(void *param) {
    (void)param;
    static const char *TAG = "lora_rx_2";
    char buffer[16];
    transport_layer_t server;
    server.protocol = TRANSP_DATAGRAM;
    server.src_port = 2;
	server.dst_port = OL_TRANSPORT_CLIENT_PORT_INIT+2;
    int ret = ol_transp_open(&server);
    if (ret == pdFAIL) {
        ESP_LOGI(TAG, "It was not possible to listen the %d port", server.src_port );
        vTaskSuspend(NULL);
    }
    while(1) {
        memset(buffer, 0, 16);
        int len = ol_transp_recv(&server, (uint8_t *)buffer, portMAX_DELAY);
        if (len > 0){
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Size: %d - %s", len, buffer);
        }else {
            ESP_LOGI(TAG, "Reception timeout!");
        }
    }
}

void lora_receive_task_3(void *param) {
    (void)param;
    static const char *TAG = "lora_rx_3";
    char buffer[16];
    transport_layer_t server;
    server.protocol = TRANSP_DATAGRAM;
    server.src_port = 3;
	server.dst_port = OL_TRANSPORT_CLIENT_PORT_INIT+2;
    int ret = ol_transp_open(&server);
    if (ret == pdFAIL) {
        ESP_LOGI(TAG, "It was not possible to listen the %d port", server.src_port );
        vTaskSuspend(NULL);
    }
    while(1) {
        memset(buffer, 0, 16);
        int len = ol_transp_recv(&server, (uint8_t *)buffer, portMAX_DELAY);
        if (len > 0){
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Size: %d - %s", len, buffer);
        }else {
            ESP_LOGI(TAG, "Reception timeout!");
        }
    }
}

void app_main()
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    uint32_t size_flash_chip;
    esp_flash_get_size(NULL, &size_flash_chip);

    printf("%ldMB %s flash\n", size_flash_chip / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

   // Tarefas lora
   // Init LoRa with datarate 4
   if (lora_init(4, CHANNEL_0, 20, true, true, true)) {
        // init openlora stack
        #if MODE == RECEIVER
        if (ol_init(1, OL_BORDER_ROUTER_ADDR) == pdTRUE){
            xTaskCreate(lora_receive_task, "task_lora_tx", 3072, NULL, 4, NULL);
            xTaskCreate(lora_receive_task_2, "task_lora_tx", 2048, NULL, 4, NULL);
            xTaskCreate(lora_receive_task_3, "task_lora_tx", 2048, NULL, 4, NULL);
        }
        #else
        if (ol_init(1, 1) == pdTRUE) {
            xTaskCreate(lora_transmit_task, "task_lora_tx", 2048, NULL, 2, NULL);
        }
        #endif
   }

   #if HAVE_OLED == 1
    // Tarefa OLED
   xTaskCreate(&task_test_SSD1306i2c, "task_oled", 10*1024, NULL, 4, NULL);
   #endif

   // Tarefa cartão SD
   //xTaskCreate(&task_sdcard, "task_sdcard", 10*1024, NULL, 3, NULL);
   //xTaskCreate(&task_tasklist, "Task List", 4096, NULL,2,NULL);

}


#if MODE == TRANSMITTER
const char *file = "Internet Engineering Task Force (IETF)\n"
"Request for Comments: 8180               Universitat Oberta de Catalunya\n"
"BCP: 210                                                       K. Pister\n"
"Category: Best Current Practice        University of California Berkeley\n"
"ISSN: 2070-1721                                              T. Watteyne\n"
"                                                          Analog Devices\n"
"                                                                May 2017\n"
"                                                                \n"
"Minimal IPv6 over the TSCH Mode of IEEE 802.15.4e (6TiSCH) Configuration\n"
"\n"
"Abstract\n"
"\n"
"   This document describes a minimal mode of operation for an IPv6 over\n"
"   the TSCH mode of IEEE 802.15.4e (6TiSCH) network.  This minimal mode\n"
"   of operation specifies the baseline set of protocols that need to be\n"
"   supported and the recommended configurations and modes of operation\n"
"   sufficient to enable a 6TiSCH functional network.  6TiSCH provides\n"
"   IPv6 connectivity over a Time-Slotted Channel Hopping (TSCH) mesh\n"
"   composed of IEEE Std 802.15.4 TSCH links.  This minimal mode uses a\n"
"   collection of protocols with the respective configurations, including\n"
"   the IPv6 Low-Power Wireless Personal Area Network (6LoWPAN)\n"
"   framework, enabling interoperable IPv6 connectivity over IEEE Std\n"
"   802.15.4 TSCH.  This minimal configuration provides the necessary\n"
"   bandwidth for network and security bootstrapping and defines the\n"
"   proper link between the IETF protocols that interface to IEEE Std\n"
"   802.15.4 TSCH.  This minimal mode of operation should be implemented\n"
"   by all 6TiSCH-compliant devices.\n"
"\n"
"Status of This Memo\n"
"\n"
"   This memo documents an Internet Best Current Practice.\n"
"\n"
"   This document is a product of the Internet Engineering Task Force\n"
"   (IETF).  It represents the consensus of the IETF community.  It has\n"
"   received public review and has been approved for publication by the\n"
"   Internet Engineering Steering Group (IESG).  Further information on\n"
"   BCPs is available in Section 2 of RFC 7841.\n"
"\n"
"   Information about the current status of this document, any errata,\n"
"   and how to provide feedback on it may be obtained at\n"
"   http://www.rfc-editor.org/info/rfc8180.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 1]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"Copyright Notice\n"
"\n"
"   Copyright (c) 2017 IETF Trust and the persons identified as the\n"
"   document authors.  All rights reserved.\n"
"\n"
"   This document is subject to BCP 78 and the IETF Trust's Legal\n"
"   Provisions Relating to IETF Documents\n"
"   (http://trustee.ietf.org/license-info) in effect on the date of\n"
"   publication of this document.  Please review these documents\n"
"   carefully, as they describe your rights and restrictions with respect\n"
"   to this document.  Code Components extracted from this document must\n"
"   include Simplified BSD License text as described in Section 4.e of\n"
"   the Trust Legal Provisions and are provided without warranty as\n"
"   described in the Simplified BSD License.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 2]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"Table of Contents\n"
"\n"
"   1.  Introduction  . . . . . . . . . . . . . . . . . . . . . . . .   4\n"
"   2.  Requirements Language . . . . . . . . . . . . . . . . . . . .   4\n"
"   3.  Terminology . . . . . . . . . . . . . . . . . . . . . . . . .   5\n"
"   4.  IEEE Std 802.15.4 Settings  . . . . . . . . . . . . . . . . .   5\n"
"     4.1.  TSCH Schedule . . . . . . . . . . . . . . . . . . . . . .   6\n"
"     4.2.  Cell Options  . . . . . . . . . . . . . . . . . . . . . .   8\n"
"     4.3.  Retransmissions . . . . . . . . . . . . . . . . . . . . .   8\n"
"     4.4.  Timeslot Timing . . . . . . . . . . . . . . . . . . . . .   8\n"
"     4.5.  Frame Contents  . . . . . . . . . . . . . . . . . . . . .   8\n"
"       4.5.1.  IEEE Std 802.15.4 Header  . . . . . . . . . . . . . .   8\n"
"       4.5.2.  Enhanced Beacon Frame . . . . . . . . . . . . . . . .   9\n"
"       4.5.3.  Acknowledgment Frame  . . . . . . . . . . . . . . . .  10\n"
"     4.6.  Link-Layer Security . . . . . . . . . . . . . . . . . . .  10\n"
"   5.  RPL Settings  . . . . . . . . . . . . . . . . . . . . . . . .  11\n"
"     5.1.  Objective Function  . . . . . . . . . . . . . . . . . . .  11\n"
"       5.1.1.  Rank Computation  . . . . . . . . . . . . . . . . . .  11\n"
"       5.1.2.  Rank Computation Example  . . . . . . . . . . . . . .  13\n"
"     5.2.  Mode of Operation . . . . . . . . . . . . . . . . . . . .  14\n"
"     5.3.  Trickle Timer . . . . . . . . . . . . . . . . . . . . . .  14\n"
"     5.4.  Packet Contents . . . . . . . . . . . . . . . . . . . . .  14\n"
"   6.  Network Formation and Lifetime  . . . . . . . . . . . . . . .  14\n"
"     6.1.  Value of the Join Metric Field  . . . . . . . . . . . . .  14\n"
"     6.2.  Time-Source Neighbor Selection  . . . . . . . . . . . . .  15\n"
"     6.3.  When to Start Sending EBs . . . . . . . . . . . . . . . .  15\n"
"     6.4.  Hysteresis  . . . . . . . . . . . . . . . . . . . . . . .  15\n"
"   7.  Implementation Recommendations  . . . . . . . . . . . . . . .  16\n"
"     7.1.  Neighbor Table  . . . . . . . . . . . . . . . . . . . . .  16\n"
"     7.2.  Queues and Priorities . . . . . . . . . . . . . . . . . .  16\n"
"     7.3.  Recommended Settings  . . . . . . . . . . . . . . . . . .  17\n"
"   8.  Security Considerations . . . . . . . . . . . . . . . . . . .  17\n"
"   9.  IANA Considerations . . . . . . . . . . . . . . . . . . . . .  19\n"
"   10. References  . . . . . . . . . . . . . . . . . . . . . . . . .  19\n"
"     10.1.  Normative References . . . . . . . . . . . . . . . . . .  19\n"
"     10.2.  Informative References . . . . . . . . . . . . . . . . .  21\n"
"   Appendix A.  Examples . . . . . . . . . . . . . . . . . . . . . .  23\n"
"     A.1.  Example: EB with Default Timeslot Template  . . . . . . .  23\n"
"     A.2.  Example: EB with Custom Timeslot Template . . . . . . . .  25\n"
"     A.3.  Example: Link-layer Acknowledgment  . . . . . . . . . . .  27\n"
"     A.4.  Example: Auxiliary Security Header  . . . . . . . . . . .  27\n"
"   Acknowledgments . . . . . . . . . . . . . . . . . . . . . . . . .  28\n"
"   Authors' Addresses  . . . . . . . . . . . . . . . . . . . . . . .  28\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 3]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"1.  Introduction\n"
"\n"
"   A 6TiSCH network provides IPv6 connectivity [RFC2460] over a Time-\n"
"   Slotted Channel Hopping (TSCH) mesh [RFC7554] composed of IEEE Std\n"
"   802.15.4 TSCH links [IEEE.802.15.4].  IPv6 connectivity is obtained\n"
"   by the use of the 6LoWPAN framework ([RFC4944], [RFC6282],\n"
"   [RFC8025],[RFC8138], and [RFC6775]), RPL [RFC6550], and the RPL\n"
"   Objective Function 0 (OF0) [RFC6552].\n"
"\n"
"   This specification defines operational parameters and procedures for\n"
"   a minimal mode of operation to build a 6TiSCH network.  Any 6TiSCH-\n"
"   compliant device should implement this mode of operation.  This\n"
"   operational parameter configuration provides the necessary bandwidth\n"
"   for nodes to bootstrap the network.  The bootstrap process includes\n"
"   initial network configuration and security bootstrapping.  In this\n"
"   specification, the 802.15.4 TSCH mode, the 6LoWPAN framework, RPL\n"
"   [RFC6550], and the RPL Objective Function 0 (OF0) [RFC6552] are used\n"
"   unmodified.  Parameters and particular operations of TSCH are\n"
"   specified to guarantee interoperability between nodes in a 6TiSCH\n"
"   network.\n"
"\n"
"   In a 6TiSCH network, nodes follow a communication schedule as per\n"
"   802.15.4 TSCH.  Nodes learn the communication schedule upon joining\n"
"   the network.  When following this specification, the learned schedule\n"
"   is the same for all nodes and does not change over time.  Future\n"
"   specifications may define mechanisms for dynamically managing the\n"
"   communication schedule.  Dynamic scheduling solutions are out of\n"
"   scope of this document.\n"
"\n"
"   IPv6 addressing and compression are achieved by the 6LoWPAN\n"
"   framework.  The framework includes [RFC4944], [RFC6282], [RFC8025],\n"
"   the 6LoWPAN Routing Header dispatch [RFC8138] for addressing and\n"
"   header compression, and [RFC6775] for Duplicate Address Detection\n"
"   (DAD) and address resolution.\n"
"\n"
"   More advanced work is expected in the future to complement the\n"
"   minimal configuration with dynamic operations that can adapt the\n"
"   schedule to the needs of the traffic at run time.\n"
"\n"
"2.  Requirements Language\n"
"\n"
"   The key words \\\"MUST\\\", \\\"MUST NOT\\\", \\\"REQUIRED\\\", \\\"SHALL\\\", \\\"SHALL NOT\\\",\n"
"   \\\"SHOULD\\\", \\\"SHOULD NOT\\\", \\\"RECOMMENDED\\\", \\\"NOT RECOMMENDED\\\", \\\"MAY\\\", and\n"
"   \\\"OPTIONAL\\\" in this document are to be interpreted as described in BCP\n"
"   14 [RFC2119] [RFC8174] when, and only when, they appear in all\n"
"   capitals, as shown here.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 4]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"3.  Terminology\n"
"\n"
"   This document uses terminology from [TERMS-6TiSCH].  The following\n"
"   concepts are used in this document:\n"
"\n"
"   802.15.4:  We use \\\"802.15.4\\\" as a short version of \\\"IEEE Std\n"
"      802.15.4\\\" in this document.\n"
"\n"
"   SFD:  Start of Frame Delimiter\n"
"\n"
"   RX:  Reception\n"
"\n"
"   TX:  Transmission\n"
"\n"
"   IE:  Information Element\n"
"\n"
"   EB:  Enhanced Beacon\n"
"\n"
"   ASN:  Absolute Slot Number\n"
"\n"
"   Join Metric:  Field in the TSCH Synchronization IE representing the\n"
"      topological distance between the node sending the EB and the PAN\n"
"      coordinator.\n"
"\n"
"   PAN:  Personal Area Network\n"
"\n"
"   MLME:  MAC Layer Management Entity\n"
"\n"
"4.  IEEE Std 802.15.4 Settings\n"
"\n"
"   An implementation compliant with this specification MUST implement\n"
"   IEEE Std 802.15.4 [IEEE.802.15.4] in Time-Slotted Channel Hopping\n"
"   (TSCH) mode.\n"
"\n"
"   The remainder of this section details the RECOMMENDED TSCH settings,\n"
"   which are summarized in Figure 1.  Any of the properties marked in\n"
"   the EB column are announced in the EBs the nodes send [IEEE.802.15.4]\n"
"   and learned by those joining the network.  Changing their value means\n"
"   changing the contents of the EB.\n"
"\n"
"   In case of discrepancy between the values in this specification and\n"
"   IEEE Std 802.15.4 [IEEE.802.15.4], the IEEE standard has precedence.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 5]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   +--------------------------------+------------------------------+---+\n"
"   |           Property             |     Recommended Setting      |EB*|\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Slotframe Size                 | Tunable. Trades off          | X |\n"
"   |                                | bandwidth against energy.    |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Number of scheduled cells**    | 1                            | X |\n"
"   | (active)                       | Timeslot        0x0000       |   |\n"
"   |                                | Channel Offset  0x0000       |   |\n"
"   |                                | Link Options = (TX Link = 1, |   |\n"
"   |                                | RX Link = 1, Shared Link = 1,|   |\n"
"   |                                | Timekeeping = 1)             |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Number of unscheduled cells    | All remaining cells in the   | X |\n"
"   | (off)                          | slotframe.                   |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Max Number MAC retransmissions | 3 (4 transmission attempts)  |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Timeslot template              | IEEE Std 802.15.4 default    | X |\n"
"   |                                | (macTimeslotTemplateId=0)    |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Enhanced Beacon Period         | Tunable. Trades off join     |   |\n"
"   | (EB_PERIOD)                    | time against energy.         |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Number used frequencies        | IEEE Std 802.15.4 default    | X |\n"
"   | (2.4 GHz O-QPSK PHY)           | (16)                         |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"   | Channel Hopping sequence       | IEEE Std 802.15.4 default    | X |\n"
"   | (2.4 GHz O-QPSK PHY)           | (macHoppingSequenceID = 0)   |   |\n"
"   +--------------------------------+------------------------------+---+\n"
"     * An \"X\" in this column means this property's value is announced\n"
"       in the EB; hence, a new node learns it when joining.\n"
"    ** This cell LinkType is set to ADVERTISING.\n"
"\n"
"           Figure 1: Recommended IEEE Std 802.15.4 TSCH Settings\n"
"\n"
"4.1.  TSCH Schedule\n"
"\n"
"   This minimal mode of operation uses a single slotframe.  The TSCH\n"
"   slotframe is composed of a tunable number of timeslots.  The\n"
"   slotframe size (i.e., the number of timeslots it contains) trades off\n"
"   bandwidth for energy consumption.  The slotframe size needs to be\n"
"   tuned; the way of tuning it is out of scope of this specification.\n"
"   The slotframe size is announced in the EB.  The RECOMMENDED value for\n"
"   the slotframe handle (macSlotframeHandle) is 0x00.  An implementation\n"
"   MAY choose to use a different slotframe handle, for example, to add\n"
"   other slotframes with higher priority.  The use of other slotframes\n"
"   is out of the scope of this document.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 6]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   There is only a single scheduled cell in the slotframe.  This cell\n"
"   MAY be scheduled at any slotOffset/channelOffset within the\n"
"   slotframe.  The location of that cell in the schedule is announced in\n"
"   the EB.  The LinkType of the scheduled cell is ADVERTISING to allow\n"
"   EBs to be sent on it.\n"
"\n"
"   Figure 2 shows an example of a slotframe of length 101 timeslots,\n"
"   resulting in a radio duty cycle below 0.99\\%.\n"
"\n"
"      Chan.  +----------+----------+          +----------+\n"
"      Off.0  | TxRxS/EB |   OFF    |          |   OFF    |\n"
"      Chan.  +----------+----------+          +----------+\n"
"      Off.1  |   OFF    |   OFF    |   ...    |   OFF    |\n"
"             +----------+----------+          +----------+\n"
"                 .\n"
"                 .\n"
"                 .\n"
"      Chan.  +----------+----------+          +----------+\n"
"      Off.15 |   OFF    |   OFF    |          |   OFF    |\n"
"             +----------+----------+          +----------+\n"
"\n"
"   slotOffset     0          1                    100\n"
"\n"
"   EB:  Enhanced Beacon\n"
"   Tx:  Transmit\n"
"   Rx:  Receive\n"
"   S:   Shared\n"
"   OFF: Unscheduled by this specification\n"
"\n"
"            Figure 2: Example Slotframe of Length 101 Timeslots\n"
"\n"
"   A node MAY use the scheduled cell to transmit/receive all types of\n"
"   link-layer frames.  EBs are sent to the link-layer broadcast address\n"
"   and are not acknowledged.  Data frames are sent unicast and are\n"
"   acknowledged by the receiving neighbor.\n"
"\n"
"   All remaining cells in the slotframe are unscheduled.  Dynamic\n"
"   scheduling solutions may be defined in the future that schedule those\n"
"   cells.  One example is the 6top Protocol (6P) [PROTO-6P].  Dynamic\n"
"   scheduling solutions are out of scope of this document.\n"
"\n"
"   The default values of the TSCH timeslot template (defined in\n"
"   Section 8.4.2.2.3 of [IEEE.802.15.4]) and channel hopping sequence\n"
"   (defined in Section 6.2.10 of [IEEE.802.15.4]) SHOULD be used.  A\n"
"   node MAY use different values by properly announcing them in its EB.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 7]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"4.2.  Cell Options\n"
"\n"
"   In the scheduled cell, a node transmits if there is a packet to\n"
"   transmit and listens otherwise (both \\\"TX\\\" and \\\"RX\\\" bits are set).\n"
"   When a node transmits, requesting a link-layer acknowledgment per\n"
"   [IEEE.802.15.4], and does not receive the requested acknowledgement,\n"
"   it uses a back-off mechanism to resolve possible collisions (\\\"Shared\\\"\n"
"   bit is set).  A node joining the network maintains time\n"
"   synchronization to its initial time-source neighbor using that cell\n"
"   (\\\"Timekeeping\\\" bit is set).\n"
"\n"
"   This translates into a Link Option for this cell:\n"
"\n"
"      b0 = TX Link = 1 (set)\n"
"      b1 = RX Link = 1 (set)\n"
"      b2 = Shared Link = 1 (set)\n"
"      b3 = Timekeeping = 1 (set)\n"
"      b4 = Priority = 0 (clear)\n"
"      b5-b7 = Reserved = 0 (clear)\n"
"\n"
"4.3.  Retransmissions\n"
"\n"
"   Per Figure 1, the RECOMMENDED maximum number of link-layer\n"
"   retransmissions is 3.  This means that, for packets requiring an\n"
"   acknowledgment, if none are received after a total of 4 attempts, the\n"
"   transmission is considered failed and the link layer MUST notify the\n"
"   upper layer.  Packets not requiring an acknowledgment (including EBs)\n"
"   are not retransmitted.\n"
"\n"
"4.4.  Timeslot Timing\n"
"\n"
"   Per Figure 1, the RECOMMENDED timeslot template is the default one\n"
"   (macTimeslotTemplateId=0) defined in [IEEE.802.15.4].\n"
"\n"
"4.5.  Frame Contents\n"
"\n"
"   [IEEE.802.15.4] defines the format of frames.  Through a set of\n"
"   flags, [IEEE.802.15.4] allows for several fields to be present (or\n"
"   not), to have different lengths, and to have different values.  This\n"
"   specification details the RECOMMENDED contents of 802.15.4 frames,\n"
"   while strictly complying with [IEEE.802.15.4].\n"
"\n"
"4.5.1.  IEEE Std 802.15.4 Header\n"
"\n"
"   The Frame Version field MUST be set to 0b10 (Frame Version 2).  The\n"
"   Sequence Number field MAY be elided.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 8]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   The EB Destination Address field MUST be set to 0xFFFF (short\n"
"   broadcast address).  The EB Source Address field SHOULD be set as the\n"
"   node's short address if this is supported.  Otherwise, the long\n"
"   address MUST be used.\n"
"\n"
"   The PAN ID Compression bit SHOULD indicate that the Source PAN ID is\n"
"   \\\"Not Present\\\" and the Destination PAN ID is \\\"Present\\\".  The value of\n"
"   the PAN ID Compression bit is specified in Table 7-2 of the IEEE Std\n"
"   802.15.4-2015 specification and depends on the type of the\n"
"   destination and source link-layer addresses (e.g., short, extended,\n"
"   not present).\n"
"\n"
"   Nodes follow the reception and rejection rules as per Section 6.7.2\n"
"   of [IEEE.802.15.4].\n"
"\n"
"   The nonce is formatted according to [IEEE.802.15.4].  In the IEEE Std\n"
"   802.15.4 specification [IEEE.802.15.4], nonce generation is described\n"
"   in Section 9.3.2.2, and byte ordering is described in Section 9.3.1,\n"
"   Annex B.2, and Annex B.2.2.\n"
"\n"
"4.5.2.  Enhanced Beacon Frame\n"
"\n"
"   After booting, a TSCH node starts in an unsynchronized, unjoined\n"
"   state.  Initial synchronization is achieved by listening for EBs.\n"
"   EBs from multiple networks may be heard.  Many mechanisms exist for\n"
"   discrimination between networks, the details of which are out of\n"
"   scope.\n"
"\n"
"   The IEEE Std 802.15.4 specification does not define how often EBs are\n"
"   sent, nor their contents [IEEE.802.15.4].  In a minimal TSCH\n"
"   configuration, a node SHOULD send an EB every EB_PERIOD.  Tuning\n"
"   EB_PERIOD allows a trade-off between joining time and energy\n"
"   consumption.\n"
"\n"
"   EBs should be used to obtain information about local networks and to\n"
"   synchronize ASN and time offset of the specific network that the node\n"
"   decides to join.  Once joined to a particular network, a node MAY\n"
"   choose to continue to listen for EBs, to gather more information\n"
"   about other networks, for example.  During the joining process,\n"
"   before secure connections to time parents have been created, a node\n"
"   MAY maintain synchronization using EBs.  [RFC7554] discusses\n"
"   different time synchronization approaches.\n"
"\n"
"   The IEEE Std 802.15.4 specification requires EBs to be sent in order\n"
"   to enable nodes to join the network.  The EBs SHOULD carry the\n"
"   Information Elements (IEs) listed below [IEEE.802.15.4].\n"
"\n"
"Vilajosana, et al.        Best Current Practice                 [Page 9]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   TSCH Synchronization IE:  Contains synchronization information such\n"
"      as ASN and Join Metric.  The value of the Join Metric field is\n"
"      discussed in Section 6.1.\n"
"\n"
"   TSCH Timeslot IE:  Contains the timeslot template identifier.  This\n"
"      template is used to specify the internal timing of the timeslot.\n"
"      This specification RECOMMENDS the default timeslot template.\n"
"\n"
"   Channel Hopping IE:  Contains the channel hopping sequence\n"
"      identifier.  This specification RECOMMENDS the default channel\n"
"      hopping sequence.\n"
"\n"
"   TSCH Slotframe and Link IE:  Enables joining nodes to learn the\n"
"      initial schedule to be used as they join the network.  This\n"
"      document RECOMMENDS the use of a single cell.\n"
"\n"
"   If a node strictly follows the recommended setting from Figure 1, the\n"
"   EB it sends has the exact same contents as an EB it received when\n"
"   joining, except for the Join Metric field in the TSCH Synchronization\n"
"   IE.\n"
"\n"
"   When a node has already joined a network (i.e., it has received an\n"
"   EB) synchronized to the EB sender and configured its schedule\n"
"   following this specification, the node SHOULD ignore subsequent EBs\n"
"   that try to change the configured parameters.  This does not preclude\n"
"   listening to EBs from other networks.\n"
"\n"
"4.5.3.  Acknowledgment Frame\n"
"\n"
"   Per [IEEE.802.15.4], each acknowledgment contains an ACK/NACK Time\n"
"   Correction IE.\n"
"\n"
"4.6.  Link-Layer Security\n"
"\n"
"   When securing link-layer frames, link-layer frames MUST be secured by\n"
"   the link-layer security mechanisms defined in IEEE Std 802.15.4\n"
"   [IEEE.802.15.4].  Link-layer authentication MUST be applied to the\n"
"   entire frame, including the 802.15.4 header.  Link-layer encryption\n"
"   MAY be applied to 802.15.4 Payload IEs and the 802.15.4 payload.\n"
"\n"
"   This specification assumes the existence of two cryptographic keys:\n"
"\n"
"      Key K1 is used to authenticate EBs.  EBs MUST be authenticated\n"
"      only (no encryption); their contents are defined in Section 4.5.2.\n"
"\n"
"      Key K2 is used to authenticate and encrypt DATA and ACKNOWLEDGMENT\n"
"      frames.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 10]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   These keys can be pre-configured or learned during a key distribution\n"
"   phase.  Key distribution mechanisms are defined, for example, in\n"
"   [SEC-6TISCH] and [SEC-JOIN-6TISCH].  Key distribution is out of scope\n"
"   of this document.\n"
"\n"
"   The behavior of a Joining Node (JN) is different depending on which\n"
"   key(s) are pre-configured:\n"
"\n"
"      If both keys K1 and K2 are pre-configured, the JN does not rely on\n"
"      a key distribution phase to learn K1 or K2.\n"
"\n"
"      If key K1 is pre-configured but not key K2, the JN authenticates\n"
"      EBs using K1 and relies on the key distribution phase to learn K2.\n"
"\n"
"      If neither key K1 nor key K2 is pre-configured, the JN accepts EBs\n"
"      as defined in Section 6.3.1.2 of IEEE Std 802.15.4\n"
"      [IEEE.802.15.4], i.e., they are passed forward even \\\"if the status\n"
"      of the unsecuring process indicated an error\\\".  The JN then runs\n"
"      the key distribution phase to learn K1 and K2.  During that\n"
"      process, the node that JN is talking to uses the secExempt\n"
"      mechanism (see Section 9.2.4 of [IEEE.802.15.4]) to process frames\n"
"      from JN.  Once the key distribution phase is done, the node that\n"
"      has installed secExempts for the JN MUST clear the installed\n"
"      exception rules.\n"
"\n"
"   In the event of a network reset, the new network MUST either use new\n"
"   cryptographic keys or ensure that the ASN remains monotonically\n"
"   increasing.\n"
"\n"
"5.  RPL Settings\n"
"\n"
"   In a multi-hop topology, the RPL routing protocol [RFC6550] MAY be\n"
"   used.\n"
"\n"
"5.1.  Objective Function\n"
"\n"
"   If RPL is used, nodes MUST implement the RPL Objective Function Zero\n"
"   (OF0) [RFC6552].\n"
"\n"
"5.1.1.  Rank Computation\n"
"\n"
"   The Rank computation is described in Section 4.1 of [RFC6552].  A\n"
"   node's Rank (see Figure 4 for an example) is computed by the\n"
"   following equations:\n"
"\n"
"      R(N) = R(P) + rank_increment\n"
"\n"
"      rank_increment = (Rf*Sp + Sr) * MinHopRankIncrease\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 11]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   Figure 3 lists the OF0 parameter values that MUST be used if RPL is\n"
"   used.\n"
"\n"
"       +----------------------+-------------------------------------+\n"
"       |    OF0 Parameters    |              Value                  |\n"
"       +----------------------+-------------------------------------+\n"
"       | Rf                   |                                   1 |\n"
"       +----------------------+-------------------------------------+\n"
"       | Sp                   |                           (3*ETX)-2 |\n"
"       +----------------------+-------------------------------------+\n"
"       | Sr                   |                                   0 |\n"
"       +----------------------+-------------------------------------+\n"
"       | MinHopRankIncrease   | DEFAULT_MIN_HOP_RANK_INCREASE (256) |\n"
"       +----------------------+-------------------------------------+\n"
"       | MINIMUM_STEP_OF_RANK |                                   1 |\n"
"       +----------------------+-------------------------------------+\n"
"       | MAXIMUM_STEP_OF_RANK |                                   9 |\n"
"       +----------------------+-------------------------------------+\n"
"       | ETX limit to select  |                                   3 |\n"
"       | a parent             |                                     |\n"
"       +----------------------+-------------------------------------+\n"
"\n"
"                         Figure 3: OF0 Parameters\n"
"\n"
"   The step_of_rank (Sp) uses the Expected Transmission Count (ETX)\n"
"   [RFC6551].\n"
"\n"
"   An implementation MUST follow OF0's normalization guidance as\n"
"   discussed in Sections 1 and 4.1 of [RFC6552].  Sp SHOULD be\n"
"   calculated as (3*ETX)-2.  The minimum value of Sp\n"
"   (MINIMUM_STEP_OF_RANK) indicates a good quality link.  The maximum\n"
"   value of Sp (MAXIMUM_STEP_OF_RANK) indicates a poor quality link.\n"
"   The default value of Sp (DEFAULT_STEP_OF_RANK) indicates an average\n"
"   quality link.  Candidate parents with ETX greater than 3 SHOULD NOT\n"
"   be selected.  This avoids having ETX values on used links that are\n"
"   larger that the maximum allowed transmission attempts.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 12]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"5.1.2.  Rank Computation Example\n"
"\n"
"   This section illustrates the use of OF0 (see Figure 4).  We have:\n"
"\n"
"      rank_increment = ((3*numTx/numTxAck)-2)*minHopRankIncrease = 512\n"
"\n"
"       +-------+\n"
"       |   0   | R(minHopRankIncrease) = 256\n"
"       |       | DAGRank(R(0)) = 1\n"
"       +-------+\n"
"           |\n"
"           |\n"
"       +-------+\n"
"       |   1   | R(1)=R(0) + 512 = 768\n"
"       |       | DAGRank(R(1)) = 3\n"
"       +-------+\n"
"           |\n"
"           |\n"
"       +-------+\n"
"       |   2   | R(2)=R(1) + 512 = 1280\n"
"       |       | DAGRank(R(2)) = 5\n"
"       +-------+\n"
"           |\n"
"           |\n"
"       +-------+\n"
"       |   3   | R(3)=R(2) + 512 = 1792\n"
"       |       | DAGRank(R(3)) = 7\n"
"       +-------+\n"
"           |\n"
"           |\n"
"       +-------+\n"
"       |   4   | R(4)=R(3) + 512 = 2304\n"
"       |       | DAGRank(R(4)) = 9\n"
"       +-------+\n"
"           |\n"
"           |\n"
"       +-------+\n"
"       |   5   | R(5)=R(4) + 512 = 2816\n"
"       |       | DAGRank(R(5)) = 11\n"
"       +-------+\n"
"\n"
"       Figure 4: Rank computation example for a 5-hop network where\n"
"                 numTx=100 and numTxAck=75 for all links.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 13]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"5.2.  Mode of Operation\n"
"\n"
"   When RPL is used, nodes MUST implement the non-storing mode of\n"
"   operation (see Section 9.7 of [RFC6550]).  The storing mode of\n"
"   operation (see Section 9.8 of [RFC6550]) SHOULD be implemented by\n"
"   nodes with enough capabilities.  Nodes not implementing RPL MUST join\n"
"   as leaf nodes.\n"
"\n"
"5.3.  Trickle Timer\n"
"\n"
"   RPL signaling messages such as DODAG Information Objects (DIOs) are\n"
"   sent using the Trickle algorithm (see Section 8.3.1 of [RFC6550] and\n"
"   Section 4.2 of [RFC6206]).  For this specification, the Trickle timer\n"
"   MUST be used with the RPL-defined default values (see Section 8.3.1\n"
"   of [RFC6550]).\n"
"\n"
"5.4.  Packet Contents\n"
"\n"
"   RPL information and hop-by-hop extension headers MUST follow\n"
"   [RFC6553] and [RFC6554].  For cases in which the packets formed at\n"
"   the Low-Power and Lossy Network (LLN) need to cross through\n"
"   intermediate routers, these MUST follow the IP-in-IP encapsulation\n"
"   requirement specified by [RFC6282] and [RFC2460].  Routing extension\n"
"   headers such as RPL Packet Information (RPI) [RFC6550] and Source\n"
"   Routing Header (SRH) [RFC6554], and outer IP headers in case of\n"
"   encapsulation, MUST be compressed according to [RFC8138] and\n"
"   [RFC8025].\n"
"\n"
"6.  Network Formation and Lifetime\n"
"\n"
"6.1.  Value of the Join Metric Field\n"
"\n"
"   The Join Metric of the TSCH Synchronization IE in the EB MUST be\n"
"   calculated based on the routing metric of the node, normalized to a\n"
"   value between 0 and 255.  A lower value of the Join Metric indicates\n"
"   the node sending the EB is topologically \\\"closer\\\" to the root of the\n"
"   network.  A lower value of the Join Metric hence indicates higher\n"
"   preference for a joining node to synchronize to that neighbor.\n"
"\n"
"   In case the network uses RPL, the Join Metric of any node (including\n"
"   the Directed Acyclic Graph (DAG) root) MUST be set to\n"
"   DAGRank(rank)-1.  According to Section 5.1.1, DAGRank(rank(0)) = 1.\n"
"   DAGRank(rank(0))-1 = 0 is compliant with 802.15.4's requirement of\n"
"   having the root use Join Metric = 0.\n"
"\n"
"   In case the network does not use RPL, the Join Metric value MUST\n"
"   follow the rules specified by [IEEE.802.15.4].\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 14]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"6.2.  Time-Source Neighbor Selection\n"
"\n"
"   When a node joins a network, it may hear EBs sent by different nodes\n"
"   already in the network.  The decision of which neighbor to\n"
"   synchronize to (e.g., which neighbor becomes the node's initial time-\n"
"   source neighbor) is implementation specific.  For example, after\n"
"   having received the first EB, a node MAY listen for at most\n"
"   MAX_EB_DELAY seconds until it has received EBs from\n"
"   NUM_NEIGHBOURS_TO_WAIT distinct neighbors.  Recommended values for\n"
"   MAX_EB_DELAY and NUM_NEIGHBOURS_TO_WAIT are defined in Figure 5.\n"
"   When receiving EBs from distinct neighbors, the node MAY use the Join\n"
"   Metric field in each EB to select the initial time-source neighbor,\n"
"   as described in Section 6.3.6 of IEEE Std 802.15.4 [IEEE.802.15.4].\n"
"\n"
"   At any time, a node MUST maintain synchronization to at least one\n"
"   time-source neighbor.  A node's time-source neighbor MUST be chosen\n"
"   among the neighbors in its RPL routing parent set when RPL is used.\n"
"   In the case a node cannot maintain connectivity to at least one time-\n"
"   source neighbor, the node looses synchronization and needs to join\n"
"   the network again.\n"
"\n"
"6.3.  When to Start Sending EBs\n"
"\n"
"   When a RPL node joins the network, it MUST NOT send EBs before having\n"
"   acquired a RPL Rank to avoid inconsistencies in the time\n"
"   synchronization structure.  This applies to other routing protocols\n"
"   with their corresponding routing metrics.  As soon as a node acquires\n"
"   routing information (e.g., a RPL Rank, see Section 5.1.1), it SHOULD\n"
"   start sending EBs.\n"
"\n"
"6.4.  Hysteresis\n"
"\n"
"   Per [RFC6552] and [RFC6719], the specification RECOMMENDS the use of\n"
"   a boundary value (PARENT_SWITCH_THRESHOLD) to avoid constant changes\n"
"   of the parent when ranks are compared.  When evaluating a parent that\n"
"   belongs to a smaller path cost than the current minimum path, the\n"
"   candidate node is selected as the new parent only if the difference\n"
"   between the new path and the current path is greater than the defined\n"
"   PARENT_SWITCH_THRESHOLD.  Otherwise, the node MAY continue to use the\n"
"   current preferred parent.  Per [RFC6719], the PARENT_SWITCH_THRESHOLD\n"
"   SHOULD be set to 192 when the ETX metric is used (in the form\n"
"   128*ETX); the recommendation for this document is to use\n"
"   PARENT_SWITCH_THRESHOLD equal to 640 if the metric being used is\n"
"   ((3*ETX)-2)*minHopRankIncrease or a proportional value.  This deals\n"
"   with hysteresis both for routing parent and time-source neighbor\n"
"   selection.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 15]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"7.  Implementation Recommendations\n"
"\n"
"7.1.  Neighbor Table\n"
"\n"
"   The exact format of the neighbor table is implementation specific.\n"
"   The RECOMMENDED per-neighbor information is (taken from the [openwsn]\n"
"   implementation):\n"
"\n"
"   identifier: Identifier(s) of the neighbor (e.g., EUI-64).\n"
"\n"
"   numTx:      Number of link-layer transmission attempts to that\n"
"               neighbor.\n"
"\n"
"   numTxAck:   Number of transmitted link-layer frames that have been\n"
"               link-layer acknowledged by that neighbor.\n"
"\n"
"   numRx:      Number of link-layer frames received from that neighbor.\n"
"\n"
"   timestamp:  When the last frame was received from that neighbor.\n"
"               This can be based on the ASN counter or any other time\n"
"               base.  It can be used to trigger a keep-alive message.\n"
"\n"
"   routing metric:  The RPL Rank of that neighbor, for example.\n"
"\n"
"   time-source neighbor:  A flag indicating whether this neighbor is a\n"
"               time-source neighbor.\n"
"\n"
"7.2.  Queues and Priorities\n"
"\n"
"   The IEEE Std 802.15.4 specification [IEEE.802.15.4] does not define\n"
"   the use of queues to handle upper-layer data (either application or\n"
"   control data from upper layers).  The following rules are\n"
"   RECOMMENDED:\n"
"\n"
"      A node is configured to keep in the queues a configurable number\n"
"      of upper-layer packets per link (default NUM_UPPERLAYER_PACKETS)\n"
"      for a configurable time that should cover the join process\n"
"      (default MAX_JOIN_TIME).\n"
"\n"
"      Frames generated by the 802.15.4 layer (including EBs) are queued\n"
"      with a priority higher than frames coming from higher layers.\n"
"\n"
"      A frame type BEACON is queued with higher priority than frame\n"
"      types DATA.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 16]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"7.3.  Recommended Settings\n"
"\n"
"   Figure 5 lists RECOMMENDED values for the settings discussed in this\n"
"   specification.\n"
"\n"
"           +-------------------------+-------------------+\n"
"           | Parameter               | RECOMMENDED Value |\n"
"           +-------------------------+-------------------+\n"
"           | MAX_EB_DELAY            |               180 |\n"
"           +-------------------------+-------------------+\n"
"           | NUM_NEIGHBOURS_TO_WAIT  |                 2 |\n"
"           +-------------------------+-------------------+\n"
"           | PARENT_SWITCH_THRESHOLD |               640 |\n"
"           +-------------------------+-------------------+\n"
"           | NUM_UPPERLAYER_PACKETS  |                 1 |\n"
"           +-------------------------+-------------------+\n"
"           | MAX_JOIN_TIME           |               300 |\n"
"           +-------------------------+-------------------+\n"
"\n"
"                      Figure 5: Recommended Settings\n"
"\n"
"8.  Security Considerations\n"
"\n"
"   This document is concerned only with link-layer security.\n"
"\n"
"   By their nature, many Internet of Things (IoT) networks have nodes in\n"
"   physically vulnerable locations.  We should assume that nodes will be\n"
"   physically compromised, their memories examined, and their keys\n"
"   extracted.  Fixed secrets will not remain secret.  This impacts the\n"
"   node-joining process.  Provisioning a network with a fixed link key\n"
"   K2 is not secure.  For most applications, this implies that there\n"
"   will be a joining phase during which some level of authorization will\n"
"   be allowed for nodes that have not been authenticated.  Details are\n"
"   out of scope, but the link layer must provide some flexibility here.\n"
"\n"
"   If an attacker has obtained K1, it can generate fake EBs to attack a\n"
"   whole network by sending authenticated EBs.  The attacker can cause\n"
"   the joining node to initiate the joining process to the attacker.  In\n"
"   the case that the joining process includes authentication and\n"
"   distribution of a K2, then the joining process will fail and the JN\n"
"   will notice the attack.  If K2 is also compromised, the JN will not\n"
"   notice the attack and the network will be compromised.\n"
"\n"
"   Even if an attacker does not know the value of K1 and K2\n"
"   (Section 4.6), it can still generate fake EB frames authenticated\n"
"   with an arbitrary key.  Here we discuss the impact these fake EBs can\n"
"   have, depending on what key(s) are pre-provisioned.\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 17]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"      If both K1 and K2 are pre-provisioned; a joining node can\n"
"      distinguish legitimate from fake EBs and join the legitimate\n"
"      network.  The fake EBs have no impact.\n"
"\n"
"      The same holds if K1 is pre-provisioned but not K2.\n"
"\n"
"      If neither K1 nor K2 is pre-provisioned, a joining node may\n"
"      mistake a fake EB for a legitimate one and initiate a joining\n"
"      process to the attacker.  That joining process will fail, as the\n"
"      joining node will not be able to authenticate the attacker during\n"
"      the security handshake.  This will force the joining node to start\n"
"      over listening for an EB.  So while the joining node never joins\n"
"      the attacker, this costs the joining node time and energy and is a\n"
"      vector of attack.\n"
"\n"
"   Choosing what key(s) to pre-provision needs to balance the different\n"
"   discussions above.\n"
"\n"
"   Once the joining process is over, the node that has joined can\n"
"   authenticate EBs (it knows K1).  This means it can process their\n"
"   contents and use EBs for synchronization.\n"
"\n"
"   ASN provides a nonce for security operations in a slot.  Any re-use\n"
"   of ASN with a given key exposes information about encrypted packet\n"
"   contents and risks replay attacks.  Replay attacks are prevented\n"
"   because, when the network resets, either the new network uses new\n"
"   cryptographic key(s) or ensures that the ASN increases monotonically\n"
"   (Section 4.6).\n"
"\n"
"   Maintaining accurate time synchronization is critical for network\n"
"   operation.  Accepting timing information from unsecured sources MUST\n"
"   be avoided during normal network operation, as described in\n"
"   Section 4.5.2.  During joining, a node may be susceptible to timing\n"
"   attacks before key K1 and K2 are learned.  During network operation,\n"
"   a node MAY maintain statistics on time updates from neighbors and\n"
"   monitor for anomalies.\n"
"\n"
"   Denial-of-Service (DoS) attacks at the Media Access Control (MAC)\n"
"   layer in an LLN are easy to achieve simply by Radio Frequency (RF)\n"
"   jamming.  This is the base case against which more sophisticated DoS\n"
"   attacks should be judged.  For example, sending fake EBs announcing a\n"
"   very low Join Metric may cause a node to waste time and energy trying\n"
"   to join a fake network even when legitimate EBs are being heard.\n"
"   Proper join security will prevent the node from joining the false\n"
"   flag, but by then the time and energy will have been wasted.\n"
"   However, the energy cost to the attacker would be lower and the\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 18]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   energy cost to the joining node would be higher if the attacker\n"
"   simply sent loud short packets in the middle of any valid EB that it\n"
"   hears.\n"
"\n"
"   ACK reception probability is less than 100% due to changing channel\n"
"   conditions and unintentional or intentional jamming.  This will cause\n"
"   the sending node to retransmit the same packet until it is\n"
"   acknowledged or a retransmission limit is reached.  Upper-layer\n"
"   protocols should take this into account, possibly using a sequence\n"
"   number to match retransmissions.\n"
"\n"
"   The 6TiSCH layer SHOULD keep track of anomalous events and report\n"
"   them to a higher authority.  For example, EBs reporting low Join\n"
"   Metrics for networks that cannot be joined, as described above, may\n"
"   be a sign of attack.  Additionally, in normal network operation,\n"
"   message integrity check failures on packets with a valid Cyclic\n"
"   Redundancy Check (CRC) will occur at a rate on the order of once per\n"
"   million packets.  Any significant deviation from this rate may be a\n"
"   sign of a network attack.  Along the same lines, time updates in ACKs\n"
"   or EBs that are inconsistent with the MAC-layer's sense of time and\n"
"   its own plausible time-error drift rate may also be a result of\n"
"   network attack.\n"
"\n"
"9.  IANA Considerations\n"
"\n"
"   This document does not require any IANA actions.\n"
"\n"
"Appendix A.  Examples\n"
"\n"
"   This section contains several example packets.  Each example contains\n"
"   (1) a schematic header diagram, (2) the corresponding bytestream, and\n"
"   (3) a description of each of the IEs that form the packet.  Packet\n"
"   formats are specific for the [IEEE.802.15.4] revision and may vary in\n"
"   future releases of the IEEE standard.  In case of differences between\n"
"   the packet content presented in this section and [IEEE.802.15.4], the\n"
"   latter has precedence.\n"
"\n"
"   The MAC header fields are described in a specific order.  All field\n"
"   formats in this example are depicted in the order in which they are\n"
"   transmitted, from left to right, where the leftmost bit is\n"
"   transmitted first.  Bits within each field are numbered from 0\n"
"   (leftmost and least significant) to k - 1 (rightmost and most\n"
"   significant), where the length of the field is k bits.  Fields that\n"
"   are longer than a single octet are sent to the PHY in the order from\n"
"   the octet containing the lowest numbered bits to the octet containing\n"
"   the highest numbered bits (little endian).\n"
"\n"
"A.1.  Example: EB with Default Timeslot Template\n"
"\n"
"                        1                   2                   3\n"
"    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   | Len1 =   0  |Element ID=0x7e|0|    Len2 = 26        |GrpId=1|1|\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   | Len3 =   6    |Sub ID = 0x1a|0|           ASN\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"                ASN                                | Join Metric   |\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   |  Len4 = 0x01  |Sub ID = 0x1c|0| TT ID = 0x00  |   Len5 = 0x01\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"         |ID=0x9 |1| CH ID = 0x00  | Len6 = 0x0A   |Sub ID = 0x1b|0|\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   |   #SF = 0x01  | SF ID = 0x00  |   SF LEN = 0x65 (101 slots)   |\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   | #Links = 0x01 |      SLOT OFFSET = 0x0000     |    CHANNEL\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"    OFF  = 0x0000  |Link OPT = 0x0F|         NO MAC PAYLOAD\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"\n"
"   Bytestream:\n"
"\n"
"       00 3F 1A 88 06 1A ASN#0 ASN#1 ASN#2 ASN#3 ASN#4 JP 01 1C 00\n"
"       01 C8 00 0A 1B 01 00 65 00 01 00 00 00 00 0F\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 23]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"   Description of the IEs:\n"
"\n"
"       #Header IE Header\n"
"           Len1 = Header IE Length (0)\n"
"           Element ID = 0x7e - termination IE indicating Payload IE\n"
"               coming next\n"
"           Type 0\n"
"\n"
"       #Payload IE Header (MLME)\n"
"           Len2 = Payload IE Len (26 bytes)\n"
"           Group ID = 1 MLME (Nested)\n"
"           Type = 1\n"
"\n"
"       #MLME-SubIE TSCH Synchronization\n"
"           Len3 = Length in bytes of the sub-IE payload (6 bytes)\n"
"           Sub-ID = 0x1a (MLME-SubIE TSCH Synchronization)\n"
"           Type = Short (0)\n"
"           ASN  = Absolute Sequence Number (5 bytes)\n"
"           Join Metric = 1 byte\n"
"\n"
"       #MLME-SubIE TSCH Timeslot\n"
"           Len4 = Length in bytes of the sub-IE payload (1 byte)\n"
"           Sub-ID = 0x1c (MLME-SubIE Timeslot)\n"
"           Type = Short (0)\n"
"           Timeslot template ID = 0x00 (default)\n"
"\n"
"       #MLME-SubIE Channel Hopping\n"
"           Len5 = Length in bytes of the sub-IE payload (1 byte)\n"
"           Sub-ID = 0x09 (MLME-SubIE Channel Hopping)\n"
"           Type = Long (1)\n"
"           Hopping Sequence ID = 0x00 (default)\n"
"\n"
"       #MLME-SubIE TSCH Slotframe and Link\n"
"           Len6 = Length in bytes of the sub-IE payload (10 bytes)\n"
"           Sub-ID = 0x1b (MLME-SubIE TSCH Slotframe and Link)\n"
"           Type = Short (0)\n"
"           Number of slotframes = 0x01\n"
"           Slotframe handle = 0x00\n"
"           Slotframe size = 101 slots (0x65)\n"
"           Number of Links (Cells) = 0x01\n"
"           Timeslot = 0x0000 (2B)\n"
"           Channel Offset = 0x0000 (2B)\n"
"           Link Options = 0x0F\n"
"           (TX Link = 1, RX Link = 1, Shared Link = 1,\n"
"            Timekeeping = 1 )\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 24]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"A.2.  Example: EB with Custom Timeslot Template\n"
"\n"
"   Using a custom timeslot template in EBs: setting timeslot length to\n"
"   15 ms.\n"
"\n"
"                        1                   2                   3\n"
"    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   | Len1 =   0  |Element ID=0x7e|0|    Len2 = 53        |GrpId=1|1|\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   | Len3 =   6    |Sub ID = 0x1a|0|           ASN\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"                ASN                                | Join Metric   |\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   |  Len4 = 25    |Sub ID = 0x1c|0| TT ID = 0x01  | macTsCCAOffset\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"     = 2700        |  macTsCCA = 128               | macTsTxOffset\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"     = 3180        |  macTsRxOffset = 1680         | macTsRxAckDelay\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"     = 1200        |  macTsTxAckDelay = 1500       | macTsRxWait\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"     = 3300        |  macTsAckWait = 600           | macTsRxTx\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"     = 192         |  macTsMaxAck  = 2400          | macTsMaxTx\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"     = 4256        | macTsTimeslotLength = 15000   | Len5 = 0x01\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"         |ID=0x9 |1| CH ID = 0x00  | Len6 = 0x0A   | ...\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"\n"
"   Bytestream:\n"
"\n"
"   00 3F 1A 88 06 1A ASN#0 ASN#1 ASN#2 ASN#3 ASN#4 JP 19 1C 01 8C 0A 80\n"
"   00 6C 0C 90 06 B0 04 DC 05 E4 0C 58 02 C0 00 60 09 A0 10 98 3A 01 C8\n"
"   00 0A ...\n"
"\n"
"   Description of the IEs:\n"
"\n"
"       #Header IE Header\n"
"           Len1 = Header IE Length (none)\n"
"           Element ID = 0x7e - termination IE indicating Payload IE\n"
"               coming next\n"
"           Type 0\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 25]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"       #Payload IE Header (MLME)\n"
"           Len2 = Payload IE Len (53 bytes)\n"
"           Group ID = 1 MLME (Nested)\n"
"           Type = 1\n"
"\n"
"       #MLME-SubIE TSCH Synchronization\n"
"           Len3 = Length in bytes of the sub-IE payload (6 bytes)\n"
"           Sub-ID = 0x1a (MLME-SubIE TSCH Synchronization)\n"
"           Type = Short (0)\n"
"           ASN  = Absolute Sequence Number (5 bytes)\n"
"           Join Metric = 1 byte\n"
"\n"
"       #MLME-SubIE TSCH Timeslot\n"
"           Len4 = Length in bytes of the sub-IE payload (25 bytes)\n"
"           Sub-ID = 0x1c (MLME-SubIE Timeslot)\n"
"           Type = Short (0)\n"
"           Timeslot template ID = 0x01 (non-default)\n"
"\n"
"           The 15 ms timeslot announced:\n"
"           +--------------------------------+------------+\n"
"           | IEEE 802.15.4 TSCH parameter   | Value (us) |\n"
"           +--------------------------------+------------+\n"
"           | macTsCCAOffset                 |       2700 |\n"
"           +--------------------------------+------------+\n"
"           | macTsCCA                       |        128 |\n"
"           +--------------------------------+------------+\n"
"           | macTsTxOffset                  |       3180 |\n"
"           +--------------------------------+------------+\n"
"           | macTsRxOffset                  |       1680 |\n"
"           +--------------------------------+------------+\n"
"           | macTsRxAckDelay                |       1200 |\n"
"           +--------------------------------+------------+\n"
"           | macTsTxAckDelay                |       1500 |\n"
"           +--------------------------------+------------+\n"
"           | macTsRxWait                    |       3300 |\n"
"           +--------------------------------+------------+\n"
"           | macTsAckWait                   |        600 |\n"
"           +--------------------------------+------------+\n"
"           | macTsRxTx                      |        192 |\n"
"           +--------------------------------+------------+\n"
"           | macTsMaxAck                    |       2400 |\n"
"           +--------------------------------+------------+\n"
"           | macTsMaxTx                     |       4256 |\n"
"           +--------------------------------+------------+\n"
"           | macTsTimeslotLength            |      15000 |\n"
"           +--------------------------------+------------+\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 26]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"       #MLME-SubIE Channel Hopping\n"
"           Len5 = Length in bytes of the sub-IE payload. (1 byte)\n"
"           Sub-ID = 0x09 (MLME-SubIE Channel Hopping)\n"
"           Type = Long (1)\n"
"           Hopping Sequence ID = 0x00 (default)\n"
"\n"
"A.3.  Example: Link-layer Acknowledgment\n"
"\n"
"   Enhanced Acknowledgment packets carry the Time Correction IE (Header\n"
"   IE).\n"
"\n"
"                        1                   2                   3\n"
"    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   | Len1 =   2  |Element ID=0x1e|0|        Time Sync Info         |\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"\n"
"   Bytestream:\n"
"\n"
"       02 0F TS#0 TS#1\n"
"\n"
"   Description of the IEs:\n"
"\n"
"       #Header IE Header\n"
"           Len1 = Header IE Length (2 bytes)\n"
"           Element ID = 0x1e - ACK/NACK Time Correction IE\n"
"           Type 0\n"
"\n"
"A.4.  Example: Auxiliary Security Header\n"
"\n"
"   802.15.4 Auxiliary Security Header with the Security Level set to\n"
"   ENC-MIC-32.\n"
"\n"
"                        1\n"
"    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"   |L = 5|M=1|1|1|0|Key Index = IDX|\n"
"   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n"
"\n"
"   Bytestream:\n"
"\n"
"       6D IDX#0\n"
"\n"
"   Security Auxiliary Header fields in the example:\n"
"\n"
"       #Security Control (1 byte)\n"
"           L = Security Level ENC-MIC-32 (5)\n"
"           M = Key Identifier Mode (0x01)\n"
"\n"
"Vilajosana, et al.        Best Current Practice                [Page 27]\n"
"RFC 8180                     6TiSCH Minimal                     May 2017\n"
"\n"
"           Frame Counter Suppression = 1 (omitting Frame Counter field)\n"
"           ASN in Nonce = 1 (construct Nonce from 5 byte ASN)\n"
"           Reserved = 0\n"
"\n"
"       #Key Identifier (1 byte)\n"
"           Key Index = IDX (deployment-specific KeyIndex parameter that\n"
"                       identifies the cryptographic key)\n"
"\n"
"Acknowledgments\n"
"\n"
"   The authors acknowledge the guidance and input from Rene Struik, Pat\n"
"   Kinney, Michael Richardson, Tero Kivinen, Nicola Accettura, Malisa\n"
"   Vucinic, and Jonathan Simon.  Thanks to Charles Perkins, Brian E.\n"
"   Carpenter, Ralph Droms, Warren Kumari, Mirja Kuehlewind, Ben\n"
"   Campbell, Benoit Claise, and Suresh Krishnan for the exhaustive and\n"
"   detailed reviews.  Thanks to Simon Duquennoy, Guillaume Gaillard,\n"
"   Tengfei Chang, and Jonathan Munoz for the detailed review of the\n"
"   examples section.  Thanks to 6TiSCH co-chair Pascal Thubert for his\n"
"   guidance and advice.\n"
"\n"
"Authors' Addresses\n"
"\n"
"   Xavier Vilajosana (editor)\n"
"   Universitat Oberta de Catalunya\n"
"   156 Rambla Poblenou\n"
"   Barcelona, Catalonia  08018\n"
"   Spain\n"
"\n"
"   Email: xvilajosana@uoc.edu\n"
"\n"
"   Kris Pister\n"
"   University of California Berkeley\n"
"   512 Cory Hall\n"
"   Berkeley, California  94720\n"
"   United States of America\n"
"\n"
"   Email: pister@eecs.berkeley.edu\n"
"\n"
"   Thomas Watteyne\n"
"   Analog Devices\n"
"   32990 Alvarado-Niles Road, Suite 910\n"
"   Union City, CA  94587\n"
"   United States of America\n"
"\n"
"   Email: twatteyne@linear.com\n";
#endif