
#define LORA_MAX_PAYLOAD_SIZE           255
#define NUMBER_OF_NET_IF_DESCRIPTORS    32

#define OL_BORDER_ROUTER_ADDR           0
#define OL_BROADCAST_ADDR               0xFF

#define LINK_ACK_TIMEOUT                1000
#define LINK_RETRIES                    3

typedef struct openlora_t_ {
    uint8_t nwk_id;
    uint8_t node_addr;
    uint8_t mac_seq_number;
    uint8_t pad;
    uint8_t neigh_seq_number[256];
}openlora_t;


typedef struct
{
	uint8_t *puc_link_buffer; 	    /* Pointer to the start of the link frame. */
	uint8_t  data_lenght; 			/* Holds the total frame length */
} net_if_buffer_descriptor_t;

typedef enum __attribute__((packed)) {
    DATA_PACKET = 0,
    //RTS_PACKET,
    //CTS_PACKET,
    ACK_PACKET
}link_packet_type_t;

typedef struct __attribute__((packed)) {
    link_packet_type_t  packet_type;
    uint8_t             network_id;
    uint8_t             seq_number;
    uint8_t             dst_addr;
    uint8_t             src_addr;
    uint8_t             payload_size;
}link_layer_header_t;

typedef struct __attribute__((packed)) {
    uint16_t            crc;
}link_layer_trailer_t;









typedef struct {
    uint16_t        dst_port;
    uint16_t        src_port;
    uint8_t         payload_size;
}transp_layer_t;


void ol_init(uint8_t nwk_id, uint8_t addr);