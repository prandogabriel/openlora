
#define LORA_MAX_PAYLOAD_SIZE           255
#define NUMBER_OF_NET_IF_DESCRIPTORS    32

#define OL_BORDER_ROUTER_ADDR           0
#define OL_BROADCAST_ADDR               0xFF

#define LINK_ACK_TIMEOUT                300
#define LINK_RETRIES                    3

/* The maximum number of retries for the example code. */
#define BACKOFF_RETRY_MAX_ATTEMPTS      LINK_RETRIES + 1
/* The base back-off delay (in milliseconds) for retry configuration in the example. */
#define RETRY_BACKOFF_BASE_MS           ( ((256U*10U*8U)/125U) )
/* The maximum back-off delay (in milliseconds) for between retries in the example. */
#define RETRY_MAX_BACKOFF_DELAY_MS      ( RETRY_BACKOFF_BASE_MS*4U )

typedef struct openlora_t_ {
    uint8_t  nwk_id;
    uint8_t  node_addr;
    uint8_t  mac_seq_number;
    uint8_t  pad;
    uint8_t  neigh_seq_number[256];
    uint32_t backoff_base_ms;
    uint32_t max_backoof_delay_ms;
    uint32_t symbol_time;
}openlora_t;


typedef struct
{
	uint8_t *puc_link_buffer; 	    /* Pointer to the start of the link frame. */
	uint8_t  data_lenght; 			/* Holds the total frame length */
} net_if_buffer_descriptor_t;

typedef enum __attribute__((packed)) {
    DATA_FRAME = 1,
    //RTS_PACKET,
    //CTS_PACKET,
    ACK_FRAME
}link_frame_type_t;

typedef struct __attribute__((packed)) {
    link_frame_type_t   frame_type;
    uint8_t             network_id;
    uint8_t             seq_number;
    uint8_t             dst_addr;
    uint8_t             src_addr;
    uint8_t             payload_size;
}link_layer_header_t;

typedef struct __attribute__((packed)) {
    uint16_t            crc;
}link_layer_trailer_t;

typedef enum __attribute__((packed)) {
    NEIGHBOR_REQ = 1,
    NEIGHBOR_ADV,
    ROUTER_ADV,
    DATA_PACKET,
}network_packet_type_t;

typedef struct __attribute__((packed)) {
    network_packet_type_t   packet_type;
    uint8_t                 hops;
    uint16_t                dst_addr;
    uint16_t                src_addr;
    uint8_t                 payload_size;
}network_layer_header_t;

struct transp_layer_s{
    uint8_t                 dst_port;
    uint8_t                 src_port;
    uint8_t                 dst_addr;
    uint8_t                 src_addr;
    SemaphoreHandle_t       transp_wakeup;
    uint8_t                 payload_size;
    struct transp_layer_s   *next;
    struct transp_layer_s   *previous;
};

typedef struct transp_layer_s transport_layer_t;

typedef enum __attribute__((packed)) {
    TRANSP_DATAGRAM = 1,
    TRANSP_SEGMENT
}transp_protocol_type_t;

typedef struct __attribute__((packed, aligned(1))) {
    uint8_t                 dst_port;
    uint8_t                 src_port;
    transp_protocol_type_t  protocol;
    uint8_t                 payload_size;
}transport_layer_header_t;

void ol_init(uint8_t nwk_id, uint8_t addr);
BaseType_t ol_to_link_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout);
BaseType_t ol_from_link_layer(net_if_buffer_descriptor_t *buffer, TickType_t timeout);
net_if_buffer_descriptor_t *ol_get_net_if_buffer(uint8_t size);
void ol_release_net_if_buffer(net_if_buffer_descriptor_t *buffer);
