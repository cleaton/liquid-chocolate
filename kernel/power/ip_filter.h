
#ifndef __IP_FILTER_H__
#define __IP_FILTER_H__

#include <mach/msm_rpcrouter.h>

#define OEM_RAPIPROG					0x3000006b
#define OEM_RAPIVERS					0x00010001

#define ONCRPC_ACER_ADD_IP_FILTER_PROC		51
#define ONCRPC_ACER_DEL_IP_FILTER_PROC		52
#define ONCRPC_ACER_UPDATE_IP_FILTER_PROC	53
#define ONCRPC_ACER_ENABLE_IP_FILTER_PROC	54
#define ONCRPC_ACER_DISABLE_IP_FILTER_PROC	55


typedef enum _acer_pktfltr_ip_mask {
	ACER_PKT_IP_HDR		= 0x0001,
	ACER_PKT_IP_SRC		= 0x0002,
	ACER_PKT_IP_DST		= 0x0004,
	ACER_PKT_NEXT_PROT	= 0x0008,
} acer_pktfltr_ip_mask;

typedef enum _acer_pktfltr_port_mask {
	ACER_PKT_SRC_PORT	= 0x0010,
	ACER_PKT_DST_PORT	= 0x0020,
} acer_pktfltr_port_mask;

#define MASK_DST_IP		(ACER_PKT_IP_HDR | ACER_PKT_IP_DST)
#define MASK_SRC_IP		(ACER_PKT_IP_HDR | ACER_PKT_IP_SRC)
#define MASK_DST_PORT	(ACER_PKT_IP_HDR | ACER_PKT_NEXT_PROT | ACER_PKT_DST_PORT)
#define MASK_SRC_PORT	(ACER_PKT_IP_HDR | ACER_PKT_NEXT_PROT | ACER_PKT_SRC_PORT)
#define MASK_NEXT_PROT	(ACER_PKT_IP_HDR | ACER_PKT_NEXT_PROT)


#define MAX_ITEM_COUNT	100

#pragma pack(1)

typedef struct port_type{
	unsigned short mask;
	unsigned char payload[5];
} port_type;

typedef struct ip_type{
	unsigned short mask;
	unsigned char payload[8];
} ip_type;

#pragma pack()

typedef struct ipfilter_request_packet_port{
	struct rpc_request_hdr header;
	unsigned int length;
	port_type item[MAX_ITEM_COUNT];
	int item_count;
} ipfilter_request_packet_port;


typedef struct ipfilter_request_packet_ip{
	struct rpc_request_hdr header;
	unsigned int length;
	ip_type item[MAX_ITEM_COUNT];
	int item_count;
} ipfilter_request_packet_ip;


#define PACKET_TYPE_PORT	1
#define PACKET_TYPE_IP		2
#define PACKET_TYPE_0		0

typedef struct ipfilter_reply_packet{
	struct rpc_reply_hdr header;
	int reply;
} ipfilter_reply_packet;

typedef struct packet_item{
	unsigned char protocol;
	unsigned int ip_local;
	unsigned int port_local;
	unsigned int ip_foreign;
	unsigned int port_foreign;
	unsigned char state;
} packet_item;

#define MAX_ALLOW_TABLE_SIZE	128

#define PROT_TCP	6
#define PROT_UDP	17

int save_ip_list(void);
int enable_ipfilter(void);
int disable_ipfilter(void);

#define SEND_DST_PORT		1
#define SEND_SRC_IP		0
#define FIRST_PACKET_LOG	1

#endif
