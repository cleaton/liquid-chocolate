
#ifndef __FAST_DORMANCY_H__
#define __FAST_DORMANCY_H__

#include <mach/msm_rpcrouter.h>

#define CMPROG						0x30000000
#define CMVERS						0x00070003
#define ONCRPC_CM_MM_CALL_CMD_PS_SIG_RELEASE_PROC	27
#define MSM_RPC_CLIENT_NULL_CB_ID			0xffffffff
#define QCRIL_TOKEN_ID_INTERNAL				0xffffffff
#define REQ_PACKET_CLIENT_ID				0

#pragma pack(1)

typedef struct _dormancy_request{
	struct rpc_request_hdr header;
	unsigned int cb_id;
	unsigned int handle;
	int client_id;
} dormancy_request;

typedef struct _dormancy_reply{
	struct rpc_reply_hdr header;
	unsigned int reply;
} dormancy_reply;

#pragma pack()

int Fast_Domancy_Enable(void);

#endif
