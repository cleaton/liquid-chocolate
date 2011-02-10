
#include "ip_filter.h"
#include <net/tcp.h>
#include <net/udp.h>


#define ACER_DBG(fmt, arg...) do {} while (0)

static packet_item packet_allow_table[MAX_ALLOW_TABLE_SIZE];
static int allow_table_count = 0;


int get_udp_table(packet_item *ptable, int *pcount)
{
	int count = 0;

	if (!ptable || !pcount) {
		ACER_DBG("NULL udp_table or count!\n");
		return -1;
	}

	if (*pcount > MAX_ALLOW_TABLE_SIZE) {
		printk(KERN_INFO  "Table Full!!\n");
		return -1;
	}

	count = *pcount;
	printk(KERN_INFO  "Get_UDP_Table!\n");
	{
		unsigned int i;
		struct inet_sock *si = NULL;

		for (i = 0; i < UDP_HTABLE_SIZE; i++) {
			struct sock *sk = NULL;
			struct hlist_nulls_node *node;
			struct udp_hslot *hslot = &udp_table.hash[i];

			spin_lock_bh(&hslot->lock);
			sk_nulls_for_each(sk, node, &hslot->head) {
				if (sk->sk_family == PF_INET)
					si = inet_sk(sk);
				else
					printk(KERN_INFO  "UDP Not Match!!! sk->sk_family = 0x%x, PF_INET=0x%x\n", sk->sk_family, PF_INET);
			}
			spin_unlock_bh(&hslot->lock);
			if (si != NULL) {
				if (count < MAX_ALLOW_TABLE_SIZE) {
					ptable[count].protocol = PROT_UDP;
					ptable[count].ip_local = ntohl(si->saddr);
					ptable[count].port_local = ntohs(si->sport);
					ptable[count].ip_foreign = ntohl(si->daddr);
					ptable[count].port_foreign = ntohs(si->dport);
					ptable[count].state = sk->sk_state;
					count++;
				} else {
					printk(KERN_INFO  "UDP Table Full! Now is %d\n", count);
					break;
				}
			}
			si = NULL;
		}
	}

	*pcount = count;
	printk(KERN_INFO  "Curr Index = %d\n", count);

	return 0;
}


int get_tcp_table(packet_item *ptable, int *pcount)
{
	int count = 0;

	if (!ptable || !pcount) {
		printk(KERN_INFO  "NULL tcp_table or count!\n");
		return -1;
	}

	if (*pcount > MAX_ALLOW_TABLE_SIZE) {
		printk(KERN_INFO  "Table Full!!\n");
		return -1;
	}

	count = *pcount;
	printk(KERN_INFO  "Get_TCP_Table!\n");

	/* TCP - TIME WAIT */
	{
		unsigned int i;
		for (i = 0; i < tcp_hashinfo.ehash_size; i++) {
			struct hlist_nulls_node *node;
			struct inet_timewait_sock *tw = NULL;
			spinlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, i);
			spin_lock_bh(lock);
				inet_twsk_for_each(tw, node,
				   &tcp_hashinfo.ehash[i].twchain)	 {
				if (tw->tw_family != PF_INET)
					continue;
			}
			spin_unlock_bh(lock);
			if (tw != NULL) {
				if (count < MAX_ALLOW_TABLE_SIZE) {
					ptable[count].protocol = PROT_TCP;
					ptable[count].ip_local = ntohl(tw->tw_rcv_saddr);
					ptable[count].port_local = ntohs(tw->tw_sport);
					ptable[count].ip_foreign = ntohl(tw->tw_daddr);
					ptable[count].port_foreign = ntohs(tw->tw_dport);
					ptable[count].state = tw->tw_state;

					count++;
				} else {
					printk(KERN_INFO  "TCP TABLE FULL!!\n");
					break;
				}
			}
			tw = NULL;
		}
	}

	/* TCP - ESTABLISHED (Exclude TIME-WAIT) */
	{
		unsigned int i;
		struct inet_sock *si = NULL;
		for (i = 0; i < tcp_hashinfo.ehash_size; i++) {
			struct hlist_nulls_node *node;

			spinlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, i);
			struct sock *sk = NULL;
			spin_lock_bh(lock);
			sk_nulls_for_each(sk, node, &tcp_hashinfo.ehash[i].chain) {
				if (sk->sk_family == PF_INET)
					si = inet_sk(sk);
			}
			spin_unlock_bh(lock);
			if (si != NULL) {
				if (count < MAX_ALLOW_TABLE_SIZE) {
					ptable[count].protocol = PROT_TCP;
					ptable[count].ip_local = ntohl(si->saddr);
					ptable[count].port_local = ntohs(si->sport);
					ptable[count].ip_foreign = ntohl(si->daddr);
					ptable[count].port_foreign = ntohs(si->dport);
					ptable[count].state = sk->sk_state;
					count++;
				} else {
					printk(KERN_INFO  "TCP Table Full!\n");
					break;
				}
			}
			si = NULL;
		}
	}

	/* TCP - LISTENING ... */
	{
		unsigned int i;
		struct inet_sock *si = NULL;
		for (i = 0; i < INET_LHTABLE_SIZE; i++) {
			struct hlist_nulls_node *node;

			spinlock_t *lock = inet_ehash_lockp(&tcp_hashinfo, i);
			struct sock *sk = NULL;

			sk = sk_nulls_head(&tcp_hashinfo.listening_hash[i].head);

			spin_lock_bh(lock);
			sk_nulls_for_each_from(sk, node) {
				si = inet_sk(sk);
			}
			spin_unlock_bh(lock);
			if (si != NULL) {
				if (count < MAX_ALLOW_TABLE_SIZE) {
					ptable[count].protocol = PROT_TCP;
					ptable[count].ip_local = 0;
					ptable[count].port_local = ntohs(si->sport);
					ptable[count].ip_foreign = 0;
					ptable[count].port_foreign = ntohs(si->dport);
					ptable[count].state = sk->sk_state;
					count++;
				} else {
					printk(KERN_INFO  "TCP Table Full!\n");
					break;
				}
			}
			si = NULL;
		}
	}

	*pcount = count;
	printk(KERN_INFO  "Curr Index = %d\n", count);
	return 0;
}

int clear_remote_table(void)
{
	int rc = 0;
	static struct msm_rpc_endpoint *ipfilter_endpoint = NULL;
	struct rpc_request_hdr req_packet;

	if (!ipfilter_endpoint) {
		ipfilter_endpoint = msm_rpc_connect(OEM_RAPIPROG, OEM_RAPIVERS, 0);
		if (IS_ERR(ipfilter_endpoint)) {
			printk(KERN_INFO  "ipfilter: cannot connect to rpc server!\n");
			ipfilter_endpoint = NULL;
			return -1;
		}
	}

	printk(KERN_INFO  "Clear Remote Table... ");
	memset(&req_packet, 0x00, sizeof(req_packet));

	rc = msm_rpc_call_reply(ipfilter_endpoint, ONCRPC_ACER_DEL_IP_FILTER_PROC, &req_packet, sizeof(req_packet), NULL, 0, -1);
	if (rc < 0) {
		printk(KERN_INFO  "\nipfilter: delete ip filter rpc call error! rc= %d\n", rc);
		return -1;
	}

	printk(KERN_INFO  "OK!\n");
	return 0;
}

int enable_ipfilter(void)
{
	int rc = 0;
	static struct msm_rpc_endpoint *ipfilter_endpoint = NULL;
	struct rpc_request_hdr req_packet;

	if (!ipfilter_endpoint) {
		ipfilter_endpoint = msm_rpc_connect(OEM_RAPIPROG, OEM_RAPIVERS, 0);
		if (IS_ERR(ipfilter_endpoint)) {
			printk(KERN_INFO  "ipfilter: cannot connect to rpc server!\n");
			ipfilter_endpoint = NULL;
			return -1;
		}
	}

	printk(KERN_INFO  "Enable IP_Filter!!\n");
	memset(&req_packet, 0x00, sizeof(req_packet));

	rc = msm_rpc_call_reply(ipfilter_endpoint, ONCRPC_ACER_ENABLE_IP_FILTER_PROC, &req_packet, sizeof(req_packet), NULL, 0, -1);
	if (rc < 0) {
		printk(KERN_INFO  "ipfilter: enable ip filter rpc call error! rc= %d\n", rc);
		return -1;
	}
	printk(KERN_INFO  "ENABLED!\n");
	return 0;
}

int disable_ipfilter(void)
{
	int rc = 0;
	static struct msm_rpc_endpoint *ipfilter_endpoint = NULL;
	struct rpc_request_hdr req_packet;

	if (!ipfilter_endpoint) {
		ipfilter_endpoint = msm_rpc_connect(OEM_RAPIPROG, OEM_RAPIVERS, 0);
		if (IS_ERR(ipfilter_endpoint)) {
			printk(KERN_INFO  "ipfilter: cannot connect to rpc server!\n");
			ipfilter_endpoint = NULL;
			return -1;
		}
	}

	printk(KERN_INFO  "Disable IP_Filter!!\n");
	memset(&req_packet, 0x00, sizeof(req_packet));

	rc = msm_rpc_call_reply(ipfilter_endpoint, ONCRPC_ACER_DISABLE_IP_FILTER_PROC, &req_packet, sizeof(req_packet), NULL, 0, -1);
	if (rc < 0) {
		printk(KERN_INFO  "ipfilter: disable ip filter rpc call error! rc= %d\n", rc);
		return -1;
	}
	printk(KERN_INFO  "DISABLED!\n");
	return 0;
}

int send_item(void *req_packet, unsigned int tp, unsigned int packet_type)
{
	int rc = 0;
	static struct msm_rpc_endpoint *ipfilter_endpoint = NULL;
	static ipfilter_reply_packet rep_packet;

	int packet_size = 0;

	if (!ipfilter_endpoint) {
		ipfilter_endpoint = msm_rpc_connect(OEM_RAPIPROG, OEM_RAPIVERS, 0);
		if (IS_ERR(ipfilter_endpoint)) {
			printk(KERN_INFO  "ipfilter: cannot connect to rpc server!\n");
			ipfilter_endpoint = NULL;
			return -1;
		}
	}

	memset(&rep_packet, 0x00, sizeof(rep_packet));

	printk(KERN_INFO  "Send Item ");
	switch (packet_type) {
	case PACKET_TYPE_PORT:
		printk(KERN_INFO  "Type: PORT..... ");
		packet_size = sizeof(struct rpc_request_hdr) + sizeof(int) + ((ipfilter_request_packet_port *)req_packet)->item_count * sizeof(port_type);
		break;
	case PACKET_TYPE_IP:
		printk(KERN_INFO  "Type: IP..... ");
		packet_size = sizeof(struct rpc_request_hdr) + sizeof(int) + ((ipfilter_request_packet_ip *)req_packet)->item_count * sizeof(ip_type);
		break;
	default:
		printk(KERN_INFO  "\nError!!!Invalid Packet Type!!!\n");
		return -1;
	}
	printk(KERN_INFO  "packet_size = %d\n", packet_size);

	if (packet_size%4 != 0)
		packet_size = packet_size + 4 - packet_size%4;

	rc = msm_rpc_call_reply(ipfilter_endpoint, ONCRPC_ACER_ADD_IP_FILTER_PROC, req_packet, packet_size, &rep_packet, sizeof(rep_packet), -1);

	if (rc < 0) {
		printk(KERN_INFO  "\nipfilter: rpc call error! rc= %d\n", rc);
		return -1;
	}
	printk(KERN_INFO  "OK! Rep=%d\n", rep_packet.reply);
	return rep_packet.reply;
}

int send_table(packet_item *ptable, int count)
{
	int i = 0;
	int ret = 0;
	int index = 0;
	static ipfilter_request_packet_port req_packet_port;
	static ipfilter_request_packet_ip	req_packet_ip;

	printk(KERN_INFO  "Send Allow Table++\n");
	if (!ptable) {
		printk(KERN_INFO  "ptable = NULL!\n");
		return -1;
	}

	memset(&req_packet_port, 0x00, sizeof(req_packet_port));
	memset(&req_packet_ip, 0x00, sizeof(req_packet_ip));

	for (i = 0; i < count; i++) {
		index = 0;
		switch (ptable[i].protocol) {
		case PROT_UDP:
			if (ptable[i].port_local) {
#if SEND_DST_PORT
				printk(KERN_INFO  "Send UDP DST_PORT: port_local = %d\n", ptable[i].port_local);
				req_packet_port.item[req_packet_port.item_count].mask = MASK_DST_PORT;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = PROT_UDP;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = (unsigned char)((ptable[i].port_local >> 8) & 0xFF);
				req_packet_port.item[req_packet_port.item_count].payload[index++] = (unsigned char)((ptable[i].port_local >> 0) & 0xFF);
				req_packet_port.item[req_packet_port.item_count].payload[index++] = 0;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = 0;

				req_packet_port.item_count++;
#else
				printk(KERN_INFO  "Skip UDP DST_PORT: port_local = %d\n", ptable[i].port_local);
#endif
			}
			break;
		case PROT_TCP:
			if (ptable[i].ip_local && ptable[i].ip_foreign) {
				if (ptable[i].ip_local == 0x7F000001) {

					/* Add TCP Port */
					printk(KERN_INFO  "Skip TCP PORT (127.0.0.1): port_local = %d\n", ptable[i].port_local);

				} else {
					/* Add TCP Src IP */
#if SEND_SRC_IP
					printk(KERN_INFO  "Send TCP SRC IP: ip_foreign = %d.%d.%d.%d\n",
						(unsigned char)(ptable[i].ip_foreign >> 24),
						(unsigned char)(ptable[i].ip_foreign >> 16),
						(unsigned char)(ptable[i].ip_foreign >> 8),
						(unsigned char)(ptable[i].ip_foreign >> 0));

					req_packet_ip.item[req_packet_ip.item_count].mask = MASK_SRC_IP;
					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = (unsigned char)((ptable[i].ip_foreign >> 24) & 0xFF);
					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = (unsigned char)((ptable[i].ip_foreign >> 16) & 0xFF);
					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = (unsigned char)((ptable[i].ip_foreign >> 8) & 0xFF);
					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = (unsigned char)((ptable[i].ip_foreign >> 0) & 0xFF);

					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = 255;
					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = 255;
					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = 255;
					req_packet_ip.item[req_packet_ip.item_count].payload[index++] = 0;

					req_packet_ip.item_count++;


#else
					printk(KERN_INFO  "Skip TCP SRC IP: ip_foreign = %d.%d.%d.%d\n",
						(unsigned char)(ptable[i].ip_foreign >> 24),
						(unsigned char)(ptable[i].ip_foreign >> 16),
						(unsigned char)(ptable[i].ip_foreign >> 8),
						(unsigned char)(ptable[i].ip_foreign >> 0));
#endif

#if SEND_DST_PORT
				printk(KERN_INFO  "Send TCP PORT (Has IP): port_local = %d\n", ptable[i].port_local);

				req_packet_port.item[req_packet_port.item_count].mask = MASK_DST_PORT;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = PROT_TCP;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = (unsigned char)((ptable[i].port_local >> 8) & 0xFF);
				req_packet_port.item[req_packet_port.item_count].payload[index++] = (unsigned char)((ptable[i].port_local >> 0) & 0xFF);
				req_packet_port.item[req_packet_port.item_count].payload[index++] = 0;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = 0;
				req_packet_port.item_count++;
#else
				printk(KERN_INFO  "Skip TCP PORT (Has IP): port_local = %d\n", ptable[i].port_local);
#endif
				}
			} else {
				/* Add TCP Port */
#if SEND_DST_PORT
				printk(KERN_INFO  "Send TCP PORT LISTENING: port_local = %d\n", ptable[i].port_local);

				req_packet_port.item[req_packet_port.item_count].mask = MASK_DST_PORT;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = PROT_TCP;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = (unsigned char)((ptable[i].port_local >> 8) & 0xFF);
				req_packet_port.item[req_packet_port.item_count].payload[index++] = (unsigned char)((ptable[i].port_local >> 0) & 0xFF);
				req_packet_port.item[req_packet_port.item_count].payload[index++] = 0;
				req_packet_port.item[req_packet_port.item_count].payload[index++] = 0;
				req_packet_port.item_count++;

#else
				printk(KERN_INFO  "Skip TCP PORT LISTENING: port_local = %d\n", ptable[i].port_local);
#endif
			}
			break;
		}
	}

	if (req_packet_port.item_count) {
		req_packet_port.length = htonl(req_packet_port.item_count * sizeof(port_type));
		send_item(&req_packet_port, ONCRPC_ACER_ADD_IP_FILTER_PROC, PACKET_TYPE_PORT);
	}

	if (req_packet_ip.item_count) {
		req_packet_ip.length = htonl(req_packet_ip.item_count * sizeof(ip_type));
		send_item(&req_packet_ip, ONCRPC_ACER_ADD_IP_FILTER_PROC, PACKET_TYPE_IP);
	}


	printk(KERN_INFO  "Send Allow Table--\n");
	return ret;

}

void print_allow_table(void)
{
	int i = 0;

	printk(KERN_INFO  "\nPrint Allow Table\n");
	printk(KERN_INFO  "PROT  Local Address  Foreign Address   State\n");
	for (i = 0; i < allow_table_count; i++) {
		switch (packet_allow_table[i].protocol) {
		case PROT_UDP:
			printk(KERN_INFO  "UDP    ");
			break;
		case PROT_TCP:
			printk(KERN_INFO  "TCP    ");
			break;
		default:
			printk(KERN_INFO  "0x%x    ", packet_allow_table[i].protocol);
		}
		printk(KERN_INFO  "%d.%d.%d.%d:%d    %d.%d.%d.%d:%d    %d\n",
			(packet_allow_table[i].ip_local>>24&0xFF),
			(packet_allow_table[i].ip_local>>16&0xFF),
			(packet_allow_table[i].ip_local>>8&0xFF),
			(packet_allow_table[i].ip_local>>0&0xFF),
			packet_allow_table[i].port_local,
			(packet_allow_table[i].ip_foreign>>24&0xFF),
			(packet_allow_table[i].ip_foreign>>16&0xFF),
			(packet_allow_table[i].ip_foreign>>8&0xFF),
			(packet_allow_table[i].ip_foreign>>0&0xFF),
			packet_allow_table[i].port_foreign,
			packet_allow_table[i].state);
	}
	printk(KERN_INFO  "\n");

}
int save_ip_list(void)
{
	printk(KERN_INFO  "++Save IP White List()++\n");

	memset(packet_allow_table, 0x00, sizeof(MAX_ALLOW_TABLE_SIZE) * sizeof(packet_item));
	allow_table_count = 0;
	get_tcp_table(packet_allow_table, &allow_table_count);
	get_udp_table(packet_allow_table, &allow_table_count);

	clear_remote_table();
	print_allow_table();
	send_table(packet_allow_table, allow_table_count);

	printk(KERN_INFO  "--Save IP White List()--\n");
	return 0;
}

