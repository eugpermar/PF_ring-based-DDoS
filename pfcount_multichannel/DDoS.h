// DDoS.h

#ifndef DDOS_H
#define DDOS_H

#include <netinet/ip.h>
#include "../sglib/sglib.h"

#define TABLE_DIM 1e9

/// Useful data from identify an unique ip_record
struct ip_record{
	struct in_addr ip_src,ip_dst;
	u_int8_t protocol;
	u_int64_t counter;
};

#endif // DDOS_H
