/*
 *
 * (C) 2011-12 - Luca Deri <deri@ntop.org>
 *               Alfredo Cardigliano <cardigliano@ntop.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesses General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#ifndef _PFRING_MOD_USRING_H_
#define _PFRING_MOD_USRING_H_

int  pfring_mod_usring_open (pfring *ring);

void pfring_mod_usring_close(pfring *ring);
int  pfring_mod_usring_enqueue(pfring *ring, char *pkt, u_int pkt_len, u_int8_t flush_packet);
int  pfring_mod_usring_enqueue_parsed(pfring *ring, char *pkt, struct pfring_pkthdr *hdr, u_int8_t flush_packet);

#endif /* _PFRING_MOD_USRING_H_ */
