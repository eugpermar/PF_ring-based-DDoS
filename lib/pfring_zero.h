/*
 *
 * Copyright 2012 - ntop
 *
 * Authors: Luca Deri <deri@ntop.org>
 *          Alfredo Cardigliano <cardigliano@ntop.org>
 *
 */

#ifndef _PFRING_ZERO_H_
#define _PFRING_ZERO_H_

int dna_bounce_init(pfring_bounce *bounce);
int dna_bounce_loop(pfring_bounce *bounce, pfringBounceProcesssPacket looper, const u_char *user_bytes, u_int8_t wait_for_packet);
void dna_bounce_destroy(pfring_bounce *bounce);

void *dna_cluster_create(u_int32_t cluster_id, u_int32_t num_apps);
int dna_cluster_register_ring(void *dna_cluster_handle, pfring *ring);
int dna_cluster_on(void *dna_cluster_handle);
int dna_cluster_off(void *dna_cluster_handle);
void dna_cluster_setaffinity(void *dna_cluster_handle, u_int32_t rx_core_id, u_int32_t tx_core_id);
int dna_cluster_stats(void *dna_cluster_handle, u_int64_t *tot_rx_packets, u_int64_t *tot_tx_packets);
void dna_cluster_destroy(void *dna_cluster_handle);

int pfring_dna_cluster_open(pfring *ring);

/* testing prototypes */
u_char *pfring_mod_dna_cluster_pkt_buff_data(pfring *ring, void *pkt_handle);
void pfring_mod_dna_cluster_set_pkt_buff_len(pfring *ring, void *pkt_handle, u_int32_t len);
void pfring_mod_dna_cluster_set_pkt_buff_ifindex(pfring *ring, void *pkt_handle, u_int32_t if_id);
void *pfring_mod_dna_cluster_alloc_pkt_buff(pfring *ring);
void pfring_mod_dna_cluster_free_pkt_buff(pfring *ring, void *pkt_handle);
int pfring_mod_dna_cluster_recv_pkt_buff(pfring *ring, void *pkt_handle, struct pfring_pkthdr *hdr, u_int8_t wait_for_incoming_packet); /* Note: this function fills the buffer pointed by pkt_handle */
int pfring_mod_dna_cluster_send_pkt_buff(pfring *ring, void *pkt_handle, u_int8_t flush_packet); /* Note: this function reset the buffer pointed by pkt_handle */

#endif /* _PFRING_ZERO_H_ */

