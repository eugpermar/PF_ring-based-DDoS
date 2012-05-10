/*
 *
 * Copyright 2012 - ntop.org
 *
 * Authors:
 *  Alfredo Cardigliano <cardigliano@ntop.org>
 *  Luca Deri <deri@ntop.org>
 *
 */

#ifndef _PFRING_VIRTUAL_MOD_H_
#define _PFRING_VIRTUAL_MOD_H_

int pfring_virtual_open (pfring *ring);

void pfring_virtual_close(pfring *ring);
int pfring_virtual_set_poll_watermark(pfring *ring, u_int16_t watermark);
int pfring_virtual_set_channel_id(pfring *ring, u_int32_t channel_id);
int pfring_virtual_set_application_name(pfring *ring, char *name);
int pfring_virtual_bind(pfring *ring, char *device_name);
u_int8_t pfring_virtual_get_num_rx_channels(pfring *ring);
int pfring_virtual_set_sampling_rate(pfring *ring, u_int32_t rate);
int pfring_virtual_set_direction(pfring *ring, packet_direction direction);
int pfring_virtual_set_socket_mode(pfring *ring, socket_mode mode);
int pfring_virtual_set_cluster(pfring *ring, u_int clusterId, cluster_type the_type);
int pfring_virtual_remove_from_cluster(pfring *ring);
int pfring_virtual_set_master_id(pfring *ring, u_int32_t master_id);
u_int16_t pfring_virtual_get_ring_id(pfring *ring);
u_int32_t pfring_virtual_get_num_queued_pkts(pfring *ring);
u_int8_t pfring_virtual_get_packet_consumer_mode(pfring *ring);
int pfring_virtual_set_packet_consumer_mode(pfring *ring, u_int8_t plugin_id,
				        char *plugin_data, u_int plugin_data_len);
int pfring_virtual_get_hash_filtering_rule_stats(pfring *ring,
					     hash_filtering_rule* rule,
					     char* stats, u_int *stats_len);
int pfring_virtual_handle_hash_filtering_rule(pfring *ring,
					  hash_filtering_rule* rule_to_add,
					  u_char add_rule);
int pfring_virtual_purge_idle_hash_rules(pfring *ring, u_int16_t inactivity_sec);  
int pfring_virtual_add_filtering_rule(pfring *ring, filtering_rule* rule_to_add);
int pfring_virtual_remove_filtering_rule(pfring *ring, u_int16_t rule_id);
int pfring_virtual_get_filtering_rule_stats(pfring *ring, u_int16_t rule_id,
					char* stats, u_int *stats_len);
int pfring_virtual_purge_idle_rules(pfring *ring, u_int16_t inactivity_sec);
int pfring_virtual_toggle_filtering_policy(pfring *ring, u_int8_t rules_default_accept_policy);
int pfring_virtual_enable_rss_rehash(pfring *ring);
int pfring_virtual_poll(pfring *ring, u_int wait_duration);
int pfring_virtual_version(pfring *ring, u_int32_t *version);
int pfring_virtual_get_bound_device_address(pfring *ring, u_char mac_address[6]);
u_int16_t pfring_virtual_get_slot_header_len(pfring *ring);
int pfring_virtual_set_virtual_device(pfring *ring, virtual_filtering_device_info *info);
int pfring_virtual_add_hw_rule(pfring *ring, hw_filtering_rule *rule);
int pfring_virtual_remove_hw_rule(pfring *ring, u_int16_t rule_id);
int pfring_virtual_set_bpf_filter(pfring *ring, char *filter_buffer);
int pfring_virtual_remove_bpf_filter(pfring *ring);
int pfring_virtual_enable_ring(pfring *ring);
int pfring_virtual_disable_ring(pfring *ring);
void pfring_virtual_shutdown(pfring *ring);

int pfring_virtual_scan_devices(int device_id, char *device_path);
void pfring_virtual_close_req(pfring *ring);

#endif /* _PFRING_VIRTUAL_MOD_H_ */
