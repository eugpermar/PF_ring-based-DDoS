/*
 *
 * (C) 2005-12 - Luca Deri <deri@ntop.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lessed General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 *
 * This code includes contributions courtesy of
 * - Fedor Sakharov <fedor.sakharov@gmail.com>
 *
 */

#define __USE_XOPEN2K
#include <sys/types.h>
#include <pthread.h>

#include "pfring.h"

// #define RING_DEBUG

/* ********************************* */

#include "pfring_mod.h"

#include "pfring_mod_usring.h"

#ifdef HAVE_DAG
#include "pfring_mod_dag.h"
#endif

#ifdef HAVE_DNA
#include "pfring_mod_dna.h"
#endif

#ifdef HAVE_VIRTUAL
#include "pfring_mod_virtual.h"
#endif

static pfring_module_info pfring_module_list[] = {
  { /* usually you don't need to specify this */
    .name = "default",
    .open = pfring_mod_open,
  },
#ifdef HAVE_VIRTUAL
  { /* vPF_RING (guest-side) */
    .name = "host",
    .open = pfring_virtual_open,
  },
#endif
#ifdef HAVE_DAG
  {
    .name = "dag",
    .open = pfring_dag_open,
  },
#endif
#ifdef HAVE_DNA
#if 0 //#ifdef HAVE_ZERO
  {
    .name = "dnacluster",
    .open = pfring_dna_cluster_open,
  },
#endif
  {
    .name = "dna",
    .open = pfring_dna_open,
  },
#endif
  {
    .name = "userspace",
    .open = pfring_mod_usring_open,
  },
  {0}
};

/* **************************************************** */

pfring* pfring_open(char *device_name, u_int32_t caplen, u_int32_t flags) {
  int i = -1;
  int mod_found = 0;
  int ret;
  char *str;
  pfring *ring;

#ifdef RING_DEBUG
  printf("[PF_RING] Attempting to pfring_open(%s)\n", device_name);
#endif

  ring = (pfring*)malloc(sizeof(pfring));
  if(ring == NULL)
    return NULL;

  memset(ring, 0, sizeof(pfring));

  ring->promisc     = (flags & PF_RING_PROMISC) ? 1 : 0;
  ring->caplen      = caplen;
  ring->reentrant   = (flags & PF_RING_REENTRANT) ? 1 : 0;
  ring->direction   = rx_and_tx_direction;
  ring->mode        = send_and_recv_mode;
  ring->long_header = (flags & PF_RING_LONG_HEADER) ? 1 : 0;

#ifdef RING_DEBUG
  printf("pfring_open: device_name=%s\n", device_name);
#endif
  /* modules */

  if(device_name) {
    ret = -1;
    ring->device_name = NULL;

    while (pfring_module_list[++i].name) {
      char *str1;

      if(!(str = strstr(device_name, pfring_module_list[i].name))) continue;
      if(!pfring_module_list[i].open)                              continue;

      str1 = strchr(str, ':');

#ifdef RING_DEBUG
      printf("pfring_open: found module %s\n", pfring_module_list[i].name);
#endif

      mod_found = 1;
      ring->device_name = str1 ? strdup(++str1) : strdup(device_name);
      ret = pfring_module_list[i].open(ring);
      break;
    }
  }

  /* default */
  if(!mod_found) {
    ring->device_name = strdup(device_name ? device_name : "any");

    ret = pfring_mod_open(ring);
  }

  if(ret < 0) {
    if(ring->device_name) free(ring->device_name);
    free(ring);
    return NULL;
  }

  if(unlikely(ring->reentrant)) {
    pthread_rwlock_init(&ring->rx_lock, PTHREAD_PROCESS_PRIVATE);
    pthread_rwlock_init(&ring->tx_lock, PTHREAD_PROCESS_PRIVATE);
  }

  ring->socket_default_accept_policy = 1; /* Accept (default) */

  ring->rdi.device_id = ring->rdi.port_id = -1; /* Default */

  ring->initialized = 1;

#ifdef RING_DEBUG
  printf("[PF_RING] Successfully open pfring_open(%s)\n", device_name);
#endif

  return ring;
}

/* **************************************************** */

pfring* pfring_open_consumer(char *device_name, u_int32_t caplen, u_int32_t flags,
			     u_int8_t consumer_plugin_id,
			     char* consumer_data, u_int consumer_data_len) {
  pfring *ring = pfring_open(device_name, caplen, flags);

  if(ring) {
    if(consumer_plugin_id > 0) {
      int rc;

      ring->kernel_packet_consumer = consumer_plugin_id;
      rc = pfring_set_packet_consumer_mode(ring, consumer_plugin_id,
					   consumer_data, consumer_data_len);
      if(rc < 0) {
	pfring_close(ring);
	return(NULL);
      }
    }
  }

  return ring;
}

/* **************************************************** */

u_int8_t pfring_open_multichannel(char *device_name, u_int32_t caplen,
				  u_int32_t flags,
				  pfring* ring[MAX_NUM_RX_CHANNELS]) {
  u_int8_t num_channels, i, num = 0;
  char *at;
  char base_device_name[32];

  snprintf(base_device_name, sizeof(base_device_name), "%s", device_name);
  at = strchr(base_device_name, '@');
  if(at != NULL)
    at[0] = '\0';

  /* Count how many RX channel the specified device supports */
  ring[0] = pfring_open(base_device_name, caplen, flags);

  if(ring[0] == NULL)
    return(0);
  else
    num_channels = pfring_get_num_rx_channels(ring[0]);

  pfring_close(ring[0]);

  /* Now do the real job */
  for(i=0; i<num_channels; i++) {
    char dev[32];

    snprintf(dev, sizeof(dev), "%s@%d", base_device_name, i);
    ring[i] = pfring_open(dev, caplen, flags);

    if(ring[i] == NULL)
      return(num);
    else
      num++;
  }

  return(num);
}

/* **************************************************** */

void pfring_close(pfring *ring) {
  if(!ring)
    return;

  pfring_shutdown(ring);

  if(ring->close)
    ring->close(ring);

  if(unlikely(ring->reentrant)) {
    pthread_rwlock_destroy(&ring->rx_lock);
    pthread_rwlock_destroy(&ring->tx_lock);
  }

  free(ring->device_name);
  free(ring);
}

/* **************************************************** */

void pfring_shutdown(pfring *ring) {
  if (!ring)
    return;

  ring->is_shutting_down = ring->break_recv_loop = 1;

  if(ring->shutdown)
    ring->shutdown(ring);
}

/* **************************************************** */

void pfring_config(u_short cpu_percentage) {
  static u_int pfring_initialized = 0;

  if(!pfring_initialized) {
    struct sched_param schedparam;

    /*if(cpu_percentage >= 50) mlockall(MCL_CURRENT|MCL_FUTURE); */

    pfring_initialized = 1;
    schedparam.sched_priority = cpu_percentage;
    if(sched_setscheduler(0, SCHED_FIFO, &schedparam) == -1) {
      printf("error while setting the scheduler, errno=%i\n", errno);
      exit(1);
    }
  }
}

/* **************************************************** */

int pfring_set_reflector_device(pfring *ring, char *device_name) {
  if((device_name == NULL) || ring->reflector_socket)
    return(-1);

  ring->reflector_socket = pfring_open(device_name, ring->caplen, PF_RING_PROMISC);

  if(ring->reflector_socket != NULL) {
    pfring_set_socket_mode(ring->reflector_socket, tx_only_direction);
    pfring_enable_ring(ring->reflector_socket);
    return(0);
  } else
    return(-1);
}

/* **************************************************** */

int pfring_loop(pfring *ring, pfringProcesssPacket looper,
		const u_char *user_bytes, u_int8_t wait_for_packet) {
  u_char *buffer = NULL;
  struct pfring_pkthdr hdr;
  int rc = 0;

  memset(&hdr, 0, sizeof(hdr));
  ring->break_recv_loop = 0;

  if((! ring)
     || ring->is_shutting_down
     || (! ring->recv)
     || ring->mode == send_only_mode)
    return -1;

  while(!ring->break_recv_loop) {
    rc = ring->recv(ring, &buffer, 0, &hdr, wait_for_packet);
    if(rc < 0)
      break;
    else if(rc > 0) {
      looper(&hdr, buffer, user_bytes);
    } else {
      /* if(!wait_for_packet) usleep(1); */
    }
  }

  return(rc);
}

/* **************************************************** */

void pfring_breakloop(pfring *ring) {
  if(!ring)
    return;

  ring->break_recv_loop = 1;
}

/* **************************************************** */

void pfring_bundle_init(pfring_bundle *bundle, bundle_read_policy p) {
  memset(bundle, 0, sizeof(pfring_bundle));
  bundle->policy = p;
}

/* **************************************************** */

int pfring_bundle_add(pfring_bundle *bundle, pfring *ring) {
  if(bundle->num_sockets >= (MAX_NUM_BUNDLE_ELEMENTS-1))
    return(-1);

  bundle->sockets[bundle->num_sockets] = ring;
  bundle->pfd[bundle->num_sockets].fd = pfring_get_selectable_fd(ring);
  bundle->num_sockets++;

  pfring_enable_ring(ring);

  return(0);
}

/* **************************************************** */

/* Returns the first bundle socket with something to read */
int pfring_bundle_poll(pfring_bundle *bundle, u_int wait_duration) {
  int i;

  for(i=0; i<bundle->num_sockets; i++) {
    pfring_sync_indexes_with_kernel(bundle->sockets[i]);
    bundle->pfd[i].events  = POLLIN /* | POLLERR */;
    bundle->pfd[i].revents = 0;
  }

  errno = 0;
  return poll(bundle->pfd, bundle->num_sockets, wait_duration);
}

/* **************************************************** */

int pfring_bundle_read(pfring_bundle *bundle,
		       u_char** buffer, u_int buffer_len,
		       struct pfring_pkthdr *hdr,
		       u_int8_t wait_for_incoming_packet) {
  int i, sock_id = -1, found, rc, empty_rings, scans;
  struct timespec ts = { 0 };
  struct timespec tmpts;

redo_pfring_bundle_read:

  switch(bundle->policy) {
  case pick_round_robin:
    for(i=0; i<bundle->num_sockets; i++) {
      bundle->last_read_socket = (bundle->last_read_socket + 1) % bundle->num_sockets;

      if(pfring_is_pkt_available(bundle->sockets[bundle->last_read_socket])) {
	return(pfring_recv(bundle->sockets[bundle->last_read_socket], buffer,
			   buffer_len, hdr, 0));
      }
    }
    break;

  case pick_fifo:
    found = 0;
    empty_rings = 0;
    scans = 0;

  sockets_scan:
    scans++;
    for(i=0; i<bundle->num_sockets; i++) {
      pfring *ring = bundle->sockets[i];

      rc = pfring_next_pkt_time(ring, &tmpts);

      if (rc == 0) {
      	if(!found || timespec_is_before(&tmpts, &ts)) {
	  memcpy(&ts, &tmpts, sizeof(struct timespec));
	  found = 1;
	  sock_id = i;
	}
      } else if (rc == PF_RING_ERROR_NO_PKT_AVAILABLE) {
        empty_rings = 1;
      } else {
        /* error */
      }
    }

    if(found) {
      if (empty_rings > 0 && scans == 1)
        goto sockets_scan; /* scanning ring twice (safety check) */

      return(pfring_recv(bundle->sockets[sock_id], buffer, buffer_len, hdr, 0));
    }
    break;
  }

  if(wait_for_incoming_packet) {
    rc = pfring_bundle_poll(bundle, bundle->sockets[0]->poll_duration);

    if(rc > 0) goto redo_pfring_bundle_read;
    else return(rc);
  }

  return(0);
}

/* **************************************************** */

void pfring_bundle_destroy(pfring_bundle *bundle) {
  int i;

  for(i=0; i<bundle->num_sockets; i++) {
    pfring_disable_ring(bundle->sockets[i]);
  }

  memset(bundle, 0, sizeof(pfring_bundle));
}

/* **************************************************** */

void pfring_bundle_close(pfring_bundle *bundle) {
  int i;

  for(i=0; i<bundle->num_sockets; i++) {
    pfring_close(bundle->sockets[i]);
  }

  memset(bundle, 0, sizeof(pfring_bundle));
}

/* **************************************************** */

int pfring_bounce_init(pfring_bounce *bounce, pfring *ingress_ring, pfring *egress_ring) {

  if (bounce == NULL || ingress_ring == NULL || egress_ring == NULL ||
      ingress_ring->bounce_init == NULL)
    return -1;

  if (pfring_set_socket_mode(ingress_ring, recv_only_mode) != 0 ||
      pfring_set_socket_mode(egress_ring, send_only_mode) != 0)
    return -1;

  bounce->rx_socket = ingress_ring;
  bounce->tx_socket = egress_ring;

  if(ingress_ring->bounce_init(bounce) != 0) {
    bounce->rx_socket = NULL;
    bounce->tx_socket = NULL;
    return -1;
  }

  /* disabling harmful functions / backing up func ptrs */
  bounce->recv          = ingress_ring->recv,         ingress_ring->recv         = NULL;
  bounce->send          = egress_ring->send,          egress_ring->send          = NULL;
  bounce->send_parsed   = egress_ring->send_parsed,   egress_ring->send_parsed   = NULL;
  bounce->send_get_time = egress_ring->send_get_time, egress_ring->send_get_time = NULL;

  if (pfring_enable_ring(ingress_ring) != 0 ||
      pfring_enable_ring(egress_ring) != 0) {
    pfring_bounce_destroy(bounce);
    return -1;
  }

  pthread_rwlock_init(&bounce->lock, PTHREAD_PROCESS_PRIVATE);

  return 0;
}

/* **************************************************** */

int pfring_bounce_loop(pfring_bounce *bounce, pfringBounceProcesssPacket looper,
                       const u_char *user_bytes, u_int8_t wait_for_packet) {
  int rc;

  if (bounce == NULL || bounce->rx_socket == NULL || bounce->tx_socket == NULL ||
      bounce->rx_socket->bounce_loop == NULL)
    return -1;

  pthread_rwlock_wrlock(&bounce->lock);
  if (bounce->running) { pthread_rwlock_unlock(&bounce->lock); return -1; }
  bounce->running = 1;
  pthread_rwlock_unlock(&bounce->lock);

  bounce->break_loop = 0;

  rc = bounce->rx_socket->bounce_loop(bounce, looper, user_bytes, wait_for_packet);

  pthread_rwlock_wrlock(&bounce->lock);
  bounce->running = 0;
  pthread_rwlock_unlock(&bounce->lock);

  return rc;
}

/* **************************************************** */

void pfring_bounce_breakloop(pfring_bounce *bounce) {
  if(!bounce || !bounce->running)
    return;

  bounce->break_loop = 1;
  pfring_breakloop(bounce->rx_socket);
}

/* **************************************************** */

void pfring_bounce_destroy(pfring_bounce *bounce) {

  if (bounce == NULL || bounce->rx_socket == NULL || bounce->tx_socket == NULL)
    return;

  pfring_disable_ring(bounce->rx_socket);
  pfring_disable_ring(bounce->tx_socket);

  /* restoring func ptrs */
  bounce->rx_socket->recv          = bounce->recv;
  bounce->tx_socket->send          = bounce->send;
  bounce->tx_socket->send_parsed   = bounce->send_parsed;
  bounce->tx_socket->send_get_time = bounce->send_get_time;

  if (bounce->rx_socket->bounce_destroy)
    bounce->rx_socket->bounce_destroy(bounce);

  memset(bounce, 0, sizeof(pfring_bounce));
}

/* **************************************************** */
/*                Module-specific functions             */
/* **************************************************** */

int pfring_stats(pfring *ring, pfring_stat *stats) {
  if(ring && ring->stats)
    return ring->stats(ring, stats);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_recv(pfring *ring, u_char** buffer, u_int buffer_len,
		struct pfring_pkthdr *hdr,
		u_int8_t wait_for_incoming_packet) {
  if(likely((ring
	     && ring->enabled
	     && ring->recv
	     && (ring->mode != send_only_mode)))) {
    int rc;

    /* Reentrancy is not compatible with zero copy */
    if(unlikely((buffer_len == 0) && ring->reentrant))
      return(PF_RING_ERROR_INVALID_ARGUMENT);

    ring->break_recv_loop = 0;
    rc = ring->recv(ring, buffer, buffer_len, hdr, wait_for_incoming_packet);

    if(unlikely(ring->reflector_socket != NULL))
      pfring_send(ring->reflector_socket, (char*)buffer, hdr->caplen, 0 /* flush */);

    return rc;
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_recv_parsed(pfring *ring, u_char** buffer, u_int buffer_len,
		       struct pfring_pkthdr *hdr, u_int8_t wait_for_incoming_packet,
		       u_int8_t level /* 1..4 */, u_int8_t add_timestamp, u_int8_t add_hash) {
  int rc = pfring_recv(ring, buffer, buffer_len, hdr, wait_for_incoming_packet);

  if(rc > 0)
    rc = pfring_parse_pkt(*buffer, hdr, level, add_timestamp, add_hash);

  return rc;
}

/* **************************************************** */

int pfring_set_poll_watermark(pfring *ring, u_int16_t watermark) {
  if(ring && ring->set_poll_watermark)
    return ring->set_poll_watermark(ring, watermark);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_poll_duration(pfring *ring, u_int duration) {
  if(ring && ring->set_poll_duration)
    return ring->set_poll_duration(ring, duration);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_tx_watermark(pfring *ring, u_int16_t watermark) {
  if(ring && ring->set_tx_watermark)
    return ring->set_tx_watermark(ring, watermark);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_add_hw_rule(pfring *ring, hw_filtering_rule *rule) {
  if(ring && ring->add_hw_rule)
    return ring->add_hw_rule(ring, rule);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_remove_hw_rule(pfring *ring, u_int16_t rule_id) {
  if(ring && ring->remove_hw_rule)
    return ring->remove_hw_rule(ring, rule_id);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_channel_id(pfring *ring, u_int32_t channel_id) {
  if(ring && ring->set_channel_id)
    return ring->set_channel_id(ring, channel_id);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_application_name(pfring *ring, char *name) {
  if(ring && ring->set_application_name)
    return ring->set_application_name(ring, name);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_bind(pfring *ring, char *device_name) {
  if(ring && ring->bind)
    return ring->bind(ring, device_name);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_send(pfring *ring, char *pkt, u_int pkt_len, u_int8_t flush_packet) {
  int rc = -1;

  if(unlikely(pkt_len > 9000 /* Jumbo MTU */))
    return rc;

  if(likely(ring
	    && ring->enabled
	    && (!ring->is_shutting_down)
	    && ring->send
	    && (ring->mode != recv_only_mode))) {

    if(unlikely(ring->reentrant))
      pthread_rwlock_wrlock(&ring->tx_lock);

    rc =  ring->send(ring, pkt, pkt_len, flush_packet);

    if(unlikely(ring->reentrant))
      pthread_rwlock_unlock(&ring->tx_lock);
  }

  return rc;
}

/* **************************************************** */

int pfring_send_parsed(pfring *ring, char *pkt, struct pfring_pkthdr *hdr, u_int8_t flush_packet) {
  int rc = -1;

  if(likely(ring
	    && ring->enabled
	    && (!ring->is_shutting_down)
	    && ring->send_parsed
	    && (ring->mode != recv_only_mode))) {

    if(unlikely(ring->reentrant))
      pthread_rwlock_wrlock(&ring->tx_lock);

    rc =  ring->send_parsed(ring, pkt, hdr, flush_packet);

    if(unlikely(ring->reentrant))
      pthread_rwlock_unlock(&ring->tx_lock);

    return rc;
  }

  if(ring && !ring->send_parsed)
    rc = PF_RING_ERROR_NOT_SUPPORTED;

  return rc;
}

/* **************************************************** */

int pfring_send_get_time(pfring *ring, char *pkt, u_int pkt_len, struct timespec *ts) {
  int rc = -1;

  if(likely(ring
	    && ring->enabled
	    && (!ring->is_shutting_down)
	    && ring->send_get_time
	    && (ring->mode != recv_only_mode))) {

    if(unlikely(ring->reentrant))
      pthread_rwlock_wrlock(&ring->tx_lock);

    rc =  ring->send_get_time(ring, pkt, pkt_len, ts);

    if(unlikely(ring->reentrant))
      pthread_rwlock_unlock(&ring->tx_lock);

    return rc;
  }

  if(ring && !ring->send_get_time)
    rc = PF_RING_ERROR_NOT_SUPPORTED;

  return rc;
}

/* **************************************************** */

u_int8_t pfring_get_num_rx_channels(pfring *ring) {
  if(ring && ring->get_num_rx_channels)
    return ring->get_num_rx_channels(ring);

  return 1;
}

/* **************************************************** */

int pfring_set_sampling_rate(pfring *ring, u_int32_t rate /* 1 = no sampling */) {
  if(ring && ring->set_sampling_rate)
    return ring->set_sampling_rate(ring, rate);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_get_selectable_fd(pfring *ring) {
  if(ring && ring->get_selectable_fd)
    return ring->get_selectable_fd(ring);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_direction(pfring *ring, packet_direction direction) {
  if(ring && ring->set_direction) {
    int rc;

    if(ring->enabled)
      return -1; /* direction must be set before pfring_enable() */

    rc = ring->set_direction(ring, direction);

    if(rc == 0)
      ring->direction = direction;

    return(rc);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_socket_mode(pfring *ring, socket_mode mode) {
  if(ring && ring->set_socket_mode) {
    int rc;

    if(ring->enabled)
      return -1; /* direction must be set before pfring_enable() */

    rc = ring->set_socket_mode(ring, mode);

    if(rc == 0)
      ring->mode = mode;

    /* We move back from RX/TX to TX-only mode */
    if(mode == send_only_mode) {
      if(ring->promisc && ring->clear_promisc) {
	if(pfring_set_if_promisc(ring->device_name, 0) == 0)
	  ring->clear_promisc = 0;
      }
    }

    return(rc);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_cluster(pfring *ring, u_int clusterId, cluster_type the_type) {
  if(ring && ring->set_cluster)
    return ring->set_cluster(ring, clusterId, the_type);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_remove_from_cluster(pfring *ring) {
  if(ring && ring->remove_from_cluster)
    return ring->remove_from_cluster(ring);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_master_id(pfring *ring, u_int32_t master_id) {
  if(ring && ring->set_master_id)
    return ring->set_master_id(ring, master_id);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_master(pfring *ring, pfring *master) {
  if(ring && ring->set_master)
    return ring->set_master(ring, master);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

u_int16_t pfring_get_ring_id(pfring *ring) {
  if(ring && ring->get_ring_id)
    return ring->get_ring_id(ring);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

u_int32_t pfring_get_num_queued_pkts(pfring *ring) {
  if(ring && ring->get_num_queued_pkts)
    return ring->get_num_queued_pkts(ring);

  return 0;
}

/* **************************************************** */

u_int8_t pfring_get_packet_consumer_mode(pfring *ring) {
  if(ring && ring->get_packet_consumer_mode)
    return ring->get_packet_consumer_mode(ring);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_packet_consumer_mode(pfring *ring, u_int8_t plugin_id,
				    char *plugin_data, u_int plugin_data_len) {
  if(ring && ring->set_packet_consumer_mode)
    return ring->set_packet_consumer_mode(ring, plugin_id, plugin_data, plugin_data_len);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_get_hash_filtering_rule_stats(pfring *ring, hash_filtering_rule* rule,
					 char* stats, u_int *stats_len) {
  if(ring && ring->get_hash_filtering_rule_stats)
    return ring->get_hash_filtering_rule_stats(ring, rule, stats, stats_len);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_handle_hash_filtering_rule(pfring *ring, hash_filtering_rule* rule_to_add,
				      u_char add_rule) {
  if(ring && ring->handle_hash_filtering_rule)
    return ring->handle_hash_filtering_rule(ring, rule_to_add, add_rule);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_purge_idle_hash_rules(pfring *ring, u_int16_t inactivity_sec) {
  if(ring && ring->purge_idle_hash_rules)
    return ring->purge_idle_hash_rules(ring, inactivity_sec);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_purge_idle_rules(pfring *ring, u_int16_t inactivity_sec) {
  if(ring && ring->purge_idle_rules)
    return ring->purge_idle_rules(ring, inactivity_sec);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_add_filtering_rule(pfring *ring, filtering_rule* rule_to_add) {
  if(ring && ring->add_filtering_rule)
    return ring->add_filtering_rule(ring, rule_to_add);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_remove_filtering_rule(pfring *ring, u_int16_t rule_id) {
  if(ring && ring->remove_filtering_rule)
    return ring->remove_filtering_rule(ring, rule_id);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_get_filtering_rule_stats(pfring *ring, u_int16_t rule_id,
				    char* stats, u_int *stats_len) {
  if(ring && ring->get_filtering_rule_stats)
    return ring->get_filtering_rule_stats(ring, rule_id, stats, stats_len);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_toggle_filtering_policy(pfring *ring, u_int8_t rules_default_accept_policy) {
  if (ring && ring->toggle_filtering_policy)
    return ring->toggle_filtering_policy(ring, rules_default_accept_policy);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_enable_rss_rehash(pfring *ring) {
  if(ring && ring->enable_rss_rehash)
    return ring->enable_rss_rehash(ring);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_poll(pfring *ring, u_int wait_duration) {
  if(likely((ring && ring->poll)))
    return ring->poll(ring, wait_duration);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_version(pfring *ring, u_int32_t *version) {
  if(ring && ring->version)
    return ring->version(ring, version);

  *version = RING_VERSION_NUM;
  return 0;/*PF_RING_ERROR_NOT_SUPPORTED*/;
}

/* **************************************************** */

int pfring_get_bound_device_address(pfring *ring, u_char mac_address[6]) {
  if(ring && ring->get_bound_device_address)
    return ring->get_bound_device_address(ring, mac_address);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_get_bound_device_id(pfring *ring, int *device_id) {
  socklen_t len = sizeof(int);

  return(getsockopt(ring->fd, 0, SO_GET_BOUND_DEVICE_ID, device_id, &len));
}

/* **************************************************** */

u_int16_t pfring_get_slot_header_len(pfring *ring) {
  if(ring && ring->get_slot_header_len)
    return ring->get_slot_header_len(ring);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_virtual_device(pfring *ring, virtual_filtering_device_info *info) {
  if(ring && ring->set_virtual_device)
    return ring->set_virtual_device(ring, info);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_loopback_test(pfring *ring, char *buffer, u_int buffer_len, u_int test_len) {
  if(ring && ring->loopback_test)
    return ring->loopback_test(ring, buffer, buffer_len, test_len);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_enable_ring(pfring *ring) {
  if(ring && ring->enable_ring) {
    int rc;

    if(ring->enabled) return(0);

    rc = ring->enable_ring(ring);
    if(rc == 0) ring->enabled = 1;

    return rc;
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_disable_ring(pfring *ring) {
  if(ring && ring->disable_ring) {
    int rc;

    if(!ring->enabled) return(0);

    rc = ring->disable_ring(ring);
    if(rc == 0) ring->enabled = 0;

    return rc;
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_is_pkt_available(pfring *ring){
  if(ring && ring->is_pkt_available) {
    return ring->is_pkt_available(ring);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_next_pkt_time(pfring *ring, struct timespec *ts){
  if(ring && ring->next_pkt_time) {
    return ring->next_pkt_time(ring, ts);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_next_pkt_raw_timestamp(pfring *ring, u_int64_t *timestamp_ns){
  if(ring && ring->next_pkt_raw_timestamp) {
    return ring->next_pkt_raw_timestamp(ring, timestamp_ns);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_bpf_filter(pfring *ring, char *filter_buffer){
  if(ring && ring->set_bpf_filter) {
    return ring->set_bpf_filter(ring, filter_buffer);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_remove_bpf_filter(pfring *ring){
  if(ring && ring->remove_bpf_filter) {
    return ring->remove_bpf_filter(ring);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

void pfring_sync_indexes_with_kernel(pfring *ring) {
  if(ring && ring->sync_indexes_with_kernel)
    ring->sync_indexes_with_kernel(ring);
}

/* **************************************************** */

int pfring_send_last_rx_packet(pfring *ring, int tx_interface_id) {
  /*
    We can't send the last packet with multithread as the concept of "last"
    does not apply here having threads that compete for packets
  */
  if(unlikely(ring->reentrant || (!ring->long_header)))
    return(PF_RING_ERROR_NOT_SUPPORTED);

  if(ring && ring->send_last_rx_packet)
    return(ring->send_last_rx_packet(ring, tx_interface_id));

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_filtering_mode(pfring *ring, filtering_mode mode){
  if(!ring)
    return -1;

  ring->ft_mode = mode;
  return 0;
}

/* **************************************************** */

int pfring_get_device_clock(pfring *ring, struct timespec *ts) {
  if(ring && ring->get_device_clock) {
    return ring->get_device_clock(ring, ts);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_set_device_clock(pfring *ring, struct timespec *ts) {
  if(ring && ring->set_device_clock) {
    return ring->set_device_clock(ring, ts);
  }

  return PF_RING_ERROR_NOT_SUPPORTED;
}

/* **************************************************** */

int pfring_adjust_device_clock(pfring *ring, struct timespec *offset, int8_t sign) {
  if(ring && ring->adjust_device_clock) {
    return ring->adjust_device_clock(ring, offset, sign);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

u_int pfring_get_num_tx_slots(pfring* ring) {
  if(ring && ring->dna_get_num_tx_slots) {
    return ring->dna_get_num_tx_slots(ring);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

u_int pfring_get_num_rx_slots(pfring* ring) {
  if(ring && ring->dna_get_num_rx_slots) {
    return ring->dna_get_num_rx_slots(ring);
  }

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_copy_tx_packet_into_slot(pfring* ring,
				    u_int16_t tx_slot_id, char* buffer, u_int len) {
  if(ring && ring->dna_copy_tx_packet_into_slot)
    return ring->dna_copy_tx_packet_into_slot(ring, tx_slot_id, buffer, len);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

u_char* pfring_get_pkt_buff_data(pfring *ring, pfring_pkt_buff *pkt_handle) {
  if(ring && ring->get_pkt_buff_data)
    return ring->get_pkt_buff_data(ring, pkt_handle);

  return NULL;
}

/* **************************************************** */

void pfring_set_pkt_buff_len(pfring *ring, pfring_pkt_buff *pkt_handle, u_int32_t len) {
  if(ring && ring->set_pkt_buff_len)
    ring->set_pkt_buff_len(ring, pkt_handle, len);
}

/* **************************************************** */

void pfring_set_pkt_buff_ifindex(pfring *ring, pfring_pkt_buff *pkt_handle, u_int32_t if_id) {
  if(ring && ring->set_pkt_buff_ifindex)
    ring->set_pkt_buff_ifindex(ring, pkt_handle, if_id);
}

/* **************************************************** */

void pfring_add_pkt_buff_ifindex(pfring *ring, pfring_pkt_buff *pkt_handle, u_int32_t if_id) {
  if(ring && ring->add_pkt_buff_ifindex)
    ring->add_pkt_buff_ifindex(ring, pkt_handle, if_id);
}

/* **************************************************** */

pfring_pkt_buff* pfring_alloc_pkt_buff(pfring *ring) {
  if(ring && ring->alloc_pkt_buff)
    return ring->alloc_pkt_buff(ring);

  return NULL;
}

/* **************************************************** */

void pfring_release_pkt_buff(pfring *ring, pfring_pkt_buff *pkt_handle) {
  if(ring && ring->release_pkt_buff)
    ring->release_pkt_buff(ring, pkt_handle);
}

/* **************************************************** */

int pfring_recv_pkt_buff(pfring *ring, pfring_pkt_buff *pkt_handle, struct pfring_pkthdr *hdr, u_int8_t wait_for_incoming_packet) {
  /* Note: this function fills the buffer pointed by pkt_handle */

  if(ring && ring->recv_pkt_buff)
    return ring->recv_pkt_buff(ring, pkt_handle, hdr, wait_for_incoming_packet);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

int pfring_send_pkt_buff(pfring *ring, pfring_pkt_buff *pkt_handle, u_int8_t flush_packet) {
  /* Note: this function reset the buffer pointed by pkt_handle */

  if(ring && ring->send_pkt_buff)
    return ring->send_pkt_buff(ring, pkt_handle, flush_packet);

  return(PF_RING_ERROR_NOT_SUPPORTED);
}

/* **************************************************** */

