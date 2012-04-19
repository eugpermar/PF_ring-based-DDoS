/*
 *
 * (C) 2011-12 - Luca Deri <deri@ntop.org>
 *               Alfredo Cardigliano <cardigliano@ntop.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lessed General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 */

#include "pfring.h"
#include "pfring_mod.h"
#include "pfring_mod_usring.h"

#define gcc_mb() __asm__ __volatile__("": : :"memory");

#define USE_WATERMARK

//#define PROFILING
#ifdef PROFILING
typedef unsigned long long ticks;

ticks tick_delta_tot = 0;
u_int32_t n_call_tot = 0;

static __inline__ ticks getticks(void)
{
  unsigned a, d;
  /* asm("cpuid"); */
  asm volatile("rdtsc" : "=a" (a), "=d" (d));
  return (((ticks)a) | (((ticks)d) << 32));
}
#endif

/* ******************************* */

int pfring_mod_usring_open(pfring *ring) {
  int rc;
  int tot_ring_mem;
  socklen_t s_len;

  ring->close        = pfring_mod_usring_close;
  ring->send         = pfring_mod_usring_enqueue;
  ring->send_parsed  = pfring_mod_usring_enqueue_parsed;

  ring->enable_ring = pfring_mod_enable_ring;

#ifdef USE_WATERMARK
  ring->dna.dna_rx_sync_watermark = ring->dna.dna_tx_sync_watermark = DEFAULT_MIN_PKT_QUEUED;
#endif

  if(strncmp(ring->device_name, "usr", 3))
    return -1; /* Device name must me usrX */

  ring->fd = socket(PF_RING, SOCK_RAW, htons(ETH_P_ALL));

  if(ring->fd < 0)
    return -1;

  rc = setsockopt(ring->fd, 0, SO_ATTACH_USERSPACE_RING, ring->device_name, sizeof(ring->device_name) + 1);

  if (rc < 0) {
    close(ring->fd);
    return -1;
  }

  ring->buffer = (char *) mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, ring->fd, 0);

  if(ring->buffer == MAP_FAILED) {
    close(ring->fd);
    return -1;
  }

  ring->slots_info = (FlowSlotInfo *) ring->buffer;

  if(ring->slots_info->version != RING_FLOWSLOT_VERSION) {
    printf("Wrong RING version: "
	   "kernel is %i, libpfring was compiled with %i\n",
	   ring->slots_info->version, RING_FLOWSLOT_VERSION);
    close(ring->fd);
    return -1;
  }

  tot_ring_mem = ring->slots_info->tot_mem;
  munmap(ring->buffer, PAGE_SIZE);

  ring->buffer = (char *) mmap(NULL, tot_ring_mem, PROT_READ|PROT_WRITE, MAP_SHARED, ring->fd, 0);

  if(ring->buffer == MAP_FAILED) {
    close(ring->fd);
    return -1;
  }

  ring->slots_info = (FlowSlotInfo *) ring->buffer;
  ring->slots = (char *) (ring->buffer + sizeof(FlowSlotInfo));

  s_len = sizeof(ring->slot_header_len);
  rc = getsockopt(ring->fd, 0, SO_GET_PKT_HEADER_LEN, &ring->slot_header_len, &s_len);

  if (rc < 0) {
    pfring_close(ring);
    return -1;
  }

  s_len = sizeof(ring->caplen);
  rc = getsockopt(ring->fd, 0, SO_GET_BUCKET_LEN, &ring->caplen, &s_len);

  if (rc < 0) {
    pfring_close(ring);
    return -1;
  }

  return 0;
}

/* ******************************* */

void pfring_mod_usring_close(pfring *ring) {
  if(ring->buffer != NULL)
    munmap(ring->buffer, ring->slots_info->tot_mem);

  close(ring->fd);

#ifdef PROFILING
  printf("[PROFILING] num_sendto_calls: %lu avg_sendto_ticks:%llu\n", 
         n_call_tot, tick_delta_tot, tick_delta_tot/n_call_tot);
#endif
}

/* ******************************* */

static inline char* get_slot(pfring *ring, u_int32_t off) {
  return &(ring->slots[off]); 
}

/* ******************************* */

static inline int get_next_slot_offset(pfring *ring, u_int32_t off)
{
  struct pfring_pkthdr *hdr;
  u_int32_t real_slot_size;

  hdr = (struct pfring_pkthdr *) get_slot(ring, off);

  real_slot_size = ring->slot_header_len + hdr->caplen;

  //TODO extended_hdr.parsed_header
  //if(ring->slot_header_len == sizeof(struct pfring_pkthdr)) /* !quick_mode */
  //  real_slot_size += hdr->extended_hdr.parsed_header_len;

  if((off + real_slot_size + ring->slots_info->slot_len) > (ring->slots_info->tot_mem - sizeof(FlowSlotInfo)))
    return 0;

  return (off + real_slot_size);
}

/* ******************************* */

static inline u_int32_t num_queued_pkts(pfring *ring)
{
  u_int32_t tot_insert = ring->slots_info->tot_insert;
  u_int32_t tot_read =   ring->slots_info->tot_read;

  if(tot_insert >= tot_read) 
    return (tot_insert - tot_read);
  else 
    return(((u_int32_t) - 1) + tot_insert - tot_read);
}

/* ******************************* */

static inline int check_and_init_free_slot(pfring *ring, int off)
{
  if(ring->slots_info->insert_off == ring->slots_info->remove_off) {
    if(num_queued_pkts(ring) >= ring->slots_info->min_num_slots)
      return 0;
  } else {
    if(ring->slots_info->insert_off < ring->slots_info->remove_off) {
      if((ring->slots_info->remove_off - ring->slots_info->insert_off) < (2 * ring->slots_info->slot_len))
	return 0;
    } else {
      if ((ring->slots_info->tot_mem - sizeof(FlowSlotInfo) - ring->slots_info->insert_off) < (2 * ring->slots_info->slot_len) &&
          ring->slots_info->remove_off == 0)
	return 0;
    }
  }

  return 1;
}

/* ******************************* */

static inline int copy_data_to_ring(pfring *ring, struct pfring_pkthdr *pkt_hdr, void *pkt, uint pkt_len) {
  struct pfring_pkthdr *hdr;
  char *ring_bucket;
  u_int32_t off;

  off = ring->slots_info->insert_off;
  ring->slots_info->tot_pkts++;

  if(!check_and_init_free_slot(ring, off)) {
    ring->slots_info->tot_lost++;
    return -1;
  }

  ring_bucket = get_slot(ring, off);
  hdr = (struct pfring_pkthdr *) ring_bucket;


  if (pkt_hdr != NULL) {
    memcpy(hdr, pkt_hdr, ring->slot_header_len);
    //TODO extended_hdr.parsed_header
  } else {
    memset(hdr, 0, ring->slot_header_len);
    //TODO
    //gettimeofday(&hdr->ts, NULL);
    //if (ring->slot_header_len == sizeof(struct pfring_pkthdr)) /* !quick_mode */
    //  hdr->extended_hdr.if_index = FAKE_PACKET;
  }

  hdr->len = pkt_len;
  hdr->caplen = min_val(pkt_len, ring->caplen);
  memcpy(&ring_bucket[ring->slot_header_len], pkt, hdr->caplen);

  ring->slots_info->insert_off = get_next_slot_offset(ring, off);

  /*
    NOTE: smp_* barriers are _compiler_ barriers on UP, mandatory barriers on SMP
    a consumer _must_ see the new value of tot_insert only after the buffer update completes
  */
  //smp_mb();
  gcc_mb();

  ring->slots_info->tot_insert++;

  return 1;
}

/* ******************************* */

static inline void pfring_mod_usring_signal(pfring *ring, u_int8_t flush_packet) {
  if (!(ring->slots_info->userspace_ring_flags & USERSPACE_RING_NO_INTERRUPT)) {

#ifdef USE_WATERMARK
    if(!flush_packet && ring->dna.num_tx_pkts_before_dna_sync < ring->dna.dna_tx_sync_watermark)
      ring->dna.num_tx_pkts_before_dna_sync++;
    else {
      ring->dna.num_tx_pkts_before_dna_sync = 0;
#endif

#ifdef PROFILING
      ticks tick_start = getticks();
#endif

      sendto(ring->fd, NULL, 0, 0, NULL, 0);

#ifdef PROFILING
      tick_delta_tot += (getticks() - tick_start); 
      n_call_tot++;
#endif

#ifdef USE_WATERMARK
    }
#endif

  }
}

/* ******************************* */

int pfring_mod_usring_enqueue(pfring *ring, char *pkt, u_int pkt_len, u_int8_t flush_packet) {
  int rc;

  rc = copy_data_to_ring(ring, NULL, pkt, pkt_len);

  if (rc == 1)
    pfring_mod_usring_signal(ring, flush_packet);

  return rc;
}

/* ******************************* */

int pfring_mod_usring_enqueue_parsed(pfring *ring, char *pkt, struct pfring_pkthdr *hdr, u_int8_t flush_packet) {
  int rc;

  rc = copy_data_to_ring(ring, hdr, pkt, hdr->len);

  if (rc == 1)
    pfring_mod_usring_signal(ring, flush_packet);

  return rc;
}

