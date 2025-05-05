/* sr.c - Selective Repeat ARQ protocol implementation */

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "emulator.h"
#include "sr.h"

#define RTT  16.0
#define WINDOWSIZE 6
#define SEQSPACE 12
#define NOTINUSE (-1)

/* Sender-side state */
static struct pkt buffer[WINDOWSIZE];
static int windowbase;
static int windowcount;
static int A_nextseqnum;
static bool acked[WINDOWSIZE];
static bool timers[WINDOWSIZE];

/* Receiver-side state */
static int rcv_base;
static bool received[WINDOWSIZE];
static struct pkt buffered[WINDOWSIZE];

/* Statistics counters - declared extern in emulator.h */
extern int window_full;
extern int total_ACKs_received;
extern int new_ACKs;
extern int packets_resent;
extern int packets_received;

/* Access to emulator simulated time */
extern float time;

int ComputeChecksum(struct pkt packet) {
  int checksum = packet.seqnum + packet.acknum;
  int i;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);
  return checksum;
}

bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

/* Start timer for a specific packet */
void starttimer_sr(int AorB, float increment, int index) {
    if (timers[index]) {
        /* Timer is already running, do not restart */
        return;
    }
    timers[index] = true;
    starttimer(AorB, increment);
}

/* Stop timer for a specific packet */
void stoptimer_sr(int AorB, int index) {
    if (!timers[index]) {
        /* Timer is already stopped */
        return;
    }
    timers[index] = false;
    stoptimer(AorB);
}

/* Find a running timer and start it if not currently running */
void manage_timers(void) {
  int i;
  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (!acked[idx] && !timers[idx]) {
      starttimer_sr(A, RTT, idx);
      break;  /* Only start one timer at a time */
    }
  }
}

/* Find buffer index for a given sequence number */
int get_buffer_index(int seqnum) {
  int i;
  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (buffer[idx].seqnum == seqnum) return idx;
  }
  return -1;
}

/* Slide the window forward for all acknowledged packets */
void slide_window(void) {
  int slide = 0;
  int i;
  for (i = 0; i < windowcount; i++) {
    if (acked[(windowbase + i) % WINDOWSIZE])
      slide++;
    else
      break;
  }
  if (slide > 0) {
    int old_base = windowbase;
    windowbase = (windowbase + slide) % WINDOWSIZE;
    windowcount -= slide;
    for (i = 0; i < slide; i++) {
      int idx = (old_base + i) % WINDOWSIZE;
      if (timers[idx]) {
        stoptimer_sr(A, idx);  /* Stop any running timers */
      }
      timers[idx] = false;  /* Clear timer flags for shifted packets */
      acked[idx] = false;   /* Clear acked flags for shifted packets */
    }
  }
}

void A_output(struct msg message) {
  struct pkt sendpkt;
  int i, buffer_index;

  if (windowcount < WINDOWSIZE) {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    buffer_index = (windowbase + windowcount) % WINDOWSIZE;
    buffer[buffer_index] = sendpkt;
    acked[buffer_index] = false;
    windowcount++;

    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);

    tolayer3(A, sendpkt);
    
    /* Start timer for this packet if no timer is running */
    if (TRACE > 1)
      printf("          START TIMER: starting timer at %f\n", time);
    
    starttimer_sr(A, RTT, buffer_index);
    
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  } else {
    window_full++;
  }
}

void A_input(struct pkt packet) {
  int idx;

  if (!IsCorrupted(packet)) {
    total_ACKs_received++;

    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);

    idx = get_buffer_index(packet.acknum);
    if (idx != -1 && !acked[idx]) {
      if (TRACE > 0)
        printf("----A: ACK %d is not a duplicate\n", packet.acknum);

      acked[idx] = true;
      new_ACKs++;
      
      if (timers[idx]) {
        if (TRACE > 1)
          printf("          STOP TIMER: stopping timer at %f\n", time);
        stoptimer_sr(A, idx);
      }
      
      slide_window();
      
      /* Check if we need to start a timer for any unacked packets */
      if (windowcount > 0) {
        int has_timer_running = 0;
        for (int i = 0; i < windowcount; i++) {
          int idx = (windowbase + i) % WINDOWSIZE;
          if (timers[idx]) {
            has_timer_running = 1;
            break;
          }
        }
        
        if (!has_timer_running) {
          for (int i = 0; i < windowcount; i++) {
            int idx = (windowbase + i) % WINDOWSIZE;
            if (!acked[idx]) {
              if (TRACE > 1)
                printf("          START TIMER: starting timer at %f\n", time);
              starttimer_sr(A, RTT, idx);
              break;
            }
          }
        }
      }
    }
  }
}

void A_timerinterrupt(void) {
  int i;

  if (windowcount <= 0) return;

  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (timers[idx]) {
      if (TRACE > 0)
        printf("----A: Timer expired, resending packet %d\n", buffer[idx].seqnum);
      tolayer3(A, buffer[idx]);
      packets_resent++;
      timers[idx] = false;
      
      if (TRACE > 1)
        printf("          START TIMER: starting timer at %f\n", time);
      starttimer_sr(A, RTT, idx);
      
      return; /* Only handle one timer at a time */
    }
  }
}

void A_init(void) {
  int i;
  A_nextseqnum = 0;
  windowbase = 0;
  windowcount = 0;

  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
    timers[i] = false;
  }
}

/* Returns true if seqnum is within the current receive window */
bool is_in_window(int seqnum) {
  int upper_bound = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;
  
  if (rcv_base <= upper_bound) {
    /* Window doesn't wrap around SEQSPACE */
    return (seqnum >= rcv_base && seqnum <= upper_bound);
  } else {
    /* Window wraps around SEQSPACE */
    return (seqnum >= rcv_base || seqnum <= upper_bound);
  }
}

/* Converts a seqnum to an index in the receiver's buffer */
int get_receiver_index(int seqnum) {
  return (seqnum - rcv_base + WINDOWSIZE) % WINDOWSIZE;
}

/* Deliver packets that have been received in order */
void deliver_buffered_packets(void) {
  int idx;
  
  while (1) {
    idx = get_receiver_index(rcv_base);
    if (received[idx]) {
      tolayer5(B, buffered[idx].payload);
      received[idx] = false;
      rcv_base = (rcv_base + 1) % SEQSPACE;
    } else {
      break;
    }
  }
}

void B_input(struct pkt packet) {
  struct pkt ackpkt;
  int idx;

  if (!IsCorrupted(packet) && is_in_window(packet.seqnum)) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);

    idx = get_receiver_index(packet.seqnum);
    if (!received[idx]) {
      received[idx] = true;
      buffered[idx] = packet;
      packets_received++;
      
      if (packet.seqnum == rcv_base) {
        deliver_buffered_packets();
      }
    }

    /* Send ACK */
    ackpkt.seqnum = NOTINUSE;  /* Not using sequence numbers for ACKs */
    ackpkt.acknum = packet.seqnum;
    memset(ackpkt.payload, 0, 20);
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
  }
}

void B_init(void) {
  int i;
  rcv_base = 0;
  for (i = 0; i < WINDOWSIZE; i++)
    received[i] = false;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}