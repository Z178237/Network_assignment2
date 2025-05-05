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
static int B_nextseqnum;

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

void starttimer_sr(int AorB, double increment, int index) {
  if (TRACE > 1)
    printf("          START TIMER: starting timer at %f\n", increment);
  timers[index] = true;
  starttimer(AorB, increment);
}

void stoptimer_sr(int AorB, int index) {
  if (TRACE > 1)
    printf("          STOP TIMER: stopping timer at %f\n", RTT);
  timers[index] = false;
  stoptimer(AorB);
}

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

int get_buffer_index(int seqnum) {
  int i;
  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (buffer[idx].seqnum == seqnum) return idx;
  }
  return -1;
}

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
      if (timers[idx])
        stoptimer_sr(A, idx);
      slide_window();
      if (windowcount > 0)
        manage_timers();
    }
  }
}

void A_timerinterrupt(void) {
  int i;
  int found_timer = 0;

  if (windowcount <= 0) return;

  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (timers[idx]) {
      if (TRACE > 0)
        printf("----A: Timer expired, resending packet %d\n", buffer[idx].seqnum);
      tolayer3(A, buffer[idx]);
      packets_resent++;
      timers[idx] = false;
      starttimer_sr(A, RTT, idx);
      found_timer = 1;
      break; /* Only resend one packet per timer interrupt */
    }
  }

  if (!found_timer && windowcount > 0)
    manage_timers();
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
  int ub = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;
  if (rcv_base <= ub) {
    return (seqnum >= rcv_base && seqnum <= ub);
  } else {
    return (seqnum >= rcv_base || seqnum <= ub);
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
    ackpkt.seqnum = B_nextseqnum;
    ackpkt.acknum = packet.seqnum;
    memset(ackpkt.payload, 0, 20);
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
    B_nextseqnum = (B_nextseqnum + 1) % 2;
  }
}

void B_init(void) {
  int i;
  rcv_base = 0;
  B_nextseqnum = 1;
  for (i = 0; i < WINDOWSIZE; i++)
    received[i] = false;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}