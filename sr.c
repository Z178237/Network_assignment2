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
  for (int i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);
  return checksum;
}

bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

void starttimer_sr(int AorB, double increment, int index) {
  /* Only start a timer if one is not already running for this packet */
  if (!timers[index]) {
    timers[index] = true;
    starttimer(AorB, increment);
    if (TRACE > 1) {
      printf("          START TIMER: starting timer at %f\n", time);
    }
  }
}

void stoptimer_sr(int AorB, int index) {
  /* Only stop a timer if one is running for this packet */
  if (timers[index]) {
    timers[index] = false;
    stoptimer(AorB);
    if (TRACE > 1) {
      printf("          STOP TIMER: stopping timer at %f\n", time);
    }
  }
}

void manage_timers(void) {
  for (int i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (!acked[idx] && !timers[idx]) {
      starttimer_sr(A, RTT, idx);
      break;
    }
  }
}

int get_buffer_index(int seqnum) {
  for (int i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (buffer[idx].seqnum == seqnum) return idx;
  }
  return -1;
}

void slide_window(void) {
  int slide = 0;
  for (int i = 0; i < windowcount; i++) {
    if (acked[(windowbase + i) % WINDOWSIZE])
      slide++;
    else
      break;
  }
  if (slide > 0) {
    int old_base = windowbase;
    windowbase = (windowbase + slide) % WINDOWSIZE;
    windowcount -= slide;
    for (int i = 0; i < slide; i++) {
      int idx = (old_base + i) % WINDOWSIZE;
      timers[idx] = false;
      acked[idx] = false;
    }
  }
}

void A_output(struct msg message) {
  if (windowcount < WINDOWSIZE) {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is not full, send new message to layer3!\n");

    struct pkt sendpkt;
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (int i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    int buffer_index = (windowbase + windowcount) % WINDOWSIZE;
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
  if (!IsCorrupted(packet)) {
    total_ACKs_received++;

    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);

    int idx = get_buffer_index(packet.acknum);
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
  if (windowcount <= 0) return;

  for (int i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (timers[idx]) {
      if (TRACE > 0)
        printf("----A: Timer expired, resending packet %d\n", buffer[idx].seqnum);
      tolayer3(A, buffer[idx]);
      packets_resent++;
      timers[idx] = false;
      starttimer_sr(A, RTT, idx);
      break;
    }
  }
  if (windowcount > 0)
    manage_timers();
}

void A_init(void) {
  A_nextseqnum = 0;
  windowbase = 0;
  windowcount = 0;
  for (int i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
    timers[i] = false;
  }
}

bool is_in_window(int seqnum) {
  int ub = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;
  if (rcv_base <= ub) {
    return (seqnum >= rcv_base && seqnum <= ub);
  } else {
    return (seqnum >= rcv_base || seqnum <= ub);
  }
}

int get_receiver_index(int seqnum) {
  return (seqnum - rcv_base + WINDOWSIZE) % WINDOWSIZE;
}

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
  if (!IsCorrupted(packet) && is_in_window(packet.seqnum)) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);

    int idx = get_receiver_index(packet.seqnum);
    if (!received[idx]) {
      received[idx] = true;
      buffered[idx] = packet;
      packets_received++;
      if (packet.seqnum == rcv_base)
        deliver_buffered_packets();
    }

    struct pkt ackpkt;
    ackpkt.seqnum = B_nextseqnum;
    ackpkt.acknum = packet.seqnum;
    memset(ackpkt.payload, 0, 20);
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
    B_nextseqnum = (B_nextseqnum + 1) % 2;
  }
}

void B_init(void) {
  rcv_base = 0;
  B_nextseqnum = 1;
  for (int i = 0; i < WINDOWSIZE; i++)
    received[i] = false;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
