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
int window_full;
int total_ACKs_received;
int new_ACKs;
int packets_resent;
int packets_received;

/* Compute checksum for a packet */
int ComputeChecksum(struct pkt packet) {
  int checksum = packet.seqnum + packet.acknum;
  int i;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);
  return checksum;
}

/* Check whether a packet is corrupted */
bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

/* Start the timer for a specific packet */
void starttimer_sr(int AorB, double increment, int index) {
  if (TRACE > 1)
    printf("----Starting timer for packet at window index %d\n", index);
  timers[index] = true;
  starttimer(AorB, increment);
}

/* Stop the timer for a specific packet */
void stoptimer_sr(int AorB, int index) {
  if (TRACE > 1)
    printf("----Stopping timer for packet at window index %d\n", index);
  timers[index] = false;
  stoptimer(AorB);
}

/*
 * Ensures at least one outstanding unACKed packet has an active timer.
 * Starts the timer for the earliest such packet.
 */
void manage_timers(void) {
  int i;
  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (!acked[idx] && !timers[idx]) {
      starttimer_sr(A, RTT, idx);
      return;
    }
  }
  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (!acked[idx]) {
      starttimer_sr(A, RTT, idx);
      return;
    }
  }
}

/* Get index of packet in buffer based on seqnum */
int get_buffer_index(int seqnum) {
  int i;
  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (buffer[idx].seqnum == seqnum) return idx;
  }
  return -1;
}

/*
 * Slides the sender window forward by counting how many packets
 * have been ACKed at the front of the window.
 * Resets their state to reuse the buffer space.
 */
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
    int idx;
    windowbase = (windowbase + slide) % WINDOWSIZE;
    windowcount -= slide;
    for (i = 0; i < slide; i++) {
      idx = (windowbase - slide + i + WINDOWSIZE) % WINDOWSIZE;
      timers[idx] = false;
      acked[idx] = false;
    }
  }
}

/*
 * Called by the application layer to send a message.
 * If the window is not full, the message is packaged and sent.
 * Otherwise, we increment the overflow counter.
 */
void A_output(struct msg message) {
  struct pkt sendpkt;
  int i, buffer_index;

  if (windowcount < WINDOWSIZE) {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is not full, sending to layer3\n");

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

/*
 * Called when an ACK packet is received.
 * If the ACK is valid and new, mark the corresponding packet as acknowledged,
 * stop its timer, and slide the window forward.
 */
void A_input(struct pkt packet) {
  int idx;

  if (!IsCorrupted(packet)) {
    total_ACKs_received++;

    if (TRACE > 0)
      printf("----A: ACK %d received and valid\n", packet.acknum);

    idx = get_buffer_index(packet.acknum);
    if (idx != -1 && !acked[idx]) {
      if (TRACE > 0)
        printf("----A: ACK %d is new\n", packet.acknum);

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

/*
 * Timer interrupt handler.
 * Retransmits the first outstanding unACKed packet and restarts its timer.
 */
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
      return;
    }
  }

  if (!found_timer && windowcount > 0)
    manage_timers();
}

/* Initialize sender-side variables */
void A_init(void) {
  int i;
  A_nextseqnum = 0;
  windowbase = 0;
  windowcount = 0;
  window_full = 0;
  total_ACKs_received = 0;
  new_ACKs = 0;
  packets_resent = 0;
  packets_received = 0;

  for (i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
    timers[i] = false;
  }
}

/*
 * Checks if the given sequence number is within the current receiver window.
 * Handles wrap-around logic using modular arithmetic.
 */
bool is_in_window(int seqnum) {
  int ub = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;
  return (rcv_base <= ub) ? (seqnum >= rcv_base && seqnum <= ub)
                          : (seqnum >= rcv_base || seqnum <= ub);
}

/* Map sequence number to receiver buffer index */
int get_receiver_index(int seqnum) {
  return (seqnum - rcv_base + WINDOWSIZE) % WINDOWSIZE;
}

/*
 * Delivers any consecutively received packets (starting from rcv_base)
 * to the application layer.
 */
void deliver_buffered_packets(void) {
  bool delivered = true;
  while (delivered) {
    int idx = get_receiver_index(rcv_base);
    if (received[idx]) {
      if (TRACE > 0)
        printf("----B: Delivering packet %d to layer 5\n", rcv_base);
      tolayer5(B, buffered[idx].payload);
      received[idx] = false;
      rcv_base = (rcv_base + 1) % SEQSPACE;
    } else {
      delivered = false;
    }
  }
}

/*
 * Called when a packet arrives at the receiver.
 * If it's not corrupted and falls within the window, buffer it.
 * If it's in-order, deliver all consecutively buffered packets to layer 5.
 * Always send an ACK for correctly received packets.
 */
void B_input(struct pkt packet) {
  struct pkt ackpkt;
  int idx, i;

  if (!IsCorrupted(packet) && is_in_window(packet.seqnum)) {
    if (TRACE > 0)
      printf("----B: Packet %d received correctly\n", packet.seqnum);

    idx = get_receiver_index(packet.seqnum);
    if (!received[idx]) {
      received[idx] = true;
      buffered[idx] = packet;
      packets_received++;
      if (packet.seqnum == rcv_base)
        deliver_buffered_packets();
    }

    ackpkt.seqnum = B_nextseqnum;
    ackpkt.acknum = packet.seqnum;
    memset(ackpkt.payload, 0, 20); // 使用memset设置全零payload
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
    B_nextseqnum = (B_nextseqnum + 1) % 2;
  }
}

/* Initialize receiver-side state */
void B_init(void) {
  int i;
  rcv_base = 0;
  B_nextseqnum = 1;
  for (i = 0; i < WINDOWSIZE; i++)
    received[i] = false;
}

/* Not used in this SR implementation */
void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
