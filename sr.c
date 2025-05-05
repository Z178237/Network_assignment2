// sr.c
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

#define RTT  16.0
#define WINDOWSIZE 6
#define SEQSPACE 12
#define NOTINUSE (-1)

// Sender-side state variables
static struct pkt buffer[WINDOWSIZE];
static int windowbase;
static int windowcount;
static int A_nextseqnum;
static bool acked[WINDOWSIZE];
static bool timers[WINDOWSIZE];

// Receiver-side state variables
static int rcv_base;
static bool received[WINDOWSIZE];
static struct pkt buffered[WINDOWSIZE];
static int B_nextseqnum;

// Basic checksum calculator for error detection
int ComputeChecksum(struct pkt packet) {
  int checksum = packet.seqnum + packet.acknum;
  for (int i = 0; i < 20; i++) checksum += (int)(packet.payload[i]);
  return checksum;
}

// Checks if a received packet is corrupted using checksum comparison
bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

// Starts the timer for a packet at a specific index
void starttimer_sr(int AorB, double increment, int index) {
  if (TRACE > 1)
    printf("----Starting timer for packet at window index %d\n", index);
  timers[index] = true;
  starttimer(AorB, increment);
}

// Stops the logical timer for a given packet index
void stoptimer_sr(int AorB, int index) {
  if (TRACE > 1)
    printf("----Stopping timer for packet at window index %d\n", index);
  timers[index] = false;
  stoptimer(AorB);
}

// Scans the send window and ensures at least one unACKed packet has a running timer
void manage_timers(void) {
  for (int i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (!acked[idx] && !timers[idx]) {
      starttimer_sr(A, RTT, idx);
      return;
    }
  }
  for (int i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (!acked[idx]) {
      starttimer_sr(A, RTT, idx);
      return;
    }
  }
}

// Finds buffer index matching the sequence number
int get_buffer_index(int seqnum) {
  for (int i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (buffer[idx].seqnum == seqnum) return idx;
  }
  return -1;
}

// Slides sender window forward after successful ACKs
void slide_window(void) {
  int slide = 0;
  for (int i = 0; i < windowcount; i++) {
    if (acked[(windowbase + i) % WINDOWSIZE]) slide++;
    else break;
  }
  if (slide > 0) {
    windowbase = (windowbase + slide) % WINDOWSIZE;
    windowcount -= slide;
    for (int i = 0; i < slide; i++) {
      int idx = (windowbase - slide + i + WINDOWSIZE) % WINDOWSIZE;
      timers[idx] = false;
      acked[idx] = false;
    }
  }
}

// Sends packet from application layer if window is not full
void A_output(struct msg message) {
  struct pkt sendpkt;
  int i, buffer_index;
  if (windowcount < WINDOWSIZE) {
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++) sendpkt.payload[i] = message.data[i];
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

// Processes incoming ACKs and updates sender state
void A_input(struct pkt packet) {
  if (!IsCorrupted(packet)) {
    total_ACKs_received++;
    int idx = get_buffer_index(packet.acknum);
    if (idx != -1 && !acked[idx]) {
      acked[idx] = true;
      new_ACKs++;
      if (timers[idx]) stoptimer_sr(A, idx);
      slide_window();
      if (windowcount > 0) manage_timers();
    }
  }
}

// Retransmits packet on timeout and restarts timer
void A_timerinterrupt(void) {
  for (int i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (timers[idx]) {
      if (TRACE > 0)
        printf("----A: resending packet %d\n", buffer[idx].seqnum);
      tolayer3(A, buffer[idx]);
      packets_resent++;
      timers[idx] = false;
      starttimer_sr(A, RTT, idx);
      return;
    }
  }
  if (windowcount > 0) manage_timers();
}

// Initializes sender state
void A_init(void) {
  A_nextseqnum = 0;
  windowbase = 0;
  windowcount = 0;
  for (int i = 0; i < WINDOWSIZE; i++) {
    acked[i] = false;
    timers[i] = false;
  }
}

// Checks if a seqnum falls within the current receiver window
bool is_in_window(int seqnum) {
  int ub = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;
  return (rcv_base <= ub) ? (seqnum >= rcv_base && seqnum <= ub)
                          : (seqnum >= rcv_base || seqnum <= ub);
}

// Converts sequence number to receiver buffer index
int get_receiver_index(int seqnum) {
  return (seqnum - rcv_base + WINDOWSIZE) % WINDOWSIZE;
}

// Delivers in-order buffered packets to the application layer
void deliver_buffered_packets(void) {
  bool delivered = true;
  while (delivered) {
    int idx = get_receiver_index(rcv_base);
    if (received[idx]) {
      if (TRACE > 0)
        printf("----B: delivering buffered packet %d to application layer\n", rcv_base);
      tolayer5(B, buffered[idx].payload);
      received[idx] = false;
      rcv_base = (rcv_base + 1) % SEQSPACE;
      packets_received++;
    } else delivered = false;
  }
}

// Handles packet reception at B, sends ACK and buffers if valid
void B_input(struct pkt packet) {
  struct pkt ackpkt;
  if (!IsCorrupted(packet) && is_in_window(packet.seqnum)) {
    if (TRACE > 0)
      printf("----B: packet %d is correctly received and in window, send ACK!\n", packet.seqnum);
    int idx = get_receiver_index(packet.seqnum);
    if (!received[idx]) {
      received[idx] = true;
      buffered[idx] = packet;
      packets_received++;
      if (packet.seqnum == rcv_base) deliver_buffered_packets();
    }
    ackpkt.seqnum = B_nextseqnum;
    ackpkt.acknum = packet.seqnum;
    for (int i = 0; i < 20; i++) ackpkt.payload[i] = '0';
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
    B_nextseqnum = (B_nextseqnum + 1) % 2;
  }
}

// Initializes receiver state
void B_init(void) {
  rcv_base = 0;
  B_nextseqnum = 1;
  for (int i = 0; i < WINDOWSIZE; i++) received[i] = false;
}

// Not used in this implementation
void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
