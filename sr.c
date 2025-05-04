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

int ComputeChecksum(struct pkt packet) {
  int checksum = packet.seqnum + packet.acknum;
  for (int i = 0; i < 20; i++) checksum += (int)(packet.payload[i]);
  return checksum;
}

bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

void starttimer_sr(int AorB, double increment, int index) {
  if (TRACE > 1)
    printf("----Starting timer for packet at window index %d\n", index);
  timers[index] = true;
  starttimer(AorB, increment);
}

void stoptimer_sr(int AorB, int index) {
  if (TRACE > 1)
    printf("----Stopping timer for packet at window index %d\n", index);
  timers[index] = false;
  stoptimer(AorB);
}

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
