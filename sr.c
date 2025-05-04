#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

#define RTT  16.0
#define WINDOWSIZE 6
#define SEQSPACE 12
#define NOTINUSE (-1)

static struct pkt buffer[WINDOWSIZE];
static int windowbase;
static int windowcount;
static int A_nextseqnum;

static int rcv_base;

int ComputeChecksum(struct pkt packet) {
  int checksum = packet.seqnum + packet.acknum;
  for (int i = 0; i < 20; i++) checksum += (int)(packet.payload[i]);
  return checksum;
}

bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

void A_output(struct msg message) {
  if (windowcount < WINDOWSIZE) {
    struct pkt sendpkt;
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (int i = 0; i < 20; i++) sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    buffer[windowcount] = sendpkt;
    tolayer3(A, sendpkt);
    starttimer(A, RTT);

    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
    windowcount++;
  }
}

void A_input(struct pkt packet) {
  if (!IsCorrupted(packet)) {
    stoptimer(A);
    windowcount--;
    windowbase = (windowbase + 1) % SEQSPACE;
  }
}

void A_timerinterrupt(void) {
  // GBN-style: resend all packets in window
  for (int i = 0; i < windowcount; i++) {
    tolayer3(A, buffer[i]);
  }
  starttimer(A, RTT);
}

void A_init(void) {
  A_nextseqnum = 0;
  windowbase = 0;
  windowcount = 0;
}

void B_input(struct pkt packet) {
  if (!IsCorrupted(packet)) {
    tolayer5(B, packet.payload);

    struct pkt ackpkt;
    ackpkt.seqnum = 0;
    ackpkt.acknum = packet.seqnum;
    for (int i = 0; i < 20; i++) ackpkt.payload[i] = '0';
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
  }
}

void B_init(void) {
  rcv_base = 0;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}