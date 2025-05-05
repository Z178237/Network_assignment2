
 //* sr.c - Selective Repeat protocol implementation.
 

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

#define RTT  16.0              // Timeout interval in simulated time units
#define WINDOWSIZE 6          // Number of outstanding unacknowledged packets allowed
#define SEQSPACE 12           // Total sequence number space (must be > 2 * WINDOWSIZE)
#define NOTINUSE (-1)         // Special value for unused acknum

// Sender-side state
static struct pkt buffer[WINDOWSIZE];      // Outgoing packet buffer
static int windowbase;                     // Sequence number of oldest unacked packet
static int windowcount;                    // Number of unacked packets in window
static int A_nextseqnum;                   // Next sequence number to use
static bool acked[WINDOWSIZE];             // Tracks which packets were ACKed
static bool timers[WINDOWSIZE];            // Tracks which timers are active

// Receiver-side state
static int rcv_base;                       // Sequence number expected next at receiver
static bool received[WINDOWSIZE];          // Marks which packets were received within the window
static struct pkt buffered[WINDOWSIZE];    // Packet buffer for reordering
static int B_nextseqnum;                   // Alternating bit for ACK packet sequence numbers (not used logically)

// Shared statistics variables declared in emulator.h
int window_full;
int total_ACKs_received;
int new_ACKs;
int packets_resent;
int packets_received;

// Calculates checksum based on packet content
int ComputeChecksum(struct pkt packet) {
  int checksum = packet.seqnum + packet.acknum;
  int i;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);
  return checksum;
}

// Verifies if a packet is corrupted
bool IsCorrupted(struct pkt packet) {
  return packet.checksum != ComputeChecksum(packet);
}

// Starts the timer for the specified index
void starttimer_sr(int AorB, double increment, int index) {
  if (TRACE > 1)
    printf("----Starting timer for packet at window index %d\n", index);
  timers[index] = true;
  starttimer(AorB, increment);
}

// Stops the timer for the specified index
void stoptimer_sr(int AorB, int index) {
  if (TRACE > 1)
    printf("----Stopping timer for packet at window index %d\n", index);
  timers[index] = false;
  stoptimer(AorB);
}

// Ensures at least one timer is active for unacked packets
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

// Returns buffer index matching given seqnum (or -1 if not found)
int get_buffer_index(int seqnum) {
  int i;
  for (i = 0; i < windowcount; i++) {
    int idx = (windowbase + i) % WINDOWSIZE;
    if (buffer[idx].seqnum == seqnum) return idx;
  }
  return -1;
}

// Moves the send window forward past ACKed packets
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

// Called when data arrives from the application layer
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

// Called when an ACK is received
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

// Timeout handler: retransmit the oldest unacked packet
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

// Initializes sender-side variables
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

// Checks if a sequence number is in the receiver's window
bool is_in_window(int seqnum) {
  int ub = (rcv_base + WINDOWSIZE - 1) % SEQSPACE;
  return (rcv_base <= ub) ? (seqnum >= rcv_base && seqnum <= ub)
                          : (seqnum >= rcv_base || seqnum <= ub);
}

// Maps a seqnum to an index in the receiver's buffer
int get_receiver_index(int seqnum) {
  return (seqnum - rcv_base + WINDOWSIZE) % WINDOWSIZE;
}

// Delivers in-order packets from receiver buffer to layer5
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

// Called when packet arrives at receiver
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
    for (i = 0; i < 20; i++)
      ackpkt.payload[i] = '0';
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
    B_nextseqnum = (B_nextseqnum + 1) % 2;
  }
}

// Initializes receiver-side state
void B_init(void) {
  int i;
  rcv_base = 0;
  B_nextseqnum = 1;
  for (i = 0; i < WINDOWSIZE; i++)
    received[i] = false;
}

// Not used in SR; required by API
void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
