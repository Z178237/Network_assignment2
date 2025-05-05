// sr.c - Selective Repeat implementation based on Go-Back-N
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol. Adapted from GBN implementation.
**********************************************************************/

#define RTT  16.0       /* round trip time. MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 12     /* the sequence space for SR must be at least 2*windowsize */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/********* Utility functions ************/

/* compute checksum for a packet */
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i=0; i<20; i++)
    checksum += (int)(packet.payload[i]);

  return checksum;
}

/* check if a packet is corrupted */
bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}

/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];   /* array for storing packets waiting for ACK */
static int A_nextseqnum;                /* the next sequence number to be used by the sender */
static bool A_timer_status[WINDOWSIZE]; /* timer status for each packet in the window */
static bool A_ack_received[WINDOWSIZE]; /* track which packets have been acknowledged */
static int A_base;                      /* base of the sending window */

/* Initialize sender variables */
void A_init(void)
{
  int i;
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  A_base = 0;
  
  /* Initialize all timers as stopped and packets as unacknowledged */
  for (i = 0; i < WINDOWSIZE; i++) {
    A_timer_status[i] = false;    /* timer is off */
    A_ack_received[i] = true;     /* mark as acked so slot can be used */
  }
}

/* Create and send a packet */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;
  int window_index;

  /* if there's space in the window */
  if ((A_nextseqnum < A_base + WINDOWSIZE) && 
      (A_nextseqnum - A_base < SEQSPACE)) {
    
    if (TRACE > 1)
      printf("----A: New message arrives, send window has space, sending packet %d\n", A_nextseqnum);

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* calculate position in the buffer */
    window_index = A_nextseqnum % WINDOWSIZE;
    
    /* store packet in window buffer */
    buffer[window_index] = sendpkt;
    A_ack_received[window_index] = false;
    
    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);
    
    /* start timer for this specific packet if not already running */
    if (!A_timer_status[window_index]) {
      A_timer_status[window_index] = true;
      starttimer(A, RTT);
    }
    
    /* increment next sequence number */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}

/* Handle incoming ACKs */
void A_input(struct pkt packet)
{
  int window_index;
  int i;
  
  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;
    
    /* Check if ACK is within the current window */
    if ((packet.acknum >= A_base) && (packet.acknum < A_base + WINDOWSIZE) ||
        (A_base + WINDOWSIZE > SEQSPACE && packet.acknum < (A_base + WINDOWSIZE) % SEQSPACE)) {
      
      /* This is a new ACK */
      window_index = packet.acknum % WINDOWSIZE;
      
      if (!A_ack_received[window_index]) {
        if (TRACE > 0)
          printf("----A: ACK %d is new\n", packet.acknum);
        
        /* Mark packet as acknowledged */
        A_ack_received[window_index] = true;
        new_ACKs++;
        
        /* Stop the timer for this packet */
        if (A_timer_status[window_index]) {
          A_timer_status[window_index] = false;
          stoptimer(A);
        }
        
        /* Check if we can slide window */
        while (A_ack_received[A_base % WINDOWSIZE]) {
          /* Mark this slot as available */
          A_ack_received[A_base % WINDOWSIZE] = true;
          
          /* Advance base pointer */
          A_base = (A_base + 1) % SEQSPACE;
          
          if (A_base == A_nextseqnum)
            break;
        }
        
        /* Restart timers for unacknowledged packets in window */
        for (i = 0; i < WINDOWSIZE; i++) {
          int seq = (A_base + i) % SEQSPACE;
          window_index = seq % WINDOWSIZE;
          
          if (seq != A_nextseqnum && !A_ack_received[window_index]) {
            if (!A_timer_status[window_index]) {
              A_timer_status[window_index] = true;
              starttimer(A, RTT);
              break; /* Start only one timer */
            }
          }
        }
      }
      else {
        if (TRACE > 0)
          printf("----A: duplicate ACK received for %d, ignoring\n", packet.acknum);
      }
    }
    else {
      if (TRACE > 0)
        printf("----A: ACK %d outside current window, ignoring\n", packet.acknum);
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, ignoring\n");
  }
}

/* Handle timer interrupts - Selective Repeat only retransmits the specific timed-out packet */
void A_timerinterrupt(void)
{
  int i;
  bool timer_restarted = false;
  
  if (TRACE > 0)
    printf("----A: timer interrupt, checking for packets to retransmit\n");
  
  /* Find unacknowledged packets in the window and retransmit */
  for (i = 0; i < WINDOWSIZE; i++) {
    int seq = (A_base + i) % SEQSPACE;
    int window_index = seq % WINDOWSIZE;
    
    if (seq != A_nextseqnum && !A_ack_received[window_index]) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", seq);
      
      tolayer3(A, buffer[window_index]);
      packets_resent++;
      
      /* Restart timer for this packet */
      if (!timer_restarted) {
        starttimer(A, RTT);
        timer_restarted = true;
      }
    }
  }
}

/********* Receiver (B) variables and functions ************/

static int B_base;                /* base of the receiving window */
static int B_nextseqnum;          /* sequence number for B's packets */
static bool B_received[WINDOWSIZE]; /* track which packets have been received */
static struct pkt B_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */

/* Initialize receiver variables */
void B_init(void)
{
  int i;
  B_base = 0;
  B_nextseqnum = 0;
  
  /* Initialize all packets as not received */
  for (i = 0; i < WINDOWSIZE; i++) {
    B_received[i] = false;
  }
}

/* Handle incoming packets */
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int window_index;
  int i;
  
  /* if not corrupted and within receiving window */
  if (!IsCorrupted(packet) &&
      ((packet.seqnum >= B_base) && (packet.seqnum < B_base + WINDOWSIZE) ||
       (B_base + WINDOWSIZE > SEQSPACE && packet.seqnum < (B_base + WINDOWSIZE) % SEQSPACE))) {
    
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, sending ACK\n", packet.seqnum);
    
    /* Store the packet and mark as received */
    window_index = packet.seqnum % WINDOWSIZE;
    B_buffer[window_index] = packet;
    B_received[window_index] = true;
    
    /* Send ACK for this packet */
    sendpkt.acknum = packet.seqnum;
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2; /* Alternating bit for B's seq numbers */
    
    /* Fill payload with zeros */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';
    
    sendpkt.checksum = ComputeChecksum(sendpkt);
    
    /* Send ACK */
    tolayer3(B, sendpkt);
    
    /* Deliver in-order packets to the application layer */
    while (B_received[B_base % WINDOWSIZE]) {
      window_index = B_base % WINDOWSIZE;
      
      /* Deliver to application layer */
      tolayer5(B, B_buffer[window_index].payload);
      packets_received++;
      
      /* Mark as not received for future use */
      B_received[window_index] = false;
      
      /* Advance window */
      B_base = (B_base + 1) % SEQSPACE;
    }
  }
  else {
    if (TRACE > 0) {
      if (IsCorrupted(packet))
        printf("----B: packet %d is corrupted, sending ACK for last correctly received packet\n", packet.seqnum);
      else
        printf("----B: packet %d is out of window, sending ACK for last correctly received packet\n", packet.seqnum);
    }
    
    /* Create ACK for the last correctly received packet */
    sendpkt.acknum = (B_base - 1 + SEQSPACE) % SEQSPACE;
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % 2;
    
    /* Fill payload with zeros */
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';
    
    sendpkt.checksum = ComputeChecksum(sendpkt);
    
    /* Send ACK */
    tolayer3(B, sendpkt);
  }
}

/* The following functions are not used for unidirectional transfer */
void B_output(struct msg message)
{
}

void B_timerinterrupt(void)
{
}