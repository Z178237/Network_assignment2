/* sr.c - Selective Repeat implementation based on Go-Back-N */
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
static bool A_timer_running[WINDOWSIZE]; /* array to track timers for each packet in window */
static bool A_ack_received[SEQSPACE];   /* track which packets have been acknowledged */
static int A_base;                      /* base of the sending window */
static bool A_timer_running_any;        /* indicates if any timer is running */

/* Initialize sender variables */
void A_init(void)
{
  int i;
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  A_base = 0;
  A_timer_running_any = false;
  
  /* Initialize all packets as acknowledged */
  for (i = 0; i < SEQSPACE; i++) {
    A_ack_received[i] = true;     /* mark as acked so slot can be used */
  }
  
  /* Initialize all timers as not running */
  for (i = 0; i < WINDOWSIZE; i++) {
    A_timer_running[i] = false;
  }
}

/* Create and send a packet */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if there's space in the window */
  if ((A_nextseqnum < A_base + WINDOWSIZE) || 
      (A_base + WINDOWSIZE > SEQSPACE && A_nextseqnum < (A_base + WINDOWSIZE) % SEQSPACE)) {
    
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* store packet in window buffer and mark as not acked */
    buffer[A_nextseqnum % WINDOWSIZE] = sendpkt;
    A_ack_received[A_nextseqnum] = false;
    
    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);
    
    /* start timer for this specific packet if not already running */
    if (!A_timer_running[A_nextseqnum % WINDOWSIZE]) {
      if (!A_timer_running_any) {
        starttimer(A, RTT);
        A_timer_running_any = true;
      }
      A_timer_running[A_nextseqnum % WINDOWSIZE] = true;
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
  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;
    
    /* Check if ACK is within the current window */
    if (((packet.acknum >= A_base) && (packet.acknum < A_base + WINDOWSIZE)) ||
        ((A_base + WINDOWSIZE > SEQSPACE) && (packet.acknum < (A_base + WINDOWSIZE) % SEQSPACE))) {
      
      /* If this packet hasn't been ACKed yet */
      if (!A_ack_received[packet.acknum]) {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        
        /* Mark packet as acknowledged */
        A_ack_received[packet.acknum] = true;
        A_timer_running[packet.acknum % WINDOWSIZE] = false;
        new_ACKs++;
        
        /* Check if we need to slide window */
        while (A_ack_received[A_base]) {
          A_ack_received[A_base] = false; /* Reset for future use */
          A_base = (A_base + 1) % SEQSPACE;
          
          /* If window is now empty, stop the timer */
          if (A_base == A_nextseqnum) {
            stoptimer(A);
            A_timer_running_any = false;
            break;
          }
        }
        
        /* Check if all packets have been ACKed, if so, stop the timer */
        bool all_acked = true;
        for (int i = 0; i < WINDOWSIZE; i++) {
          if (A_timer_running[i]) {
            all_acked = false;
            break;
          }
        }
        
        if (all_acked && A_timer_running_any) {
          stoptimer(A);
          A_timer_running_any = false;
        } else if (A_timer_running_any) {
          /* Restart timer for remaining packets */
          stoptimer(A);
          starttimer(A, RTT);
        }
      }
    }
  }
  else {
    if (TRACE > 0)
      printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

/* Handle timer interrupts - Selective Repeat only retransmits the specific timed-out packet */
void A_timerinterrupt(void)
{
  int i;
  
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");
  
  A_timer_running_any = false;
  
  /* Find oldest unacknowledged packet and resend it */
  for (i = 0; i < WINDOWSIZE; i++) {
    int seq = (A_base + i) % SEQSPACE;
    
    if (!A_ack_received[seq] && seq != A_nextseqnum) {
      if (TRACE > 0)
        printf("---A: resending packet %d\n", seq);
      
      tolayer3(A, buffer[seq % WINDOWSIZE]);
      packets_resent++;
      
      /* Only send one packet at a time */
      break;
    }
  }
  
  /* Restart timer if there are still unacknowledged packets */
  if (A_base != A_nextseqnum) {
    starttimer(A, RTT);
    A_timer_running_any = true;
  }
}

/********* Receiver (B) variables and functions ************/

static int B_base;                /* base of the receiving window */
static int B_nextseqnum;          /* sequence number for B's packets */
static bool B_received[SEQSPACE]; /* track which packets have been received */
static struct pkt B_buffer[SEQSPACE]; /* buffer for out-of-order packets */

/* Initialize receiver variables */
void B_init(void)
{
  int i;
  B_base = 0;
  B_nextseqnum = 0;
  
  /* Initialize all packets as not received */
  for (i = 0; i < SEQSPACE; i++) {
    B_received[i] = false;
  }
}

/* Handle incoming packets */
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;
  
  /* if not corrupted and within receiving window */
  if (!IsCorrupted(packet) &&
      (((packet.seqnum >= B_base) && (packet.seqnum < B_base + WINDOWSIZE)) ||
       ((B_base + WINDOWSIZE > SEQSPACE) && (packet.seqnum < (B_base + WINDOWSIZE) % SEQSPACE)))) {
    
    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n", packet.seqnum);
    
    /* Store the packet and mark as received */
    B_buffer[packet.seqnum % SEQSPACE] = packet;
    B_received[packet.seqnum] = true;
    
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
    while (B_received[B_base]) {
      /* Deliver to application layer */
      tolayer5(B, B_buffer[B_base % SEQSPACE].payload);
      packets_received++;
      
      /* Mark as not received for future use */
      B_received[B_base] = false;
      
      /* Advance window */
      B_base = (B_base + 1) % SEQSPACE;
    }
  }
  else {
    if (TRACE > 0) {
      if (IsCorrupted(packet))
        printf("----B: packet %d is corrupted, send ACK!\n", packet.seqnum);
      else
        printf("----B: packet %d is out of window, send ACK!\n", packet.seqnum);
    }
    
    /* Create ACK for the highest in-order packet received so far */
    int lastAck = (B_base - 1 + SEQSPACE) % SEQSPACE;
    sendpkt.acknum = lastAck;
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