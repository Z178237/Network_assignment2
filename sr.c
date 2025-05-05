#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "sr.h"

struct pkt sndpkt[SEQSPACE];
struct msg msg_buffer[1000];

int base = 0;
int nextseqnum = 0;
int nbuffered = 0;
int msg_idx = 0;
int next_msg_to_send = 0;

bool acked[SEQSPACE];

int B_base = 0;
bool received[SEQSPACE];
struct pkt recv_buffer[SEQSPACE];

int compute_checksum(struct pkt packet) {
    int checksum = 0;
    int i;
    checksum += packet.seqnum;
    checksum += packet.acknum;
    for (i = 0; i < 20; i++) {
        checksum += packet.payload[i];
    }
    return checksum;
}

void A_output(struct msg message) {
    // If buffer is full, drop the message
    if (nbuffered >= 1000) {
        window_full++;
        return;
    }

    // Buffer the message
    msg_buffer[msg_idx] = message;
    msg_idx++;
    nbuffered++;

    // Send packets while window has space and there are unsent messages
    int end = (base + WINDOWSIZE) % SEQSPACE;
    if (base + WINDOWSIZE <= SEQSPACE - 1) {
        while (nextseqnum < base + WINDOWSIZE && next_msg_to_send < msg_idx) {
            struct pkt packet;
            packet.seqnum = nextseqnum;
            packet.acknum = 0;
            strncpy(packet.payload, msg_buffer[next_msg_to_send].data, 20);
            packet.checksum = compute_checksum(packet);
            sndpkt[nextseqnum] = packet;
            acked[nextseqnum] = false;
            tolayer3(A, packet);
            printf("A: sending packet %d\n", nextseqnum);
            if (base == nextseqnum) {
                starttimer(A, TIMEOUT);
            }
            nextseqnum = (nextseqnum + 1) % SEQSPACE;
            next_msg_to_send++;
        }
    } else {
        while ((nextseqnum < end || nextseqnum >= base) && next_msg_to_send < msg_idx) {
            struct pkt packet;
            packet.seqnum = nextseqnum;
            packet.acknum = 0;
            strncpy(packet.payload, msg_buffer[next_msg_to_send].data, 20);
            packet.checksum = compute_checksum(packet);
            sndpkt[nextseqnum] = packet;
            acked[nextseqnum] = false;
            tolayer3(A, packet);
            printf("A: sending packet %d\n", nextseqnum);
            if (base == nextseqnum) {
                starttimer(A, TIMEOUT);
            }
            nextseqnum = (nextseqnum + 1) % SEQSPACE;
            next_msg_to_send++;
        }
    }
}

void A_input(struct pkt packet) {
    int checksum = compute_checksum(packet);
    if (checksum != packet.checksum) {
        return;
    }
    total_ACKs_received++;
    printf("A: received ACK %d\n", packet.acknum);

    int acknum = packet.acknum;
    bool inWindow;
    if (base + WINDOWSIZE <= SEQSPACE - 1) {
        inWindow = (acknum >= base && acknum < base + WINDOWSIZE);
    } else {
        int end = (base + WINDOWSIZE) % SEQSPACE;
        inWindow = (acknum >= base || acknum < end);
    }
    if (!inWindow) {
        return;
    }

    if (!acked[acknum]) {
        new_ACKs++;
        acked[acknum] = true;
    }

    int old_base = base;
    while (acked[base]) {
        acked[base] = false;
        base = (base + 1) % SEQSPACE;
        nbuffered--;
    }
    bool base_moved = (base != old_base);
    if (base_moved) {
        stoptimer(A);
    }

    // Send new packets if window has opened
    int end2 = (base + WINDOWSIZE) % SEQSPACE;
    bool timer_started_during_send = false;
    if (base + WINDOWSIZE <= SEQSPACE - 1) {
        while (nextseqnum < base + WINDOWSIZE && next_msg_to_send < msg_idx) {
            struct pkt packet_out;
            packet_out.seqnum = nextseqnum;
            packet_out.acknum = 0;
            strncpy(packet_out.payload, msg_buffer[next_msg_to_send].data, 20);
            packet_out.checksum = compute_checksum(packet_out);
            sndpkt[nextseqnum] = packet_out;
            acked[nextseqnum] = false;
            tolayer3(A, packet_out);
            printf("A: sending packet %d\n", nextseqnum);
            if (base == nextseqnum) {
                starttimer(A, TIMEOUT);
                timer_started_during_send = true;
            }
            nextseqnum = (nextseqnum + 1) % SEQSPACE;
            next_msg_to_send++;
        }
    } else {
        while ((nextseqnum < end2 || nextseqnum >= base) && next_msg_to_send < msg_idx) {
            struct pkt packet_out;
            packet_out.seqnum = nextseqnum;
            packet_out.acknum = 0;
            strncpy(packet_out.payload, msg_buffer[next_msg_to_send].data, 20);
            packet_out.checksum = compute_checksum(packet_out);
            sndpkt[nextseqnum] = packet_out;
            acked[nextseqnum] = false;
            tolayer3(A, packet_out);
            printf("A: sending packet %d\n", nextseqnum);
            if (base == nextseqnum) {
                starttimer(A, TIMEOUT);
                timer_started_during_send = true;
            }
            nextseqnum = (nextseqnum + 1) % SEQSPACE;
            next_msg_to_send++;
        }
    }

    if (nbuffered == 0) {
        // no outstanding packets
        return;
    }
    if (base_moved && !timer_started_during_send) {
        starttimer(A, TIMEOUT);
    }
}

void A_timerinterrupt() {
    printf("A: timeout, resending packets from %d\n", base);
    int i;
    for (i = 0; i < WINDOWSIZE; i++) {
        int idx = (base + i) % SEQSPACE;
        if (idx == nextseqnum) {
            break;
        }
        if (!acked[idx]) {
            tolayer3(A, sndpkt[idx]);
            packets_resent++;
        }
    }
    starttimer(A, TIMEOUT);
}

void A_init() {
    int i;
    base = 0;
    nextseqnum = 0;
    nbuffered = 0;
    msg_idx = 0;
    next_msg_to_send = 0;
    for (i = 0; i < SEQSPACE; i++) {
        acked[i] = false;
    }
}

void B_input(struct pkt pkt) {
    int checksum = compute_checksum(pkt);
    if (checksum != pkt.checksum) {
        printf("----B: checksum error\n");
        return;
    }
    int seq = pkt.seqnum;
    bool inWindow;
    if (B_base + WINDOWSIZE <= SEQSPACE - 1) {
        inWindow = (seq >= B_base && seq < B_base + WINDOWSIZE);
    } else {
        int end = (B_base + WINDOWSIZE) % SEQSPACE;
        inWindow = (seq >= B_base || seq < end);
    }
    struct pkt ack;
    if (!inWindow) {
        if (seq >= 0 && seq < SEQSPACE) {
            memset(ack.payload, 0, sizeof(ack.payload));
            ack.seqnum = 0;
            ack.acknum = seq;
            ack.checksum = compute_checksum(ack);
            tolayer3(B, ack);
            printf("----B: packet %d is duplicate or out of window, send ACK again!\n", seq);
        }
        return;
    }
    if (!received[seq]) {
        received[seq] = true;
        recv_buffer[seq] = pkt;
        printf("B: packet %d is correctly received, send ACK!\n", seq);
        memset(ack.payload, 0, sizeof(ack.payload));
        ack.seqnum = 0;
        ack.acknum = seq;
        ack.checksum = compute_checksum(ack);
        tolayer3(B, ack);
        while (received[B_base]) {
            tolayer5(B, recv_buffer[B_base].payload);
            received[B_base] = false;
            B_base = (B_base + 1) % SEQSPACE;
            packets_received++;
        }
    } else {
        memset(ack.payload, 0, sizeof(ack.payload));
        ack.seqnum = 0;
        ack.acknum = seq;
        ack.checksum = compute_checksum(ack);
        tolayer3(B, ack);
        printf("----B: duplicate packet %d received again, send ACK!\n", seq);
    }
}

void B_init() {
    int i;
    B_base = 0;
    for (i = 0; i < SEQSPACE; i++) {
        received[i] = false;
    }
}

// B_output and B_timerinterrupt are not used in unidirectional (A->B) communication
void B_output(struct msg message) {
    // Not used in unidirectional protocols
}

void B_timerinterrupt(void) {
    // Not used in unidirectional protocols
}
