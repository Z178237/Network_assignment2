#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "emulator.h"

#define WINDOW_SIZE 8
#define SEQ_SPACE 16
#define RTT 16.0

struct pkt send_buffer[SEQ_SPACE];
bool acked[SEQ_SPACE];
float timeout_time[SEQ_SPACE];

int base = 0;
int next_seqnum = 0;

struct pkt recv_buffer[SEQ_SPACE];
bool received[SEQ_SPACE];
int expected_seqnum = 0;

int compute_checksum(struct pkt packet) {
    int checksum = 0;
    checksum += packet.seqnum;
    checksum += packet.acknum;
    for (int i = 0; i < 20; ++i) {
        checksum += packet.payload[i];
    }
    return checksum;
}

bool is_corrupted(struct pkt packet) {
    return compute_checksum(packet) != packet.checksum;
}

bool in_window(int seq) {
    return ((seq - base + SEQ_SPACE) % SEQ_SPACE) < WINDOW_SIZE;
}

struct pkt make_ack(int acknum) {
    struct pkt ack;
    ack.seqnum = 0;
    ack.acknum = acknum;
    memset(ack.payload, 0, 20);
    ack.checksum = compute_checksum(ack);
    return ack;
}

void A_output(struct msg message) {
    if ((next_seqnum - base + SEQ_SPACE) % SEQ_SPACE < WINDOW_SIZE) {
        struct pkt p;
        p.seqnum = next_seqnum;
        p.acknum = -1;
        memcpy(p.payload, message.data, 20);
        p.checksum = compute_checksum(p);
        send_buffer[next_seqnum] = p;
        acked[next_seqnum] = false;
        timeout_time[next_seqnum] = get_sim_time() + RTT;

        printf("Sending packet %d to layer 3\n", p.seqnum);
        tolayer3(A, p);

        if (base == next_seqnum) {
            starttimer(A, RTT);
        }

        next_seqnum = (next_seqnum + 1) % SEQ_SPACE;
    } else {
        printf("Window full. Dropping message.\n");
    }
}

void A_input(struct pkt packet) {
    if (!is_corrupted(packet) && in_window(packet.acknum)) {
        acked[packet.acknum] = true;

        while (acked[base]) {
            acked[base] = false;
            base = (base + 1) % SEQ_SPACE;
        }

        stoptimer(A);
        if (base != next_seqnum) {
            starttimer(A, RTT);
        }
    }
}

void A_timerinterrupt(void) {
    for (int i = 0; i < SEQ_SPACE; ++i) {
        if (((i - base + SEQ_SPACE) % SEQ_SPACE) < WINDOW_SIZE && !acked[i]) {
            printf("Retransmitting packet %d due to timeout\n", i);
            tolayer3(A, send_buffer[i]);
            timeout_time[i] = get_sim_time() + RTT;
        }
    }
    starttimer(A, RTT);
}

void A_init(void) {
    base = 0;
    next_seqnum = 0;
    for (int i = 0; i < SEQ_SPACE; ++i) {
        acked[i] = false;
    }
}

void B_input(struct pkt packet) {
    if (!is_corrupted(packet)) {
        if (!received[packet.seqnum]) {
            recv_buffer[packet.seqnum] = packet;
            received[packet.seqnum] = true;
        }

        while (received[expected_seqnum]) {
            tolayer5(B, recv_buffer[expected_seqnum].payload);
            received[expected_seqnum] = false;
            expected_seqnum = (expected_seqnum + 1) % SEQ_SPACE;
        }

        struct pkt ackpkt = make_ack(packet.seqnum);
        tolayer3(B, ackpkt);
    } else {
        printf("Received corrupted packet at B\n");
    }
}

void B_init(void) {
    expected_seqnum = 0;
    for (int i = 0; i < SEQ_SPACE; ++i) {
        received[i] = false;
    }
}
