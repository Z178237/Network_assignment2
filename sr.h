#ifndef SR_H
#define SR_H

// Window size and sequence number space
#define WINDOWSIZE 5
#define SEQSPACE 8
#define TIMEOUT 25.0

// Message structure from emulator
struct msg {
  char data[20];
};

// Packet structure over the network
struct pkt {
  int seqnum;
  int acknum;
  int checksum;
  char payload[20];
};

// A-side functions
void A_init();
void A_input(struct pkt);
void A_output(struct msg);
void A_timerinterrupt();

// B-side functions
void B_init();
void B_input(struct pkt);

// included for extension to bidirectional communication
#define BIDIRECTIONAL 0
void B_output(struct msg);
void B_timerinterrupt();

#endif
