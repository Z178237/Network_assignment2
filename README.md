# Network_assignment2
first test: Normal Transmit Test
Verification result: through 
Protocol operates correctly under ideal conditions.
# Selective Repeat (SR) Protocol Implementation

## Overview

This project implements the Selective Repeat (SR) reliable transport protocol in C. It is based on the `emulator.c/h` and `sr.c/h` framework adapted from J.F. Kurose. The sender (A) and receiver (B) maintain independent windows and support out-of-order delivery, retransmission, and selective acknowledgments.
## Files
- `sr.c`: Main source file implementing the SR logic for sender and receiver.
- `sr.h`: Header file declaring the required SR interface.
- `emulator.c/h`: Provided simulation environment for testing.
- | Test | Description                           | Result | Key Observations |
|------|-----------------------------------------|--------|------------------|
| T1   | Normal transmission                     | Pass | Window slides smoothly; ACKs trigger correctly |
| T2   | Data packet loss                        | Pass | Only base packet is retransmitted |
| T3   | ACK loss                                | Pass | Sender handles missing ACK and retransmits |
| T4   | Multiple data losses                    | Pass | Each base timeout leads to one packet resend |
| T5   | Out-of-order delivery                   | Pass | Receiver buffers and reorders correctly |
| T6   | Window overflow                         | Pass | New messages blocked when window is full |
| T7   | Duplicate ACKs                          | Pass | Handled gracefully; no duplicate sliding |
| T8   | All ACKs lost                           | Pass | System retries base packet persistently |
| T9   | Mixed out-of-order + ACK loss           | Pass | Demonstrates robustness under complex faults |
| T10  | Long run + seqnum wrap-around           | Pass | Window logic and modulo ops are stable |


