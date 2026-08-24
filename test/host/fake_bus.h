#pragma once
#include <driver/twai.h>
#include <stdint.h>
#include <vector>

namespace fake {

using ReplyFn = void (*)(const twai_message_t& sent);

// Every frame the driver transmitted, in order.
extern std::vector<twai_message_t> tx_log;
// Called on each transmit so a test can answer like a motor would.
extern ReplyFn auto_reply;

void reset();
// Make `m` readable delay_us of virtual time from now.
void schedule(const twai_message_t& m, int64_t delay_us);
// Put `m` straight into the RX queue, as if it had arrived before we looked.
void queueNow(const twai_message_t& m);
int64_t nowUs();
void    advance(int64_t us);
void    setState(int s);

} // namespace fake
