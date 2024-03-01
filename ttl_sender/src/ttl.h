#ifndef TTL_H
#define TTL_H

#include "inttypes.h"

#define TTL_OK (0)
#define TTL_ERR (-1)

#define TTL_POLLING_INTERVAL_MS (100)

/**
 * @brief TTL light states
 */
typedef union {
  uint32_t entire;
  struct __attribute__((__packed__)) {
    uint8_t magic;
    uint8_t reserved;
    struct __attribute__((__packed__)) {
      uint16_t reverse : 1; // ???
      uint16_t breaks : 1;  // rot/54
      uint16_t lturn : 1;   // yellow/L
      uint16_t rturn : 1;   // green/R
      uint16_t ldrive : 1;  // black/58L
      uint16_t rdrive : 1;  // gray/58R
      uint16_t fog : 1;     // blue/54G
      uint16_t reserved : 9;
    } bits;
  } parts;
} ttl_state_t;

typedef int (*ttl_upd_state_cb_t)(ttl_state_t);

#define PRINT_TTL_STATE(state)                                                 \
  LOG_INF("TTLight State: {magic: 0x%x, breaks: %d, lturn: %d, rturn: %d, "    \
          "reverse: %d, ldrive: %d, rdrive: %d}",                              \
          state.parts.magic, state.parts.bits.breaks, state.parts.bits.lturn,  \
          state.parts.bits.rturn, state.parts.bits.reverse,                    \
          state.parts.bits.ldrive, state.parts.bits.rdrive);
#endif