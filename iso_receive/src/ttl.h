#ifndef TTL_H
#define TTL_H

#include "inttypes.h"

#define TTL_OK (0)
#define TTL_ERR (-1)

enum TTL_LIGHT_STATES
{
    TTL_LIGHT_REVERSE = 0,
    TTL_LIGHT_BREAK,
    TTL_LIGHT_TURN,
    TTL_LIGHT_RUNNING
};

typedef union
{
    uint16_t state;
    struct __attribute__((__packed__))
    {
        uint8_t magic;
        uint8_t state;
    } parts;
} ttl_light_state_t;

typedef union
{
    uint32_t entire;
    struct __attribute__((__packed__))
    {
        ttl_light_state_t left;
        ttl_light_state_t right;
    } parts;
} ttl_state_t;

typedef int (*ttl_upd_state_cb_t)(ttl_state_t);

#endif