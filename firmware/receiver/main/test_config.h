#ifndef SANITY_TEST_CONFIG_H
#define SANITY_TEST_CONFIG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define SZ_TEST_CONFIG_SIZE 16
int sz_test_config(const uint8_t *packet, size_t length, uint8_t device, uint32_t owner_session,
                   bool owner_valid, bool enabled, bool current, uint8_t reply[SZ_TEST_CONFIG_SIZE]);
#endif
