#include "test_config.h"
#include <string.h>
int sz_test_config(const uint8_t *p, size_t n, uint8_t device, uint32_t session,
                   bool valid, bool enabled, bool current, uint8_t reply[SZ_TEST_CONFIG_SIZE]) {
    if (n!=SZ_TEST_CONFIG_SIZE || memcmp(p,"SZTM",4) || p[4]!=1 || p[5]!=device || p[6]>1 || p[7]) return -1;
    uint32_t requested=((uint32_t)p[8]<<24)|((uint32_t)p[9]<<16)|((uint32_t)p[10]<<8)|p[11];
    bool accepted=valid && session && requested==session && !enabled;
    memcpy(reply,p,SZ_TEST_CONFIG_SIZE);memcpy(reply,"SZTA",4);
    reply[6]=accepted?p[6]:current;reply[7]=accepted?0:1;
    return accepted?1:0;
}
