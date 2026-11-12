#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>
typedef struct blackboxFieldDefinition_s {
    const char *name;
    // If the field name has a number to be included in square brackets [1] afterwards, set it here, or -1 for no brackets:
    int16_t fieldNameIndex; // Extended Sanity trace indices exceed 127.

    // Each member of this array will be the value to print for this field for the given header index
    uint8_t arr[1];
} blackboxFieldDefinition_t;
typedef struct blackboxSimpleFieldDefinition_s {
    const char *name;
    int16_t fieldNameIndex; // Extended Sanity trace indices exceed 127.

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
} blackboxSimpleFieldDefinition_t;
typedef struct blackboxConditionalFieldDefinition_s {
    const char *name;
    int16_t fieldNameIndex; // Extended Sanity trace indices exceed 127.

    uint8_t isSigned;
    uint8_t predict;
    uint8_t encode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxConditionalFieldDefinition_t;
typedef struct blackboxDeltaFieldDefinition_s {
    const char *name;
    int16_t fieldNameIndex; // Extended Sanity trace indices exceed 127.

    uint8_t isSigned;
    uint8_t Ipredict;
    uint8_t Iencode;
    uint8_t Ppredict;
    uint8_t Pencode;
    uint8_t condition; // Decide whether this field should appear in the log
} blackboxDeltaFieldDefinition_t;
_Static_assert(offsetof(blackboxSimpleFieldDefinition_t,name)==offsetof(blackboxFieldDefinition_t,name),"common name");
_Static_assert(offsetof(blackboxSimpleFieldDefinition_t,fieldNameIndex)==offsetof(blackboxFieldDefinition_t,fieldNameIndex),"common fieldNameIndex");
_Static_assert(offsetof(blackboxSimpleFieldDefinition_t,isSigned)==offsetof(blackboxFieldDefinition_t,arr),"common metadata offset");
_Static_assert(offsetof(blackboxConditionalFieldDefinition_t,name)==offsetof(blackboxFieldDefinition_t,name),"common name");
_Static_assert(offsetof(blackboxConditionalFieldDefinition_t,fieldNameIndex)==offsetof(blackboxFieldDefinition_t,fieldNameIndex),"common fieldNameIndex");
_Static_assert(offsetof(blackboxConditionalFieldDefinition_t,isSigned)==offsetof(blackboxFieldDefinition_t,arr),"common metadata offset");
_Static_assert(offsetof(blackboxDeltaFieldDefinition_t,name)==offsetof(blackboxFieldDefinition_t,name),"common name");
_Static_assert(offsetof(blackboxDeltaFieldDefinition_t,fieldNameIndex)==offsetof(blackboxFieldDefinition_t,fieldNameIndex),"common fieldNameIndex");
_Static_assert(offsetof(blackboxDeltaFieldDefinition_t,isSigned)==offsetof(blackboxFieldDefinition_t,arr),"common metadata offset");
int main(void) { const blackboxDeltaFieldDefinition_t f[2]={{"test",-1,1,2,3,4,5,6},{"test",189,6,5,4,3,2,1}}; const char *base=(const char *)f; size_t stride=(const char *)(f+1)-base; int16_t index; memcpy(&index,base+stride+offsetof(blackboxFieldDefinition_t,fieldNameIndex),sizeof(index)); assert(index==189); const unsigned char *metadata=(const unsigned char *)(base+stride+offsetof(blackboxFieldDefinition_t,arr)); for(unsigned i=0;i<6;i++)assert(metadata[i]==6-i); return 0;}
