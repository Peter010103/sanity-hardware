#include <stddef.h>
#include "control.h"
/* Guard ctypes fixtures before they pass an undersized mutable C structure. */
size_t sz_test_command_size(void) {return sizeof(struct sz_command);}
size_t sz_test_state_size(void) {return sizeof(struct sz_state);}
