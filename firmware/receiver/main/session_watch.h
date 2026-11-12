#ifndef SANITY_SESSION_WATCH_H
#define SANITY_SESSION_WATCH_H
#include "zenoh-pico.h"
#if Z_FEATURE_CONNECTIVITY == 1
struct sz_session_watch { bool connected; };
int sz_session_watch_start(const z_loaned_session_t *session,struct sz_session_watch *watch,z_owned_transport_events_listener_t *listener);
bool sz_session_watch_connected(const struct sz_session_watch *watch);
#endif
#endif
