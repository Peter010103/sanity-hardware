#include "session_watch.h"
#if Z_FEATURE_CONNECTIVITY == 1
static void connection_event(z_loaned_transport_event_t *event,void *context) {
    struct sz_session_watch *watch=context;
    __atomic_store_n(&watch->connected,z_transport_event_kind(event)==Z_SAMPLE_KIND_PUT,__ATOMIC_RELEASE);
}
int sz_session_watch_start(const z_loaned_session_t *session,struct sz_session_watch *watch,z_owned_transport_events_listener_t *listener) {
    __atomic_store_n(&watch->connected,false,__ATOMIC_RELEASE);
    z_owned_closure_transport_event_t closure;
    z_closure_transport_event(&closure,connection_event,NULL,watch);
    z_transport_events_listener_options_t options;z_transport_events_listener_options_default(&options);
    options.history=true;
    return z_declare_transport_events_listener(session,listener,z_closure_transport_event_move(&closure),&options);
}
bool sz_session_watch_connected(const struct sz_session_watch *watch) {
    return __atomic_load_n(&watch->connected,__ATOMIC_ACQUIRE);
}
#endif
