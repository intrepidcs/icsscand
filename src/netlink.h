#ifdef __cplusplus
extern "C" {
#endif

typedef void (*msg_callback)(int /* ifindex */, int /* rta_type */, void * /* data */);

bool read_netlink_msgs(int s, msg_callback callback);

int open_netlink_socket();

#ifdef __cplusplus
}
#endif
