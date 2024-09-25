#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <sys/uio.h>
#include <sys/socket.h>
#include <linux/if_link.h>
#include <linux/if_arp.h>
#include <linux/netlink.h>
#include <linux/can/netlink.h>
#include <linux/rtnetlink.h>

#include "netlink.h"

bool read_netlink_msgs(int s, msg_callback callback)
{
    struct nlmsghdr buf[8192/sizeof(struct nlmsghdr)];

    struct nlmsghdr *nh;
    int len = recv(s, buf, sizeof(buf), 0);

    for (nh = (struct nlmsghdr *) buf; NLMSG_OK (nh, len);
            nh = NLMSG_NEXT (nh, len)) {
        /* The end of multipart message */
        if (nh->nlmsg_type == NLMSG_DONE) {
            printf("Done\n");
            return true;
        }

        if (nh->nlmsg_type == NLMSG_ERROR) {
            printf("Error\n");
        } else if (nh->nlmsg_type == RTM_NEWLINK) {
            struct ifinfomsg *iface = NLMSG_DATA(nh);
            int attrlen = nh->nlmsg_len - NLMSG_LENGTH(sizeof(*iface));
            if (iface->ifi_type == ARPHRD_CAN) {
                for (struct rtattr *rta = IFLA_RTA(iface); RTA_OK(rta, attrlen); rta = RTA_NEXT(rta, attrlen)) {
                    switch(rta->rta_type) {
                        case IFLA_LINKINFO:
                            {
                                int attr2len = RTA_PAYLOAD(rta);
                                char *kind = NULL;
                                void *data = NULL;
                                int data_len;
                                for (struct rtattr *rta2 = RTA_DATA(rta); RTA_OK(rta2, attr2len);
                                        rta2 = RTA_NEXT(rta2, attr2len)) {
                                    switch (rta2->rta_type) {
                                        case IFLA_INFO_KIND:
                                            kind = RTA_DATA(rta2);
                                            break;
                                        case IFLA_INFO_DATA:
                                            data = RTA_DATA(rta2);
                                            data_len = RTA_PAYLOAD(rta2);
                                            break;
                                    }
                                }
                                if (kind && strcmp(kind, "can") == 0 && data) {
                                    attr2len = data_len;
                                    for (struct rtattr *rta2 = data; RTA_OK(rta2, attr2len);
                                            rta2 = RTA_NEXT(rta2, attr2len)) {
                                        if (callback) {
                                            callback(iface->ifi_index, rta2->rta_type, RTA_DATA(rta2));
                                        }
                                    }
                                }
                            }
                            break;
                    }
                }
            }
        }
    }
}

int open_netlink_socket()
{
    int s = socket(AF_NETLINK, SOCK_RAW|SOCK_CLOEXEC, NETLINK_ROUTE);

    if (s < 0) {
        return -1;
    }

    struct sockaddr_nl addr = {
        .nl_family = AF_NETLINK,
        .nl_groups = RTMGRP_LINK,
    };

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        return -1;
    }


    int group = RTMGRP_LINK;
	if (setsockopt(s, SOL_NETLINK, NETLINK_ADD_MEMBERSHIP, &group, sizeof(group))) {
        return -1;
    }

    return s;
}
