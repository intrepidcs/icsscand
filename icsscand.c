/*
 * icsscd.c - Userspace daemon for Intrepid SocketCAN support
 *
 * Copyright (c) 2016 Intrepid Control Systems, Inc.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <syslog.h>
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/if.h>
#include <fcntl.h>
#include <pthread.h>
#include <ics/icsneo40API.h>

#define DAEMON_NAME             "icsscand"
#define RUN_AS_USER             "root"
#define NETDEVICE_PATH          "/dev/intrepid_netdevice"
#define SIOCSADDIF              0x3001
#define SIOCSREMOVEIF           0x3002
#define SIOCGSHAREDMEMSIZE      0x3003
#define SIOCSMSGSWRITTEN        0x3004
#define SIOCGMAXIFACES          0x3005
#define MSG_BUFFER_NUM_MSGS     20000

struct interface_t {
        int             kernel_handle;
        unsigned char   *to_kernel;
        char            name[IFNAMSIZ];
};

struct connected_device_t {
        /* handle returned by icsneoOpenNeoDevice */
        void               *handle;

        /* info on interfaces created by the kernel driver */
        struct interface_t *interfaces;

        /* the number of interfaces for this device */
        int                 num_interfaces;

        /* translation from the icsSpyMessage NetworkID to interface handle */
        int                 netid_to_interface[NETID_MAX];

        /* while this is true the rx thread will keep running */
        int                 keep_running;

        pthread_t           rx_thread;
        NeoDevice           device;
};

struct intrepid_pending_tx_info {
        int tx_box_index;
        int count;
};

struct netid_lookup_t {
        int device_index;
        int netid;
};

static int              run_as_daemon           = 0; /* if running in daemon mode */
static int              icsscand_running        = 0; /* if we're running */
static int              netdevice               = 0; /* fd of /dev/intrepid_netdevice */
static int              shared_mem_size         = 0; /* size of shared kernel mem */
static int              max_num_ifaces          = 0; /* max ifaces the kernel supports */
static unsigned char    *shared_mem             = NULL; /* shared kernel mem */
static int exit_code;

static struct connected_device_t *connected     = NULL; /* connected devices */
static struct netid_lookup_t     *netid_lookup  = NULL; /* iface index -> device/netid */
static pthread_mutex_t            devices_mutex       ; /* mutex for adding/removing devices */

#define RX_BOX_SIZE                (shared_mem_size / (max_num_ifaces * 2))
#define TX_BOX_SIZE                (shared_mem_size / 4)
#define GET_RX_BOX(DEVICE_INDEX)   (shared_mem + (RX_BOX_SIZE * DEVICE_INDEX))
#define GET_TX_BOX(INDEX)          (shared_mem + (shared_mem_size / 2) + (INDEX * TX_BOX_SIZE))
#define MAX_NUM_RX_MSGS            (RX_BOX_SIZE / sizeof(icsSpyMessage))
#define MAX_NUM_TX_MSGS            (TX_BOX_SIZE / sizeof(icsSpyMessage))

#define PROBE_INTERVAL_MS           1000UL
#define TX_TIMEOUT_MS               100UL

#define LOG(LVL, MSG, ...)         do{if(run_as_daemon) syslog(LVL, MSG, __VA_ARGS__); \
                                        else fprintf(stderr, MSG, __VA_ARGS__);}while(0)
#define LOG_NOARGS(LVL, MSG)       do{if(run_as_daemon) syslog(LVL, MSG); \
                                        else fprintf(stderr, MSG);}while(0)

static void print_usage(char *prg)
{
        fprintf(stderr, "\nUsage: %s\n\n", prg);
        fprintf(stderr, "         -D         (run as a daemon)\n");
        fprintf(stderr, "         -h         (show this help page)\n");
        fprintf(stderr, "\n");
        exit(EXIT_FAILURE);
}

static void signal_handler(int signum)
{
        switch (signum) {

        case SIGUSR1:
                exit(EXIT_SUCCESS);
                break;
        case SIGALRM:
        case SIGCHLD:
                LOG(LOG_NOTICE, "received signal %i\n", signum);
                exit_code = EXIT_FAILURE;
                icsscand_running = 0;
                break;
        case SIGINT:
        case SIGTERM:
                LOG(LOG_NOTICE, "received signal %i\n", signum);
                exit_code = EXIT_SUCCESS;
                icsscand_running = 0;
                break;
        }
}

static void free_connected_device(struct connected_device_t* device, int join_thread)
{
        if (join_thread)
        {
                device->keep_running = 0;
                pthread_join(device->rx_thread, NULL);
        }
        if (device->handle)
        {
                int num_errors;
                icsneoClosePort(device->handle, &num_errors);
        }
        if (device->interfaces)
        {
                int i;
                for (i = 0 ; i < device->num_interfaces ; ++i)
                {
                        ioctl(netdevice, SIOCSREMOVEIF, device->interfaces[i].kernel_handle);
                        netid_lookup[device->interfaces[i].kernel_handle].device_index = -1;
                }
                free(device->interfaces);
        }
        memset(device, 0, sizeof(*device));
}

static void setup_interface_info(int device_index, int net_id,
        int iface_index, const char* name)
{
        struct connected_device_t *device  = &connected[device_index];
        struct interface_t        *iface   = &device->interfaces[iface_index];

        device->netid_to_interface[net_id] = iface_index;

        netid_lookup[iface->kernel_handle].device_index = device_index;
        netid_lookup[iface->kernel_handle].netid = net_id;

        /* this is apparently how you change a netdevice's name */
        if (name)
        {
                char new_name[IFNAMSIZ];
                struct ifreq ifr;
                int s = socket(PF_INET, SOCK_DGRAM, 0);

                sprintf(new_name, "ics%d%s", device_index, name);

                if (s >= 0)
                {
                        strncpy(ifr.ifr_name,    iface->name,  IFNAMSIZ);
                        strncpy(ifr.ifr_newname, new_name,     IFNAMSIZ);

                        if (ioctl(s, SIOCSIFNAME, &ifr) < 0)
                        {
                                LOG(LOG_NOTICE, "could not rename %s to %s\n",
                                        iface->name, new_name);
                        }
                        else
                                strncpy(iface->name, new_name, IFNAMSIZ);

                        close(s);
                }
        }
}

static void* rx(void* arg)
{
        int num_errors, num_msgs, ret, i;

        struct connected_device_t* dev  = (struct connected_device_t*)arg;
        icsSpyMessage* msgs = (icsSpyMessage*)malloc(
                sizeof(icsSpyMessage) * MSG_BUFFER_NUM_MSGS);
        int *box_count   = (int*)malloc(sizeof(int) * dev->num_interfaces);
        int max_num_msgs = MAX_NUM_RX_MSGS;

        /* we pass the num of written messages as a short, so cap at this amount */
        if (max_num_msgs >= 1 << 16)
                max_num_msgs = (1 << 16) - 1;

        if (!msgs || !box_count)
        {
                LOG(LOG_ERR, "could not allocate a %d message buffer\n",
                        MSG_BUFFER_NUM_MSGS);
                goto exit;
        }

        while (dev->keep_running)
        {
                /* block the thread waiting for more messages.
                 * this is a pthread_cond_timedwait underneath, so our thread
                 * will sleep if there's nothing there */
                int ret = icsneoWaitForRxMessagesWithTimeOut(dev->handle, 100);
                if (ret < 0)
                {
                        LOG(LOG_ERR, "error waiting for messages on device %d\n",
                                dev->device.SerialNumber);
                        goto error;
                }
                else if(ret == 0)
                        continue; /* timeout */

                /* read out the pending messages */
                ret = icsneoGetMessages(dev->handle, msgs, &num_msgs, &num_errors);
                if (ret == 0)
                {
                        LOG(LOG_ERR, "error reading messages on device %d\n",
                                dev->device.SerialNumber);
                        goto error;
                }

                /* reset the counts of all boxes */
                memset(box_count, 0, sizeof(int) * dev->num_interfaces);

                for(i = 0 ; i < num_msgs ; ++i)
                {
                        icsSpyMessage* box;
                        struct interface_t *iface;

                        /* get the message to copy */
                        const icsSpyMessage* msg = &msgs[i];

                        /* lookup which interface this network maps to */
                        int iface_index = dev->netid_to_interface[msg->NetworkID];
                        if (iface_index == -1)
                                continue; /* unknown network */
                        iface = &dev->interfaces[iface_index];

                        /* find the appropriate box for writing to this iface */
                        box = (icsSpyMessage*)iface->to_kernel;

                        /* copy to kernel memory */
                        memcpy(box + box_count[iface_index], msg, sizeof(icsSpyMessage));

                        /* check to see if the box is full -- if so, notify the kernel */
                        if (++box_count[iface_index] == max_num_msgs)
                        {
                                unsigned int ioctl_arg =
                                        (iface->kernel_handle << 16) | max_num_msgs;

                                ret = ioctl(netdevice, SIOCSMSGSWRITTEN, ioctl_arg);
                                if (ret < 0)
                                {
                                        LOG(LOG_ERR, "error transferring to kernel: %s\n",
                                                strerror(ret));
                                        goto error;
                                }
                                box_count[iface_index] = 0;
                        }
                }

                /* notify the kernel of any boxes that didn't fill up */
                for(i = 0 ; i < dev->num_interfaces ; ++i)
                {
                        struct interface_t *iface = &dev->interfaces[i];
                        if(box_count[i] > 0)
                        {
                                unsigned int ioctl_arg =
                                        (iface->kernel_handle << 16) | box_count[i];

                                ret = ioctl(netdevice, SIOCSMSGSWRITTEN, ioctl_arg);
                                if (ret < 0)
                                {
                                        LOG(LOG_ERR, "error transferring to kernel: %s\n",
                                                strerror(ret));
                                        goto error;
                                }
                        }
                }
        }

        goto exit;

error:
        pthread_mutex_lock(&devices_mutex);
        free_connected_device(dev, 0); /* don't cancel our thread quite yet */
        pthread_mutex_unlock(&devices_mutex);

exit:
        if (msgs)
                free(msgs);
        if (box_count)
                free(box_count);
        return 0;
}

static void probe_new_devices()
{
        NeoDevice *detected = (NeoDevice*)malloc(sizeof(NeoDevice) * max_num_ifaces);
        int num_devices = max_num_ifaces, i;

        /* find all the connected devices */
        int ret = icsneoFindNeoDevices(NEODEVICE_ALL & ~NEODEVICE_RADGALAXY, detected, &num_devices);
        if (ret == 0 || num_devices == 0)
        {
                free(detected);
                return;
        }

        pthread_mutex_lock(&devices_mutex);
        for (i = 0 ; i < num_devices ; ++i)
        {
                int j, num_nets = 0, already_connected = 0, device_index;
                struct connected_device_t *device = NULL;

                /* see if we're already connected to this device, and if we aren't,
                 * find a spot for this device in our array */
                for(j = 0 ; j < max_num_ifaces ; ++j)
                {
                        if (connected[j].handle              != NULL                    &&
                            connected[j].device.DeviceType   == detected[i].DeviceType  &&
                            connected[j].device.SerialNumber == detected[i].SerialNumber  )
                        {
                                already_connected = 1;
                                break;
                        }
                        if (device == NULL && connected[j].handle == NULL)
                                device = &connected[device_index = j];
                }
                if (already_connected)
                        continue;

                if (device == NULL)
                        break; /* no more room at the inn */

                /* actually open the device */
                ret = icsneoOpenNeoDevice(&detected[i], &device->handle, NULL, 1, 0);
                if (ret != 1)
                {
                        LOG(LOG_ERR, "Unable to open device with serial %i\n",
                                device->device.SerialNumber);
                        continue;
                }

                device->device = detected[i];

                /* figure out how many interfaces we need to make */
                switch(device->device.DeviceType)
                {
				case NEODEVICE_VCAN41:
				case NEODEVICE_VCAN42:
                case NEODEVICE_VCAN3:
                        num_nets = 2;
                        break;
                case NEODEVICE_FIRE:
                case NEODEVICE_ANY_ION:
                case NEODEVICE_ANY_PLASMA:
                        /* some ions and plasmas actually have 8,
                        *  but we can't tell from the PID */
                        num_nets = 6;
                        break;
                case NEODEVICE_FIRE2:
                        /* todo: software selectable networks */
                        num_nets = 6;
                        break;
                }

                if(num_nets <= 0)
                {
                        LOG(LOG_ERR, "Unknown device with serial %d\n",
                                device->device.SerialNumber);
                        free_connected_device(device, 0);
                        continue;
                }

                device->interfaces = (struct interface_t*)malloc(
                        sizeof(struct interface_t) * num_nets
                );

                for(j = 0 ; j < num_nets ; ++j)
                {
                        struct interface_t *iface = &device->interfaces[device->num_interfaces++];
                        ret = ioctl(netdevice, SIOCSADDIF, iface->name);
                        if (ret >= 0)
                        {
                                iface->kernel_handle    = ret;
                                iface->to_kernel        = GET_RX_BOX(ret);
                        }
                }

                /* make sure we got all the interfaces we wanted */
                if (device->num_interfaces != num_nets)
                {
                        LOG(LOG_ERR, "Could not create all interfaces for %d\n",
                                device->device.SerialNumber);
                        free_connected_device(device, 0);
                        continue;
                }

                /* build a mapping from network id -> interface id */
                for(j = 0 ; j < NETID_MAX ; ++j)
                        device->netid_to_interface[j] = -1;
                switch(device->device.DeviceType)
                {
				case NEODEVICE_VCAN41:
				case NEODEVICE_VCAN42:
                case NEODEVICE_VCAN3:
                        setup_interface_info(device_index, NETID_HSCAN,   0, "can0");
                        setup_interface_info(device_index, NETID_MSCAN,   1, "can1");
                        break;
                case NEODEVICE_FIRE:
                case NEODEVICE_ANY_PLASMA:
                case NEODEVICE_ANY_ION:
                        setup_interface_info(device_index, NETID_HSCAN,   0, "can0");
                        setup_interface_info(device_index, NETID_MSCAN,   1, "can1");
                        setup_interface_info(device_index, NETID_HSCAN2,  2, "can2");
                        setup_interface_info(device_index, NETID_HSCAN3,  3, "can3");
                        setup_interface_info(device_index, NETID_SWCAN,   4, "swcan0");
                        setup_interface_info(device_index, NETID_LSFTCAN, 5, "lsftcan0");
                        /* TODO: IONs w/ 8 CAN */
                        break;
                case NEODEVICE_FIRE2:
                        setup_interface_info(device_index, NETID_HSCAN,   0, "can0");
                        setup_interface_info(device_index, NETID_MSCAN,   1, "can1");
                        setup_interface_info(device_index, NETID_HSCAN2,  2, "can2");
                        setup_interface_info(device_index, NETID_HSCAN3,  3, "can3");
                        setup_interface_info(device_index, NETID_HSCAN4,  4, "can4");
                        setup_interface_info(device_index, NETID_HSCAN5,  5, "can5");
                        /* TODO: software selectable networks */
                        break;
                }

                device->keep_running = 1;
                ret = pthread_create(&device->rx_thread, NULL, rx, device);
                if (ret)
                {
                        LOG(LOG_ERR, "Error creating thread for %d\n",
                                device->device.SerialNumber);
                        free_connected_device(device, 0);
                        continue;
                }
        }
        pthread_mutex_unlock(&devices_mutex);

        free(detected);
}

static unsigned long get_ms_tick_count()
{
        struct timeval tv;

        gettimeofday(&tv, NULL);

        return (tv.tv_sec * 1000UL) + (tv.tv_usec / 1000UL);
}

/* kernel wants us to transmit some messages */
static void tx(int index, int count)
{
        struct netid_lookup_t     *lookup;
        struct connected_device_t *dev;
        icsSpyMessage             *msg = (icsSpyMessage*)GET_TX_BOX(index);
        int i, ret;

        for(i = 0 ; i < count ; ++i, ++msg)
        {
                /* NetworkID is actually the device index. When we went online we stored
                 * the real NetworkID for this device index in netid. Do the reverse
                 * lookup and set the correct NetworkID */
                if (msg->NetworkID < 0 || msg->NetworkID >= max_num_ifaces)
                        continue;
                lookup = &netid_lookup[msg->NetworkID];

                if (lookup->device_index < 0 || lookup->device_index > max_num_ifaces)
                        continue;

                msg->NetworkID  = lookup->netid;
                dev             = &connected[lookup->device_index];

                ret = icsneoTxMessages(dev->handle, msg, lookup->netid, 1);

                if (ret == 0)
                {
                        LOG(LOG_ERR, "error transmitting on device %d\n",
                                dev->device.SerialNumber);

                        pthread_mutex_lock(&devices_mutex);
                        free_connected_device(dev, 1);
                        pthread_mutex_unlock(&devices_mutex);

                        break;
                }
        }
}

static void* probe_device_thread(void* arg)
{
        while (icsscand_running)
        {
                usleep(PROBE_INTERVAL_MS * 1000);

                if (!icsscand_running)
                        return NULL;

                probe_new_devices(netdevice);
        }
}

int main(int argc, char **argv)
{
        int opt, ret, i, opened_devices = 0;
        unsigned long last_probe = 0;
        fd_set fds;
        struct timeval timeout;
        struct timespec clock;
        pthread_t probe_thread;

        /* process command line switches */
        while ((opt = getopt(argc, argv, "hD")) != -1)
        {
                switch(opt)
                {
                case 'D':
                        run_as_daemon = 1;
                        break;
                case 'h':
                case '?':
                default:
                        print_usage(argv[0]);
                        break;
                }
        }

        /* setup logging, signals, and daemonize if requested */
        openlog(DAEMON_NAME, LOG_PID, LOG_LOCAL5);
        if (run_as_daemon)
        {
                if (daemon(0, 0))
                {
                        LOG_NOARGS(LOG_ERR, "failed to daemonize");
                        exit(EXIT_FAILURE);
                }
        }
        else
        {
                signal(SIGINT, signal_handler);
                signal(SIGTERM, signal_handler);
        }

        /* we're now "running" -- incoming signals tell us to stop */
        icsscand_running = 1;

        /* this is the lock for adding/removing devices from the connected array */
        pthread_mutex_init(&devices_mutex, NULL);

        icsneoInitializeAPI();

        /* open /dev/intrepid_netdevice -- this has ioctls for adding can interfaces */
        netdevice = open(NETDEVICE_PATH, O_RDWR | O_NONBLOCK);
        if (netdevice < 0)
        {
                LOG(LOG_ERR, "failed to open %s: %s\n", NETDEVICE_PATH, strerror(errno));
                exit(EXIT_FAILURE);
        }

        /* read out some constants from the driver (these are #defines that can change) */
        max_num_ifaces = ioctl(netdevice, SIOCGMAXIFACES);
        if (max_num_ifaces <= 0)
        {
                LOG_NOARGS(LOG_ERR, "could not get maximum number of interfaces\n");
                exit(EXIT_FAILURE);
        }
        shared_mem_size = ioctl(netdevice, SIOCGSHAREDMEMSIZE);
        if (shared_mem_size <= 0)
        {
                LOG_NOARGS(LOG_ERR, "could not read shared memory size\n");
                exit(EXIT_FAILURE);
        }

        /* the shared memory is what we use to transfer messages between the kernel and
         * user mode. the only blocking kernel switch is the ioctl once we're done copying
         * (and the inital page faults) */
        shared_mem = mmap(NULL, shared_mem_size, PROT_READ | PROT_WRITE, MAP_SHARED,
                netdevice, 0);
        if (shared_mem == MAP_FAILED)
        {
                LOG_NOARGS(LOG_ERR, "could not create shared memory mapping\n");
                exit(EXIT_FAILURE);
        }

        /* allocate the global structure of the devices we're conntected to */
        connected = (struct connected_device_t*)
                malloc(sizeof(struct connected_device_t) * max_num_ifaces);
        memset(connected, 0, sizeof(struct connected_device_t) * max_num_ifaces);

        /* the NetworkID of transmit requests is the interface index, we need to
         * convert it to the device/netid pair to transmit on -- allocate this map */
        netid_lookup = (struct netid_lookup_t*)
                malloc(sizeof(struct netid_lookup_t) * max_num_ifaces);
        for(i = 0 ; i < max_num_ifaces ; ++i)
                netid_lookup[i].device_index = -1;

        /* probe once for new devices, then start a thread to do it in the background
         * periodically */
        probe_new_devices(netdevice);
        pthread_create(&probe_thread, NULL, probe_device_thread, NULL);

        /* main loop. icsscand_running can be set to 0 from SIGINT, SIGTERM, etc. */
        while (icsscand_running)
        {
                /* wait for some new transmit messages */
                timeout.tv_sec  = 0;
                timeout.tv_usec = TX_TIMEOUT_MS * 1000;

                FD_ZERO(&fds);
                FD_SET(netdevice, &fds);

                ret = select(netdevice + 1, &fds, NULL, NULL, &timeout);

                if (ret == -1)
                {
                        LOG(LOG_ERR, "error waiting for tx messages: %s\n", strerror(errno));
                        icsscand_running = 0;
                        break;
                }
                else if(ret)
                {
                        /* kernel says there're some new transmit messages waiting to go
                         * out. call read() to find out which box they're in and how many */
                        struct intrepid_pending_tx_info info;
                        ssize_t r;

                        r = read(netdevice, &info, sizeof(info));
                        if (r == -1)
                        {
                                LOG(LOG_ERR, "error reading tx messages: %s\n",
                                        strerror(errno));
                                icsscand_running = 0;
                                break;
                        }
                        else if(r != sizeof(info))
                        {
                                LOG(LOG_ERR,
                                        "unexpected number of bytes read, expected %d got %d\n",
                                        (int)sizeof(info), (int)r);
                                icsscand_running = 0;
                                break;
                        }
                        else
                        {
                                /* send the messages! */
                                tx(info.tx_box_index, info.count);
                        }
                }
                else
                {
                        /* timeout */
                }
        }

        /* cleanup */

        pthread_join(probe_thread, NULL);

        pthread_mutex_lock(&devices_mutex);
        for(i = 0 ; i < max_num_ifaces ; ++i)
                free_connected_device(&connected[i], 1);
        pthread_mutex_unlock(&devices_mutex);

        free(connected);
        free(netid_lookup);
        munmap(shared_mem, shared_mem_size);
        close(netdevice);

        pthread_exit(NULL);
        return exit_code;
}
