#include <iostream>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <map>

#include <sysexits.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <signal.h>
#include <linux/if.h>
#include <linux/can/netlink.h>

#include <icsneo/icsneocpp.h>
#include <icsneo/communication/message/neomessage.h>
#include <icsneo/communication/message/message.h>
#include <icsneo/communication/network.h>
#include <icsneo/communication/message/callback/canmessagecallback.h>
#include <generated/buildinfo.h>

#include "netlink.h"

#define LOG(LVL, MSG)			do{if(runningAsDaemon) syslog(LVL, MSG); \
					else fprintf(stderr, MSG);}while(0)
#define LOGF(LVL, MSG, ...)		do{if(runningAsDaemon) syslog(LVL, MSG, __VA_ARGS__); \
					else fprintf(stderr, MSG, __VA_ARGS__);}while(0)

#define SIOCSADDCANIF			0x3001
#define SIOCSADDETHIF			0x3002
#define SIOCSREMOVECANIF		0x3003
#define SIOCSREMOVEETHIF		0x3004
#define SIOCGSHAREDMEMSIZE		0x3005
#define SIOCSMSGSWRITTEN		0x3006
#define SIOCGMAXIFACES			0x3007
#define SIOCGVERSION			0x3008
#define SIOCGCLIENTVEROK		0x3009
#define SIOCSBAUDRATE			0x300A
#define SIOCSERRCOUNT           0x300B
#define SIOCSIFSETTINGS         0x300C

#define RX_BOX_SIZE			(sharedMemSize / (maxInterfaces * 2))
#define TX_BOX_SIZE			(sharedMemSize / 4)
#define GET_RX_BOX(DEVICE_INDEX)	(reinterpret_cast<uint8_t*>(sharedMemory) + (RX_BOX_SIZE * DEVICE_INDEX))
#define GET_TX_BOX(INDEX)		(reinterpret_cast<uint8_t*>(sharedMemory) + (sharedMemSize / 2) + (INDEX * TX_BOX_SIZE))

#define DEFAULT_SCAN_INTERVAL_MS 1000

bool runningAsDaemon = false;
int driver = 0; // /dev/intrepid_netdevice
int driverMajor = 0;
int driverMinor = 0;
int driverPatch = 0;
int maxInterfaces = 0; // From driver
int sharedMemSize = 0; // From driver
void* sharedMemory = nullptr;
std::string serialFilter;
int scanIntervalMs = DEFAULT_SCAN_INTERVAL_MS;

std::atomic<bool> stopRunning(false);

struct intrepid_pending_tx_info {
	int tx_box_index;
	int count;
	size_t bytes;
};

#define ICS_MAGIC 0x49435343 // ICSC

struct add_can_if_info {
    char alias[IFALIASZ];
    __u32 magic;
    __u32 ctrl_mode;
    struct can_clock clock;
    struct can_bittiming_const bittiming_const;
    struct can_bittiming_const data_bittiming_const;
};

struct can_err_report {
    int device;
    enum can_state state;
    struct can_berr_counter err_count;
};

struct can_dev_settings {
    int device;
    struct can_bittiming bittiming;
    struct can_bittiming data_bittiming;
    __u32 ctrl_mode;
    bool termination;
};

static struct can_clock clock_bxcan = {
    .freq = 80000000,
};
static struct can_bittiming_const bittiming_const_bxcan = {
    .name = "bxcan-31X",
    .tseg1_min = 2,		/* Time segment 1 = prop_seg + phase_seg1 */
    .tseg1_max = 256,
    .tseg2_min = 2,		/* Time segment 2 = phase_seg2 */
    .tseg2_max = 128,
    .sjw_max = 128,
    .brp_min = 1,
    .brp_max = 512,
    .brp_inc = 1,
};
static struct can_bittiming_const data_bittiming_const_bxcan = {
    .name = "bxcan-31X",
    .tseg1_min = 1,		/* Time segment 1 = prop_seg + phase_seg1 */
    .tseg1_max = 32,
    .tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
    .tseg2_max = 16,
    .sjw_max = 16,
    .brp_min = 1,
    .brp_max = 32,
    .brp_inc = 1,
};

static struct can_clock clock_dspic = {
    .freq = 40000000,
};

static struct can_bittiming_const bittiming_const_dspic = {
    .name = "dspic33fj",
    .tseg1_min = 1,		/* Time segment 1 = prop_seg + phase_seg1 */
    .tseg1_max = 8,
    .tseg2_min = 1,		/* Time segment 2 = phase_seg2 */
    .tseg2_max = 8,
    .sjw_max = 4,
    .brp_min = 2,
    .brp_max = 128,
    .brp_inc = 1,
};

static struct can_dev_info {
    devicetype_t device_type;
    struct can_clock *clock;
    struct can_bittiming_const *bittiming_const;
    struct can_bittiming_const *data_bittiming_const;
} dev_infos[] = {
    { icsneo::DeviceType::Enum::ECU_AVB, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADMars, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::VCAN4_1, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADPluto, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::VCAN4_2EL, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::FIRE3, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADJupiter, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::VCAN4_IND, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADGigastar, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RED2, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RAD_A2B, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADEpsilon, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADMoon3, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADComet, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::FIRE3_FlexRay, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::VCAN4_4, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::VCAN4_2, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::FIRE2, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADGalaxy, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::RADStar2, &clock_bxcan, &bittiming_const_bxcan, &data_bittiming_const_bxcan },
    { icsneo::DeviceType::Enum::FIRE, &clock_dspic, &bittiming_const_dspic, NULL },
    { icsneo::DeviceType::Enum::VCAN3, &clock_dspic, &bittiming_const_dspic, NULL },
    { icsneo::DeviceType::Enum::RED, &clock_dspic, &bittiming_const_dspic, NULL },
};

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*x))

static struct can_dev_info* get_infos_for_device(devicetype_t device_type)
{
    for (size_t i = 0; i < ARRAY_SIZE(dev_infos); ++i) {
        if (dev_infos[i].device_type == device_type) {
            return &dev_infos[i];
        }
    }
    return NULL;
}

class NetworkInterface {
public:
	NetworkInterface(const std::string& desiredName, icsneo::Network::Type device, devicetype_t device_type)
            : type(device), name(desiredName) {
        struct add_can_if_info info = {
            .magic = ICS_MAGIC,
        };
		strncpy(info.alias, name.c_str(), IFALIASZ);

		if(device == icsneo::Network::Type::CAN) {
            struct can_dev_info *dev_info = get_infos_for_device(device_type);
            if (dev_info) {
                info.ctrl_mode = CAN_CTRLMODE_BERR_REPORTING;
                info.clock = *(dev_info->clock);
                info.bittiming_const = *(dev_info->bittiming_const);
                if (dev_info->data_bittiming_const) {
                    info.ctrl_mode |= CAN_CTRLMODE_FD | CAN_CTRLMODE_FD_NON_ISO;
                    info.data_bittiming_const = *(dev_info->data_bittiming_const);
                }
            }
			kernelHandle = ioctl(driver, SIOCSADDCANIF, &info); // this will call the intrepid_dev_ioctl()
		} else if(device == icsneo::Network::Type::Ethernet) {
			kernelHandle = ioctl(driver, SIOCSADDETHIF, &info.alias); // this will call the intrepid_dev_ioctl()
		}

		if(openedSuccessfully()) {
            ifindex = ioctl(driver, SIOCGIFINDEX, kernelHandle);
			LOGF(LOG_INFO, "Ifindex for device %s is %d\n", name.c_str(), ifindex);
			rxBox = GET_RX_BOX(kernelHandle);
			rxBoxCurrentPosition = rxBox;
		}
	}
	~NetworkInterface() {
		if(openedSuccessfully()) {
			int res = 0;
			LOGF(LOG_DEBUG, "Removing device %s with handle %d\n", name.c_str(), kernelHandle);
			if(type == icsneo::Network::Type::CAN) {
				res = ioctl(driver, SIOCSREMOVECANIF, kernelHandle);
			} else if(type == icsneo::Network::Type::Ethernet) {
				res = ioctl(driver, SIOCSREMOVEETHIF, kernelHandle);
			}
			LOGF(LOG_DEBUG, "Removed device %s with handle %d, result %d\n", name.c_str(), kernelHandle, res);
		} else
			LOG(LOG_DEBUG, "Removing interface which was not opened successfully\n");
	}

	NetworkInterface(const NetworkInterface&) = delete;
	NetworkInterface& operator =(const NetworkInterface&) = delete;

	bool openedSuccessfully() const { return kernelHandle >= 0; }
	int getKernelHandle() const { return kernelHandle; }
    int getIfIndex() const { return ifindex; }
	const std::string& getName() const { return name; }
	uint8_t* getRxBox() { return rxBox; }
	const uint8_t* getRxBox() const { return rxBox; }

    void reportErrorCount(const std::shared_ptr<icsneo::CANErrorCountMessage>& msg) {
        LOGF(LOG_INFO, "%s CAN error count tx:%d rx:%d busoff:%d\n",
                name.c_str(), msg->transmitErrorCount, msg->receiveErrorCount,
                msg->busOff);
        struct can_err_report err = {
            .device = kernelHandle,
            .state = (msg->busOff)?CAN_STATE_BUS_OFF:CAN_STATE_ERROR_ACTIVE,
            .err_count = {
                .txerr = msg->transmitErrorCount,
                .rxerr = msg->receiveErrorCount,
            },
        };

        if(ioctl(driver, SIOCSERRCOUNT, &err) < 0) {
            LOGF(LOG_DEBUG, "error report ioctl failed %d\n", kernelHandle);
            return;
        }
    }

	template<typename T>
	void addReceivedMessageToQueue(const std::shared_ptr<icsneo::Frame>& msg) {
		const auto neomessageGeneric = icsneo::CreateNeoMessage(msg);
		if(neomessageGeneric.messageType != neomessagetype_t(icsneo::Message::Type::Frame)) {
			LOG(LOG_DEBUG, "could not create a neomessage_can_t\n");
			return;
		}

		if(msg->network.getType() == icsneo::Network::Type::CAN || msg->network.getType() == icsneo::Network::Type::Ethernet) {

			const auto& neomessage = *reinterpret_cast<const T*>(&neomessageGeneric);

			size_t bytesNeeded = sizeof(neomessage) + neomessage.length;
			std::lock_guard<std::mutex> lg(rxBoxLock);
			if(ssize_t((rxBoxCurrentPosition - rxBox) + bytesNeeded) > RX_BOX_SIZE) {
				// fail, too big!
				LOG(LOG_DEBUG, "box too small\n");
				return;
			}
			memcpy(rxBoxCurrentPosition, &neomessage, sizeof(neomessage));
			rxBoxCurrentPosition += sizeof(neomessage);
			memcpy(rxBoxCurrentPosition, neomessage.data, neomessage.length);
			rxBoxCurrentPosition += neomessage.length;
			rxBoxMessageCount++;
			if(ioctl(driver, SIOCSMSGSWRITTEN, (kernelHandle << 16) | rxBoxMessageCount) < 0) {
				LOGF(LOG_DEBUG, "send ioctl failed %d %zu\n", kernelHandle, rxBoxMessageCount);
				return;
			}
			rxBoxCurrentPosition = rxBox;
			rxBoxMessageCount = 0;
		}
	}

    void update_bittiming(struct can_bittiming *bt)
    {

         struct can_clock clock = {
                .freq = 80000000,
            };
        __u64 v64 = (__u64)bt->brp * 1000 * 1000 * 1000;
        v64 = v64 / clock.freq;
        bt->tq = (__u32)v64;

        __u32 tseg = 1 + bt->prop_seg + bt->phase_seg1 + bt->phase_seg2;
        bt->bitrate = clock.freq / (bt->brp * tseg);
        bt->sample_point = 1000 * (tseg - bt->phase_seg2) / tseg;

        bt->sjw = std::max(1U, std::min(bt->phase_seg1, bt->phase_seg2 / 2));
    }


    void storeCanSettings(const CAN_SETTINGS *can, const CANFD_SETTINGS *canfd, bool termination) {
        LOGF(LOG_INFO, "Baudrate:%d TqSeg1:%d TqSeg2:%d TqProp:%d TqSync:%d BRP:%d ifdelay:%d\n",
            can->Baudrate, can->TqSeg1, can->TqSeg2, can->TqProp, can->TqSync, can->BRP, can->innerFrameDelay25us);
        LOGF(LOG_INFO, "FD Baudrate:%d TqSeg1:%d TqSeg2:%d TqProp:%d TqSync:%d BRP:%d\n",
            canfd->FDBaudrate, canfd->FDTqSeg1, canfd->FDTqSeg2, canfd->FDTqProp, canfd->FDTqSync, canfd->FDBRP);
        LOGF(LOG_INFO, "FDMode:0x%x TransceiverMode:0x%x\n", canfd->FDMode, can->transceiver_mode);

        bit_timing.prop_seg = can->TqProp;
        bit_timing.phase_seg1 = can->TqSeg1;
        bit_timing.phase_seg2 = can->TqSeg2;
        bit_timing.brp = can->BRP + 1;
        bit_timing.sjw = can->TqSync;

        data_bit_timing.prop_seg = canfd->FDTqProp;
        data_bit_timing.phase_seg1 = canfd->FDTqSeg1;
        data_bit_timing.phase_seg2 = canfd->FDTqSeg2;
        data_bit_timing.brp = canfd->FDBRP + 1;
        data_bit_timing.sjw = canfd->FDTqSync;

        this->termination = termination;

        ctrl_mode = 0;
        switch (canfd->FDMode) {
            case NO_CANFD:
                break;
                ctrl_mode = 0;
            case CANFD_ENABLED:
            case CANFD_BRS_ENABLED:
                ctrl_mode = CAN_CTRLMODE_FD_NON_ISO;
                break;
            case CANFD_ENABLED_ISO:
            case CANFD_BRS_ENABLED_ISO:
                ctrl_mode = CAN_CTRLMODE_FD;
                break;
        }

        switch (can->transceiver_mode) {
            case LOOPBACK:
                ctrl_mode |= CAN_CTRLMODE_LOOPBACK;
                break;
            case LISTEN_ONLY:
            case LISTEN_ALL:
                ctrl_mode |= CAN_CTRLMODE_LISTENONLY;
                break;
        }

        update_bittiming(&bit_timing);
        if (canfd->FDMode != NO_CANFD) {
            update_bittiming(&data_bit_timing);
        } else {
            data_bit_timing.bitrate = 0;
        }

        struct can_dev_settings settings = {
            .device = kernelHandle,
            .bittiming = bit_timing,
            .data_bittiming = data_bit_timing,
            .ctrl_mode = ctrl_mode,
            .termination = termination,
        };
        if(ioctl(driver, SIOCSIFSETTINGS, &settings) < 0) {
            LOGF(LOG_DEBUG, "device settings ioctl failed %d\n", kernelHandle);
            return;
        }
    }

    void setBittiming(struct can_bittiming *timing, std::shared_ptr<icsneo::Device> device, icsneo::Network::NetID netid) {
        if (timing->prop_seg == bit_timing.prop_seg
                && timing->phase_seg1 == bit_timing.phase_seg1
                && timing->phase_seg2 == bit_timing.phase_seg2
                && timing->sjw == bit_timing.sjw
                && timing->brp == bit_timing.brp) {
            LOG(LOG_INFO, "no change in bittiming\n");
            return;
        }


        CAN_SETTINGS *settings = device->settings->getMutableCANSettingsFor(netid);

        settings->SetBaudrate = USE_TQ;
        settings->TqSeg1 = timing->phase_seg1;
        settings->TqSeg2 = timing->phase_seg2;
        settings->TqSync = timing->sjw;
        settings->TqProp = timing->prop_seg;
        settings->BRP = timing->brp - 1;

        LOGF(LOG_INFO, "Set Bittiming TqSeg1:%d TqSeg2:%d TqProp:%d TqSync:%d BRP:%d\n",
                settings->TqSeg1, settings->TqSeg2, settings->TqProp, settings->TqSync, settings->BRP);

        if (! device->settings->apply() ) {
            LOGF(LOG_ERR, "Unable to set bit timings for %s", name.c_str());
        }

        bit_timing = *timing;
    }

    void setDataBittiming(struct can_bittiming *timing, std::shared_ptr<icsneo::Device> device, icsneo::Network::NetID netid) {
        if (timing->prop_seg == data_bit_timing.prop_seg
                && timing->phase_seg1 == data_bit_timing.phase_seg1
                && timing->phase_seg2 == data_bit_timing.phase_seg2
                && timing->sjw == data_bit_timing.sjw
                && timing->brp == data_bit_timing.brp) {
            return;
        }
        CANFD_SETTINGS *settings = device->settings->getMutableCANFDSettingsFor(netid);

        settings->FDTqSeg1 = timing->phase_seg1;
        settings->FDTqSeg2 = timing->phase_seg2;
        settings->FDTqSync = timing->sjw;
        settings->FDTqProp = timing->prop_seg;
        settings->FDBRP = timing->brp - 1;

        if (! device->settings->apply() ) {
            LOGF(LOG_ERR, "Unable to set data bit timings for %s", name.c_str());
        }

        data_bit_timing = *timing;
    }

    void setCtrlMode(uint32_t mode, std::shared_ptr<icsneo::Device> device, icsneo::Network::NetID netid) {
        if (mode == ctrl_mode) {
            return;
        }

        if ((mode & (CAN_CTRLMODE_FD_NON_ISO | CAN_CTRLMODE_FD))
                != (ctrl_mode & (CAN_CTRLMODE_FD_NON_ISO | CAN_CTRLMODE_FD))) {
            CANFD_SETTINGS *settings = device->settings->getMutableCANFDSettingsFor(netid);
            if (mode & CAN_CTRLMODE_FD_NON_ISO) {
                if (bit_timing.bitrate == data_bit_timing.bitrate) {
                    settings->FDMode = CANFD_ENABLED;
                } else {
                    settings->FDMode = CANFD_BRS_ENABLED;
                }
            } else if (mode & CAN_CTRLMODE_FD) {
                if (bit_timing.bitrate == data_bit_timing.bitrate) {
                    settings->FDMode = CANFD_ENABLED_ISO;
                } else {
                    settings->FDMode = CANFD_BRS_ENABLED_ISO;
                }
            } else {
                settings->FDMode = NO_CANFD;
            }
        }

        if ((mode & (CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_LOOPBACK))
                != (ctrl_mode & (CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_LOOPBACK))) {
            CAN_SETTINGS *settings = device->settings->getMutableCANSettingsFor(netid);
            if (mode & CAN_CTRLMODE_LISTENONLY) {
                settings->transceiver_mode = LISTEN_ONLY;
            } else if (mode & CAN_CTRLMODE_LOOPBACK) {
                settings->transceiver_mode = LOOPBACK;
            } else {
                settings->transceiver_mode = NORMAL;
            }
        }

        if (! device->settings->apply() ) {
            LOGF(LOG_ERR, "Unable to set controller mode for %s", name.c_str());
        }
        ctrl_mode = mode;
    }

    void setTermination(bool termination, std::shared_ptr<icsneo::Device> device, icsneo::Network::NetID netid) {
        if (termination != this->termination) {
            if (! device->settings->setTerminationFor(netid, termination) ||
                    ! device->settings->apply() ) {
                LOGF(LOG_ERR, "Unable to set termination for %s", name.c_str());
            }
            this->termination = termination;
        }
    }

private:
	icsneo::Network::Type type;
	std::string name;
	int kernelHandle = -1;
    int ifindex = -1;
	std::mutex rxBoxLock;
	uint8_t* rxBox = nullptr;
	uint8_t* rxBoxCurrentPosition = nullptr;
	size_t rxBoxMessageCount = 0;
    struct can_bittiming bit_timing;
    struct can_bittiming data_bit_timing;
    uint32_t ctrl_mode;
    bool termination;
};

class OpenDevice {
public:
	OpenDevice(const std::shared_ptr<icsneo::Device>& openDevice) : device(openDevice) {}
	std::shared_ptr<icsneo::Device> device;
	std::map<icsneo::Network::NetID, std::shared_ptr<NetworkInterface>> interfaces;

	bool operator ==(const std::shared_ptr<icsneo::Device>& other) const {
		return device->getSerial() == other->getSerial();
	}
};

template <typename T>
class Lazy {
public:
	Lazy(std::function<T()> f) : fn(f) {}
	operator T() {
		if(!valid)
			evaluate();
		return result;
	}
	void invalidate() { valid = false; }
	void evaluate() {
		result = fn();
		valid = true;
	}
private:
	T result;
	bool valid = false;
	std::function<T()> fn;
};

std::vector<OpenDevice> openDevices;
std::vector<std::string /* serial */> failedToOpen;
std::mutex openDevicesMutex;

std::string& replaceInPlace(std::string& str, char o, const std::string& n) {
	size_t start_pos = 0;
	const size_t new_len = n.length();
	while((start_pos = str.find(o, start_pos)) != std::string::npos) {
		str.replace(start_pos, 1, n);
		start_pos += new_len;
	}
	return str;
}

std::string sanitizeInterfaceName(std::string str) {
	static const std::string nullString = "";
	replaceInPlace(str, ' ', nullString);
	std::transform(str.begin(), str.end(), str.begin(), ::tolower);
	return str;
}

void header() {
	std::cout << "The libicsneo SocketCAN Usermode Daemon\n";
	std::cout << "Copyright Intrepid Control Systems, Inc. 2024\n\n";
	std::cout << "Daemon v";
	std::cout << (int)ICSNEO_SOCKETCAN_BUILD_MAJOR << '.' << (int)ICSNEO_SOCKETCAN_BUILD_MINOR << '.' << (int)ICSNEO_SOCKETCAN_BUILD_PATCH;
	if(ICSNEO_SOCKETCAN_BUILD_METADATA[0] != '\0')
		std::cout << '+' << ICSNEO_SOCKETCAN_BUILD_METADATA;
	std::string describe(ICSNEO_SOCKETCAN_GIT_DESCRIBE);
	if(describe.find("fatal") != 0) {
		if(std::string(ICSNEO_SOCKETCAN_GIT_BRANCH) != "master")
			std::cout << ' ' << ICSNEO_SOCKETCAN_GIT_BRANCH;
		if(describe[0] != 'v')
			std::cout << " @ " << describe;
	}
	std::cout << "\nlibicsneo " << icsneo::GetVersion() << "\n";
}

void usage(std::string executableName) {
	std::cerr << "The libicsneo SocketCAN Usermode Daemon\n";
	std::cerr << "Copyright 2019-2024 Intrepid Control Systems, Inc.\n\n";
	std::cerr << "Usage: " << executableName << " [option]\n\n";
	std::cerr << "Options:\n";
	std::cerr << "\t-d,     --daemon\t\t\tRun as a daemon in the background\n";
	std::cerr << "\t-h, -?, --help, --usage\t\t\tShow this help page\n";
	std::cerr << "\t        --devices\t\t\tList supported devices\n";
	std::cerr << "\t        --filter <serial>\t\tOnly connect to devices with serial\n\t\t\t\t\t\tnumbers starting with this filter\n";
	std::cerr << "\t        --scan-interval-ms <interval>\tDevice scan interval in ms\n\t\t\t\t\t\tIf 0, only a single scan is performed\n";
}

void terminateSignal(int signal) {
	stopRunning = true;
}

void searchForDevices() {
	auto found = icsneo::FindAllDevices();
	std::lock_guard<std::mutex> lg(openDevicesMutex);

	// Open devices we have not seen before
	for(auto& dev : found) {
		bool alreadyOpen = false;
		for(const auto& openDev : openDevices) {
			if(openDev == dev) {
				alreadyOpen = true;
				break;
			}
		}
		if(alreadyOpen)
			continue;

		const std::string serial = dev->getSerial();

		// If we have a serial filter, make sure our serial starts with the given filter
		if(!serialFilter.empty() && serial.rfind(serialFilter, 0) != 0)
			continue;

		// Now open the device
		OpenDevice newDevice(dev);
		Lazy<bool> firstTimeFailedToOpen([&serial]() {
			return std::find(failedToOpen.begin(), failedToOpen.end(), serial) == failedToOpen.end();
		});
		if(!newDevice.device->open() || !newDevice.device->goOnline()) {
			if(firstTimeFailedToOpen) {
				const std::string err = icsneo::GetLastError().describe();
				LOGF(LOG_INFO, "%s failed to connect. Will keep trying...\n%s\n", newDevice.device->describe().c_str(), err.c_str());
				failedToOpen.push_back(serial);
			}
			continue;
		}

		// Get the supported networks
		auto supportedNetworks = newDevice.device->getSupportedRXNetworks();
		supportedNetworks.erase(std::remove_if(supportedNetworks.begin(), supportedNetworks.end(), [](const icsneo::Network& net) -> bool {
			return net.getType() != icsneo::Network::Type::CAN && net.getType() != icsneo::Network::Type::Ethernet;
		}), supportedNetworks.end());
		if(supportedNetworks.empty()) {
			if(firstTimeFailedToOpen) {
				LOGF(LOG_INFO, "%s has no supported networks\n", newDevice.device->describe().c_str());
				failedToOpen.push_back(serial);
			}
			continue;
		}

		// Create a network interface for each network
		for(const auto& net : supportedNetworks) {
			std::stringstream ss;
			ss << sanitizeInterfaceName(icsneo::Network::GetNetIDString(net.getNetID())) << "_" << serial;
			std::string interfaceName(ss.str());
			if(firstTimeFailedToOpen)
				LOGF(LOG_INFO, "Creating network interface %s\n", interfaceName.c_str());

			newDevice.interfaces[net.getNetID()] = std::make_shared<NetworkInterface>(interfaceName, net.getType(), newDevice.device->getType());
			LOGF(LOG_INFO, "Created network interface %s\n", interfaceName.c_str());
		}
		bool failedToCreateNetworkInterfaces = false;
		for(const auto& iface : newDevice.interfaces) {
			if(!iface.second->openedSuccessfully()) {
				failedToCreateNetworkInterfaces = true;
				break;
			}
		}
		if(failedToCreateNetworkInterfaces) {
			if(firstTimeFailedToOpen) {
				LOGF(LOG_INFO, "%s failed to create network interfaces. Will keep trying...\n", newDevice.device->describe().c_str());
				failedToOpen.push_back(serial);
			}

			continue;
		}
			if (driverMinor > 0) {
				for(const auto& net : supportedNetworks) {
					if (net.getType() != icsneo::Network::Type::CAN)
						continue;
                    const CAN_SETTINGS *can = newDevice.device->settings->getCANSettingsFor(net.getNetID());
                    const CANFD_SETTINGS *fd = newDevice.device->settings->getCANFDSettingsFor(net.getNetID());
                    bool termination = newDevice.device->settings->isTerminationEnabledFor(net.getNetID())
                        .value_or(false);
                    newDevice.interfaces[net.getNetID()]->storeCanSettings(can, fd, termination);
				}
			}

		// Create rx listener
		newDevice.device->addMessageCallback(std::make_shared<icsneo::MessageCallback>([serial](std::shared_ptr<icsneo::Message> message) {
			const auto frame = std::static_pointer_cast<icsneo::Frame>(message);
			const auto messageType = frame->network.getType();
			const OpenDevice* openDevice = nullptr;
			std::lock_guard<std::mutex> lg(openDevicesMutex);
			for(const auto& dev : openDevices) {
				if(dev.device->getSerial() == serial) {
					openDevice = &dev;
					break;
				}
			}
			if(frame->type == icsneo::Message::Type::CANErrorCount) {
                const auto errmsg = std::static_pointer_cast<icsneo::CANErrorCountMessage>(message);
                openDevice->interfaces.at(frame->network.getNetID())->reportErrorCount(errmsg);
                return;
            }
			if(frame->type != icsneo::Message::Type::Frame) {
				LOG(LOG_ERR, "Dropping message: received invalid message type, expected RawMessage\n");
				return;
			}

			if(messageType == icsneo::Network::Type::CAN) {
				openDevice->interfaces.at(frame->network.getNetID())->addReceivedMessageToQueue<neomessage_can_t>(frame);
			} else if(messageType == icsneo::Network::Type::Ethernet) {
				openDevice->interfaces.at(frame->network.getNetID())->addReceivedMessageToQueue<neomessage_eth_t>(frame);
			} else
				LOG(LOG_ERR, "Dropping message, only CAN and Ethernet are currently supported\n");
		}));

		LOGF(LOG_INFO, "%s connected\n", newDevice.device->describe().c_str());
		failedToOpen.erase(std::remove_if(failedToOpen.begin(), failedToOpen.end(), [&serial](const std::string& s) -> bool {
			return serial == s;
		}), failedToOpen.end());
		openDevices.push_back(std::move(newDevice));
	}

	// Close devices we don't see anymore
	openDevices.erase(
		std::remove_if(
			openDevices.begin(),
			openDevices.end(),
			[&found](OpenDevice& openDev) -> bool {
				bool stillHere = false;
				for(const auto& dev : found) {
					if(openDev == dev) {
						stillHere = true;
						break;
					}
				}
				if(stillHere)
					return false;
				// The device is closed and the networks are removed by virtue of removing it from the array
				LOGF(LOG_INFO, "%s disconnected\n", openDev.device->describe().c_str());
				return true;
			}
		),
		openDevices.end()
	);

	for(const auto& err : icsneo::GetEvents()) {
		bool forErrorDevice = false;
		for(const auto& dev : failedToOpen) {
			if(err.isForDevice(dev)) {
				forErrorDevice = true;
				break;
			}
		}
		if(forErrorDevice)
			continue;
		std::string description = err.describe();
		description += "\n";
		LOGF(LOG_INFO, "%s", description.c_str());
	}
}

void deviceSearchThread() {
	while(!stopRunning) {
		searchForDevices();
		if(scanIntervalMs == 0) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(scanIntervalMs));
	}
}

int main(int argc, char** argv) {
	for(int i = 1; i != argc; i++) {
		const std::string arg = argv[i];
		if(arg == "-d" || arg == "--daemon") {
			runningAsDaemon = true;
		} else if(arg == "-h" || arg == "--help" || arg == "-?" || arg == "--usage") {
			usage(argv[0]);
			return EXIT_SUCCESS;
		} else if(arg == "--devices") {
			header();
			std::cout<< "\nSupported devices:" << std::endl;
			for(auto& dev : icsneo::GetSupportedDevices())
				std::cout << '\t' << dev << std::endl;
			return EXIT_SUCCESS;
		} else if(arg == "--filter" && i + 1 <= argc) {
			serialFilter = argv[++i];
			transform(serialFilter.begin(), serialFilter.end(), serialFilter.begin(), ::toupper);
		} else if(arg == "--scan-interval-ms" && i + 1 <= argc) {
			try {
				scanIntervalMs = std::stoi(argv[++i]);
			} catch (const std::invalid_argument& e) {
				std::cerr << "Invalid input for scan-interval-ms\n";
				return EX_USAGE;
			} catch (const std::out_of_range& e) {
				std::cerr << "Out of range input for scan-interval-ms\n";
				return EX_USAGE;
			}

			if(scanIntervalMs < 0) {
				std::cerr << "Invalid input for scan-interval-ms\n";
				return EX_USAGE;
			}
		} else {
			usage(argv[0]);
			return EX_USAGE;
		}
	}

	header();

	// Open the /dev/intrepid_netdevice kernel driver
	if((driver = open("/dev/intrepid_netdevice", O_RDWR | O_NONBLOCK)) <= 0) {
		std::cout << '\n'; // Still printing versions
		LOGF(LOG_ERR, "Could not open the kernel driver\nError %d: %s\n", errno, strerror(errno));
		switch(errno) {
		case 2: // No such file or directory
			LOG(LOG_ERR, "\nThis usually happens if the driver has not been loaded with insmod\n");
			break;
		case 5: // Input/output error
			LOG(LOG_ERR, "\nThis usually happens if there is already a daemon running\n");
			break;
		case 13: // Permission denied
			LOG(LOG_ERR, "\nThis usually happens if the daemon is not being run as root (use sudo)\n");
			break;
		}
		return EXIT_FAILURE;
	}

	// Read out version from the driver
	int version = 0;
	if((version = ioctl(driver, SIOCGVERSION)) <= 0) {
		std::cout << '\n'; // Still printing versions
		LOGF(LOG_ERR, "Error reading the version from the kernel driver\nError %d: %s\n\nThis can happen if the driver is too old",
			errno, strerror(errno));
		return EXIT_FAILURE;
	}
	driverMajor = (version >> 16) & 0xFF;
	driverMinor = (version >> 8) & 0xFF;
	driverPatch = version & 0xFF;
	if(driverMajor < 2) {
		std::cout << '\n'; // Still printing versions
		LOGF(LOG_ERR, "Error reading the version from the kernel driver\nError version %d.%d.%d\n", driverMajor, driverMinor, driverPatch);
		return EXIT_FAILURE;
	}
	std::cout << "Driver v" << driverMajor << '.' << driverMinor << '.' << driverPatch << "\n\n";
	if(driverMajor > 3) {
		LOG(LOG_ERR, "This version of the usermode daemon is too old to work with this driver\nPlease ensure that both the usermode daemon and kernel driver are up to date\n");
		return EXIT_FAILURE;
	}

	if(ioctl(driver, SIOCGCLIENTVEROK, ICSNEO_SOCKETCAN_BUILD_VERINT) != 0) {
		LOG(LOG_ERR, "The kernel driver reports an incompatibility with this version of the usermode daemon\nPlease ensure that both the usermode daemon and kernel driver are up to date\n");
		return EXIT_FAILURE;
	}

	// Read out other constants from the driver
	if((maxInterfaces = ioctl(driver, SIOCGMAXIFACES)) <= 0) {
		LOGF(LOG_ERR, "Error reading the maximum number of interfaces from the kernel driver\nError %d: %s\n", errno, strerror(errno));
		return EXIT_FAILURE;
	}
	if((sharedMemSize = ioctl(driver, SIOCGSHAREDMEMSIZE)) <= 0) {
		LOGF(LOG_ERR, "Error reading the shared memory size from the kernel driver\nError %d: %s\n", errno, strerror(errno));
		return EXIT_FAILURE;
	}

	// Set up shared memory
	if((sharedMemory = mmap(nullptr, sharedMemSize, PROT_READ | PROT_WRITE, MAP_SHARED, driver, 0)) == MAP_FAILED || sharedMemory == nullptr) {
		LOG(LOG_ERR, "Error setting up shared memory with the kernel driver\n");
		return EXIT_FAILURE;
	}

    int netlink_socket = open_netlink_socket();
    if (netlink_socket < 0) {
        LOGF(LOG_ERR, "Unable to open netlink socket\nError %d: %s\n", errno, strerror(errno));
        return EXIT_FAILURE;
    }

	// Daemonize if necessary
	if(runningAsDaemon) {
		LOG(LOG_INFO, "The daemon will now continue to run in the background\n");
		openlog("icsneo-socketcan", LOG_PID, LOG_LOCAL5);
		if(daemon(0 /* change pwd to root */, 0 /* no stdout or stderr anymore */)) {
			std::cerr << "Failed to spawn the daemon. Exiting...\n";
			return EXIT_FAILURE;
		}
	} else {
		signal(SIGINT, terminateSignal);
		LOG(LOG_INFO, "Waiting for connections...\n");
	}

	std::thread searchThread(deviceSearchThread);

	while(!stopRunning) {
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(driver, &fds);
		FD_SET(netlink_socket, &fds);

        int max_fd = (driver > netlink_socket)?driver:netlink_socket;

		struct timeval timeout = {};
		timeout.tv_sec = 1;

		auto ret = select(max_fd + 1, &fds, NULL, NULL, &timeout);
		if(ret == -1) {
			// Fatal error
			LOGF(LOG_ERR, "Error waiting for tx messages: %s\n", strerror(errno));
			stopRunning = true;
			break;
        }
        if (FD_ISSET(netlink_socket, &fds)) {
            // Kernel sent some information via netlink, handle it.
            read_netlink_msgs(netlink_socket, [](int ifindex, int type, void *data) {
				for(auto& dev : openDevices) {
					for(auto& netifPair : dev.interfaces) {
						auto netid = netifPair.first;
                        auto iface = netifPair.second;
                        if (iface->getIfIndex() != ifindex) {
                            continue;
                        }
                        switch (type) {
                            case IFLA_CAN_BITTIMING:
                                iface->setBittiming((struct can_bittiming *) data, dev.device, netid);
                                break;
                            case IFLA_CAN_DATA_BITTIMING:
                                iface->setDataBittiming((struct can_bittiming *) data, dev.device, netid);
                                break;
                            case IFLA_CAN_TERMINATION:
                                iface->setTermination(*((uint16_t *) data) != 0, dev.device, netid);
                                break;
                            case IFLA_CAN_CTRLMODE:
                                iface->setCtrlMode(((struct can_ctrlmode *) data)->flags, dev.device, netid);
                                break;
                        }
                    }
                }
            });
        }
		if (FD_ISSET(driver, &fds)) {
			// Kernel says there are some new transmit messages waiting to go out.
			// Call read() to find out which box they're in and how many
			struct intrepid_pending_tx_info info;
			ssize_t r = read(driver, &info, sizeof(info));
			if(r == -1) {
				LOGF(LOG_ERR, "Error waiting for tx messages: %s\n", strerror(errno));
				stopRunning = true;
				break;
			} else if(r != sizeof(info)) {
				LOGF(LOG_ERR, "Unexpected number of bytes read, expected %d got %d\n", (int)sizeof(info), (int)r);
				stopRunning = true;
				break;
			} else if (info.tx_box_index < 0) {
				// Baudrate changed in kernel
				int dev_idx = -(info.tx_box_index + 1);
				LOGF(LOG_INFO, "Baudrate change, device %d, baudrate %d fd_baudrate %ld\n",
					dev_idx, info.count, info.bytes);
				/* fd baudrate is zero if fd mode is disabled in kernel
				 * set fd baudrate equal to baudrate */
				if (info.bytes == 0) {
					info.bytes = info.count;
				}
				for(auto& dev : openDevices) {
					for(auto& netifPair : dev.interfaces) {
						auto netid = netifPair.first;
						if(netifPair.second->getKernelHandle() != dev_idx)
							continue;
						if (! dev.device->settings->setBaudrateFor(netid, info.count) ) {
							LOGF(LOG_ERR, "Unable to set baudrate for device %s\n",
								netifPair.second->getName().c_str());
						} else if (! dev.device->settings->setFDBaudrateFor(netid, info.bytes)) {
							LOGF(LOG_ERR, "Unable to set fd baudrate for device %s\n",
								netifPair.second->getName().c_str());
						} else if (! dev.device->settings->setTerminationFor(netid, false)) {
							LOGF(LOG_ERR, "Unable to set termination for device %s\n",
								netifPair.second->getName().c_str());
						} else if (! dev.device->settings->apply()) {
							LOGF(LOG_ERR, "Unable to apply settings for device %s\n",
								netifPair.second->getName().c_str());
						}
					}
				}
			} else {
				// Send!
				uint8_t* currentPosition = GET_TX_BOX(info.tx_box_index);
				while(info.count--) {
					neomessage_frame_t* msg = reinterpret_cast<neomessage_frame_t*>(currentPosition);
					currentPosition += sizeof(neomessage_frame_t);
					msg->data = currentPosition;
					currentPosition += msg->length;

					if(msg->type != neonettype_t(icsneo::Network::Type::CAN) && msg->type != neonettype_t(icsneo::Network::Type::Ethernet)) {
						LOG(LOG_ERR, "Message dropped, kernel sent a non-CAN/Ethernet message\n");
						continue;
					}

					bool sent = false;
					std::lock_guard<std::mutex> lg(openDevicesMutex);
					for(auto& dev : openDevices) {
						for(auto& netifPair : dev.interfaces) {
							if(netifPair.second->getKernelHandle() != msg->netid)
								continue;
							msg->netid = static_cast<uint16_t>(netifPair.first);
							auto txMsg = icsneo::CreateMessageFromNeoMessage(reinterpret_cast<neomessage_t*>(msg));
							auto tx = std::dynamic_pointer_cast<icsneo::Frame>(txMsg);
							if(!tx || !dev.device->transmit(tx))
								break;
							sent = true;
							break;
						}
						if(sent)
							break;
					}
					if(!sent)
						LOG(LOG_ERR, "Message dropped, could not find the device the kernel referenced\n");
				}
			}
		}
	}

	searchThread.join();

	LOG(LOG_INFO, "\nExiting...\n");
	return EXIT_SUCCESS;
}
