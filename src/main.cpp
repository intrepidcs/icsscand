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

#include <icsneo/icsneocpp.h>
#include <icsneo/communication/message/neomessage.h>
#include <icsneo/communication/message/callback/canmessagecallback.h>
#include <generated/buildinfo.h>

#define LOG(LVL, MSG)              do{if(runningAsDaemon) syslog(LVL, MSG); \
                                    else fprintf(stderr, MSG);}while(0)
#define LOGF(LVL, MSG, ...)        do{if(runningAsDaemon) syslog(LVL, MSG, __VA_ARGS__); \
                                    else fprintf(stderr, MSG, __VA_ARGS__);}while(0)

#define SIOCSADDIF                 0x3001
#define SIOCSREMOVEIF              0x3002
#define SIOCGSHAREDMEMSIZE         0x3003
#define SIOCSMSGSWRITTEN           0x3004
#define SIOCGMAXIFACES             0x3005
#define SIOCGVERSION               0x3006
#define SIOCGCLIENTVEROK           0x3007

#define RX_BOX_SIZE                (sharedMemSize / (maxInterfaces * 2))
#define TX_BOX_SIZE                (sharedMemSize / 4)
#define GET_RX_BOX(DEVICE_INDEX)   (reinterpret_cast<uint8_t*>(sharedMemory) + (RX_BOX_SIZE * DEVICE_INDEX))
#define GET_TX_BOX(INDEX)          (reinterpret_cast<uint8_t*>(sharedMemory) + (sharedMemSize / 2) + (INDEX * TX_BOX_SIZE))

bool runningAsDaemon = false;
int driver = 0; // /dev/intrepid_netdevice
int driverMajor = 0;
int driverMinor = 0;
int driverPatch = 0;
int maxInterfaces = 0; // From driver
int sharedMemSize = 0; // From driver
void* sharedMemory = nullptr;
std::string serialFilter;

std::atomic<bool> stopRunning(false);

struct intrepid_pending_tx_info {
	int tx_box_index;
	int count;
	size_t bytes;
};

class NetworkInterface {
public:
	NetworkInterface(const std::string& desiredName) : name(desiredName) {
		char ifname[IFALIASZ + 1] = {0};
		strncpy(ifname, name.c_str(), IFALIASZ);
		kernelHandle = ioctl(driver, SIOCSADDIF, ifname);
		if(openedSuccessfully()) {
			rxBox = GET_RX_BOX(kernelHandle);
			rxBoxCurrentPosition = rxBox;
		}
	}
	~NetworkInterface() {
		if(openedSuccessfully()) {
			LOGF(LOG_DEBUG, "Removing device %s with handle %d\n", name.c_str(), kernelHandle);
			int res = ioctl(driver, SIOCSREMOVEIF, kernelHandle);
			LOGF(LOG_DEBUG, "Removed device %s with handle %d, result %d\n", name.c_str(), kernelHandle, res);
		} else
			LOG(LOG_DEBUG, "Removing interface which was not opened successfully\n");
	}
	NetworkInterface(const NetworkInterface&) = delete;
	NetworkInterface& operator =(const NetworkInterface&) = delete;

	bool openedSuccessfully() const { return kernelHandle >= 0; }
	int getKernelHandle() const { return kernelHandle; }
	const std::string& getName() const { return name; }
	uint8_t* getRxBox() { return rxBox; }
	const uint8_t* getRxBox() const { return rxBox; }
	void addReceivedMessageToQueue(const std::shared_ptr<icsneo::Message>& msg) {
		auto neomessage = icsneo::CreateNeoMessage(msg);
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

private:
	std::string name;
	int kernelHandle = -1;
	std::mutex rxBoxLock;
	uint8_t* rxBox = nullptr;
	uint8_t* rxBoxCurrentPosition = nullptr;
	size_t rxBoxMessageCount = 0;
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
	std::cout << "Copyright Intrepid Control Systems, Inc. 2019\n\n";
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
	std::cerr << "Copyright 2019-2020 Intrepid Control Systems, Inc.\n\n";
	std::cerr << "Usage: " << executableName << " [option]\n\n";
	std::cerr << "Options:\n";
	std::cerr << "\t-d,     --daemon\t\tRun as a daemon in the background\n";
	std::cerr << "\t-h, -?, --help, --usage\t\tShow this help page\n";
	std::cerr << "\t        --devices\t\tList supported devices\n";
	std::cerr << "\t        --filter <serial>\tOnly connect to devices with serial\n\t\t\t\t\tnumbers starting with this filter\n";
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
		
		// Get the supported CAN networks
		auto supportedNetworks = newDevice.device->getSupportedRXNetworks();
		supportedNetworks.erase(std::remove_if(supportedNetworks.begin(), supportedNetworks.end(), [](const icsneo::Network& net) -> bool {
			return net.getType() != icsneo::Network::Type::CAN;// Only want CAN networks
		}), supportedNetworks.end());
		if(supportedNetworks.empty()) {
			if(firstTimeFailedToOpen) {
				LOGF(LOG_INFO, "%s has no supported CAN networks\n", newDevice.device->describe().c_str());
				failedToOpen.push_back(serial);
			}
			continue;
		}

		// Create a network interface for each CAN network
		for(const auto& net : supportedNetworks) {
			std::stringstream ss;
			ss << sanitizeInterfaceName(icsneo::Network::GetNetIDString(net.getNetID())) << "_" << serial;
			std::string interfaceName(ss.str());
			if(firstTimeFailedToOpen)
				LOGF(LOG_INFO, "Creating network interface %s\n", interfaceName.c_str());
			newDevice.interfaces[net.getNetID()] = std::make_shared<NetworkInterface>(interfaceName);
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

		// Create rx listener
		newDevice.device->addMessageCallback(icsneo::CANMessageCallback([serial](std::shared_ptr<icsneo::Message> message) {
			auto canMessage = std::static_pointer_cast<icsneo::CANMessage>(message);
			const OpenDevice* openDevice = nullptr;
			std::lock_guard<std::mutex> lg(openDevicesMutex);
			for(const auto& dev : openDevices) {
				if(dev.device->getSerial() == serial) {
					openDevice = &dev;
					break;
				}
			}
			if(!openDevice) {
				LOG(LOG_ERR, "Dropping message, no open device\n");
				return;
			}

			// todo might throw
			openDevice->interfaces.at(canMessage->network.getNetID())->addReceivedMessageToQueue(canMessage);
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
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
	if(driverMajor > 2) {
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

		struct timeval timeout = {};
		timeout.tv_sec = 1;

		auto ret = select(driver + 1, &fds, NULL, NULL, &timeout);
		if(ret == -1) {
			// Fatal error
			LOGF(LOG_ERR, "Error waiting for tx messages: %s\n", strerror(errno));
			stopRunning = true;
			break;
		} else if(ret != 0) {
			// Kernel says there are some new transmit messages waiting to go out.
			// Call read() to find out which box they're in and how many
			struct intrepid_pending_tx_info info;
			ssize_t r = read(driver, &info, sizeof(info));
			if (r == -1) {
				LOGF(LOG_ERR, "Error waiting for tx messages: %s\n", strerror(errno));
				stopRunning = true;
				break;
			} else if(r != sizeof(info)) {
				LOGF(LOG_ERR, "Unexpected number of bytes read, expected %d got %d\n", (int)sizeof(info), (int)r);
				stopRunning = true;
				break;
			} else {
				// Send!
				uint8_t* currentPosition = GET_TX_BOX(info.tx_box_index);
				while(info.count--) {
					neomessage_t* msg = reinterpret_cast<neomessage_t*>(currentPosition);
					currentPosition += sizeof(neomessage_t);
					msg->data = currentPosition;
					currentPosition += msg->length;
					bool sent = false;
					std::lock_guard<std::mutex> lg(openDevicesMutex);
					for(auto& dev : openDevices) {
						for(auto& netifPair : dev.interfaces) {
							if(netifPair.second->getKernelHandle() != msg->netid)
								continue;
							msg->netid = static_cast<uint16_t>(netifPair.first);
							auto tx = icsneo::CreateMessageFromNeoMessage(msg);
							if(!dev.device->transmit(tx))
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