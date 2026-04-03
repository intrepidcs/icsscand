# RPC

icsscand contains an RPC endpoint that can be used to control the libicsneo
instance within the daemon. To enable the RPC channel, add `--fifo-path <path>`
during launching. The provided path will be used to open a named FIFO, the path
should not exist prior to launching icsscand.

## lock_networks

Invokes `icsneo::Device::lockNetworks()` with the provided space separated
arguments and writes the results of the call to the provided client FIFO. This
call in non-blocking, with the return type indicating that the request was sent
to the device (it does not indicate that the network was successfully locked).
Use `get_network_mutex_status` to check the status of the lock request.

### Usage

- Request: `<api version> lock_networks <device serial> <netid integers, comma separated> <priority> <ttl, ms> <lock type> <client fifo path>`
- Response: `<0 or 1, with 0 indicating an error and 1 indicating success>`
  - On error, check the icsscand logs for more detailed information

### Example

- API version: 1
- Service serial: ON0123
- Networks: `ETHERNET_01` & `ETHERNET_02`
  - See `icsneo::Network::NetID` for values
- Priority: 2
- TTL (in ms): 1s
- Opened client FIFO path: `/tmp/tmp.AYkIg0b3np`
  - This FIFO must be opened prior to invoking the RPC

`1 lock_networks ON0123 93,520 2 1000 /tmp/tmp.AYkIg0b3np`

## unlock_networks

Invokes `icsneo::Device::unlockNetworks()` with the provided space separated
arguments and writes the results of the call to the provided client FIFO.

### Usage

- Request: `<api version> unlock_networks <device serial> <netid integers, comma separated> <client fifo path>`
- Response: `<0 or 1, with 0 indicating an error and 1 indicating success>`
  - On error, check the icsscand logs for more detailed information

### Example

- API version: 1
- Service serial: ON0123
- Networks: `ETHERNET_01` & `ETHERNET_02`
  - See `icsneo::Network::NetID` for values
- Opened client FIFO path: `/tmp/tmp.KXY8SCXtux`
  - This FIFO must be opened prior to invoking the RPC

`1 lock_networks ON0123 93,520 /tmp/tmp.KXY8SCXtux`

## get_network_mutex_status

Invokes `icsneo::Device::getNetworkMutexStatus()` with the provided space separated
arguments and writes the results of the call to the provided client FIFO.

### Usage

- Request: `<api version> get_network_mutex_status <device serial> <netid integer> <client fifo path>`
- Response (one of):
  - `0`, error, check the icsscand logs for more detailed information
  - `1 <owner client id> <type> <priority> <ttl, ms> <netid integers, comma separated> <event>`

### Example

- API version: 1
- Service serial: ON0123
- Network: `ETHERNET_01`
  - See `icsneo::Network::NetID` for values
- Opened client FIFO path: `/tmp/tmp.4huaosZjhA`
  - This FIFO must be opened prior to invoking the RPC

`1 get_network_mutex_status ON0123 93 /tmp/tmp.4huaosZjhA`

## get_serials

Returns a space separated list of devices serial numbers that isscand has open.

### Usage

- Request: `<api version> get_serials <client fifo path>`
- Response: `<0 or 1, with 0 indicating an error and 1 indicating success> [serial]...`

### Example

- API version: 1
- Opened client FIFO path: `/tmp/tmp.bBcUh5obRK`
  - This FIFO must be opened prior to invoking the RPC

`1 get_serials /tmp/tmp.bBcUh5obRK`
