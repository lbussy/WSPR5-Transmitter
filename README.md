# WSPR5-Transmitter

:no_entry_sign: _**This does not work, it may never, and it may brick your system if you have an SSD.  Unless you are a bit-banger and a developer, please do not poke at this.  If you ARE an expert in RPi 5 DMA, I could use help.**_ :no_entry_sign:

A singleton C++ class for generating WSPR-encoded or test-tone RF signals using the Raspberry Pi 5 (RP1) PCIe interface with DMA and PIO.

## Features

* **Singleton**: Only one instance allowed to prevent resource conflicts.
* **Test Tone**: Generate a continuous square‑wave test tone at any frequency with PPM correction and duty‑cycle power control.
* **WSPR Mode**: (TODO) Transmit WSPR messages with correct timing and symbol encoding.
* **Thread Scheduling**: Configure Linux real-time scheduling policies for predictability.
* **GPIO Selection**: Choose any BCM 0–27 pin as RF output (defaults to GPIO 4).
* **DMA/PIO Pipeline**: Manual BAR1 register mapping, CMA allocator for physically contiguous buffers, DMA setup, and PIO state machine.

## Prerequisites

* **Hardware**: Raspberry Pi 5 with RP1 PCIe enabled.
* **Kernel**: CMA and DMA-heap support (`/dev/dma_heap/system`).
* **Build Tools**: `g++` (C++17), CMake (optional), `make`.
* **Permissions**: Root or appropriate udev rules to map `/sys/bus/pci/.../resource1` and open DMA-heap.

## Building

```bash
# From project root (assuming Makefile exists)
make debug
```

Or compile manually:

```bash
g++ -std=c++17 -pthread \
    src/wspr5_transmit.cpp \
    src/wspr_message.cpp \
    test/main.cpp \
    -o test_tone
```

## Usage

```bash
sudo ./test_tone
```

Press **Ctrl-C** to stop the tone.

### Example in `main.cpp`

```cpp
Wspr5Transmitter tx;
tx.setTransmissionCallbacks(
  [](auto&m){std::cout<<m<<"\n";},
  [](auto&m){std::cout<<m<<"\n";}
);
tx.setThreadScheduling(SCHED_FIFO,10);
tx.setOutputGPIO(4);
tx.setupTransmission(
  7.040100e6,  // Frequency Hz
  7,           // Power index (0–7)
  0.0,         // PPM correction
  "", "",   // callsign, grid (empty = test tone)
  0,           // power_dBm (ignored for tone)
  false        // no random offset
);
tx.enableTransmission();

// ... wait, then:
tx.stopTransmission();
```

## API Reference

* **Constructor / Destructor**: Manage singleton instantiation and cleanup.
* **setTransmissionCallbacks(on\_start, on\_end)**: Register callbacks for TX events.
* **setupTransmission(frequency, power\_index, ppm, call\_sign, grid, power\_dbm, use\_offset)**: Configure parameters.
* **setThreadScheduling(policy, priority)**: Set POSIX thread scheduling.
* **setOutputGPIO(gpio)**: Select BCM 0–27 pin for output.
* **enableTransmission()**: Start RX thread (non-blocking).
* **disableTransmission()**: Stop and free DMA resources.
* **stopTransmission()**: Immediately halt current TX but keep registers mapped.
* **shutdownTransmitter()**: Full resource teardown.
* **isTransmitting()**: Query TX state.
* **printParameters()**: Dump current configuration.

## Contributing

1. Fork the repository.
2. Create a feature branch.
3. Submit a pull request.

## License

MIT © Lee C. Bussy
