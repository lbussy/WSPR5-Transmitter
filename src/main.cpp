#include "wspr5_transmit.hpp"
#include <iostream>
#include <csignal>
#include <thread>
#include <atomic>

static std::atomic<bool> keep_running{true};

void signal_handler(int) {
    keep_running = false;
}

int main() {
    // Handle Ctrl-C to stop transmission cleanly
    std::signal(SIGINT, signal_handler);

    try {
        Wspr5Transmitter tx;

        // Callbacks to print start and end messages
        tx.setTransmissionCallbacks(
            [](const std::string &msg){ std::cout << msg << std::endl; },
            [](const std::string &msg){ std::cout << msg << std::endl; }
        );

        // Optional: set real-time scheduling policy and priority
        tx.setThreadScheduling(SCHED_FIFO, 10);

        // Optional: choose a specific GPIO for RF output (default is 4)
        tx.setOutputGPIO(4);

        // Configure a test tone at 7.040100 MHz, max power index, no PPM offset
        double frequency_hz = 7.040100e6;
        int power_index     = 7;      // 0-7 duty-cycle steps
        double ppm_correction = 0.0;  // no frequency offset
        int power_dbm       = 0;      // ignored for tone
        bool use_offset     = false;  // no random offset

        tx.setupTransmission(
            frequency_hz,
            power_index,
            ppm_correction,
            "",      // callsign (empty => test tone)
            "",      // grid_square
            power_dbm,
            use_offset
        );

        // Start the test tone (non-blocking)
        tx.enableTransmission();
        std::cout << "Test tone running at " << frequency_hz/1e6 << " MHz. Press Ctrl-C to stop." << std::endl;

        // Wait until signal (Ctrl-C) received
        while (keep_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop transmission
        tx.stopTransmission();
        std::cout << "Transmission stopped." << std::endl;

    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
