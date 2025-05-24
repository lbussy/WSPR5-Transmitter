// wspr5_transmit.hpp
#ifndef WSPR5_TRANSMIT_HPP
#define WSPR5_TRANSMIT_HPP

#include <functional>
#include <string>
#include <mutex>
#include <pthread.h>            // scheduling policy constants
#include "wspr_message.hpp"

/**
 * @brief Singleton-style WSPR/RP1 transmitter over PCIe + DMA/PIO.
 * Allows one instance; configure then enableTransmission().
 */
class Wspr5Transmitter {
public:
    using Callback = std::function<void(const std::string &msg)>;

    Wspr5Transmitter();
    ~Wspr5Transmitter();

    Wspr5Transmitter(const Wspr5Transmitter&) = delete;
    Wspr5Transmitter& operator=(const Wspr5Transmitter&) = delete;
    Wspr5Transmitter(Wspr5Transmitter&&) = delete;
    Wspr5Transmitter& operator=(Wspr5Transmitter&&) = delete;

    void setTransmissionCallbacks(Callback on_start = {}, Callback on_end = {});
    void setupTransmission(
        double frequency,
        int    power,
        double ppm,
        std::string call_sign,
        std::string grid_square,
        int    power_dbm,
        bool   use_offset
    );
    void setThreadScheduling(int policy, int priority);
    void setOutputGPIO(int gpio);    ///< default: 4

    void enableTransmission();
    void disableTransmission();
    void stopTransmission();
    void shutdownTransmitter();

    bool isTransmitting() const noexcept;
    void printParameters() const;

private:
    static bool       instance_exists_;
    static std::mutex instance_mutex_;

    Callback on_start_;
    Callback on_end_;

    struct TransParams {
        double       frequency;
        int          power_index;
        double       ppm;
        std::string  call_sign;
        std::string  grid_square;
        int          power_dbm;
        bool         use_offset;
    } trans_params_;

    bool transmitting_;
    int thread_policy_{SCHED_OTHER};
    int thread_priority_{0};
    int output_gpio_{4};

    void*   reg_base_     = nullptr;
    void*   dma_buf_      = nullptr;
    size_t  dma_buf_size_ = 0;

    // DMA register offsets
    static constexpr size_t DMA_BASE_OFFSET    = 0x00188000;
    static constexpr size_t DMA_CHANNEL_STRIDE = 0x1000;
    static constexpr int    DMA_CHANNEL_INDEX  = 0;
    static constexpr size_t dmaSrcAddrOfst(int n) { return DMA_BASE_OFFSET + size_t(n)*DMA_CHANNEL_STRIDE + 0x00; }
    static constexpr size_t dmaLliAddrOfst(int n) { return DMA_BASE_OFFSET + size_t(n)*DMA_CHANNEL_STRIDE + 0x08; }
    static constexpr size_t dmaControlOfst(int n)  { return DMA_BASE_OFFSET + size_t(n)*DMA_CHANNEL_STRIDE + 0x0C; }
    static constexpr size_t dmaConfigOfst(int n)   { return DMA_BASE_OFFSET + size_t(n)*DMA_CHANNEL_STRIDE + 0x10; }

    // Pad control offsets
    static constexpr size_t PADS_BANK0_OFFSET = 0x000F0000;
    static constexpr size_t PADS_GPIO0_OFFSET = 0x00000100;
    static constexpr uint32_t DRIVE_MASK      = 0x3u << 4;
    static constexpr uint32_t DRIVE_SHIFT     = 4;

    void configureOutputPin();
    void setPadDriveStrength(int gpio, uint32_t strength);

    void runWsprTransmission();
    void runTestTone();
};

#endif // WSPR5_TRANSMIT_HPP
