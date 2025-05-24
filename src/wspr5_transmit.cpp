// wspr5_transmit.cpp
#include "wspr5_transmit.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <utility>
#include <pthread.h>
#include <cstring>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>

static constexpr const char* PCI_RESOURCE_PATH = "/sys/bus/pci/devices/0000:01:00.0/resource1";
static constexpr size_t REG_MAP_SIZE = 0x00200000;

// PIO0 register block offsets within BAR1 (RP1 datasheet)
static constexpr size_t PIO0_BASE_OFFSET       = 0x00050000;
static constexpr size_t PIO_INSTR_MEM_OFFSET   = PIO0_BASE_OFFSET + 0x0000;
static constexpr size_t PIO_SM0_OFFSET         = PIO0_BASE_OFFSET + 0x0100;
static constexpr size_t PIO_SM0_CLKDIV         = PIO_SM0_OFFSET   + 0x00;
static constexpr size_t PIO_SM0_EXECCTRL       = PIO_SM0_OFFSET   + 0x04;
static constexpr size_t PIO_SM0_SHIFTCTRL      = PIO_SM0_OFFSET   + 0x08;
static constexpr size_t PIO_SM0_FSTAT          = PIO_SM0_OFFSET   + 0x10;
static constexpr size_t PIO_SM0_TXF            = PIO_SM0_OFFSET   + 0x14;

static int dma_heap_fd = -1;
static int dma_buf_fd  = -1;

static void* allocateDmaBuffer(size_t size, uint64_t &phys_addr) {
    if (dma_heap_fd < 0) {
        dma_heap_fd = open("/dev/dma_heap/system", O_RDONLY | O_CLOEXEC);
        if (dma_heap_fd < 0) {
            perror("open /dev/dma_heap/system");
            return nullptr;
        }
    }
    struct dma_heap_allocation_data req{};
    req.len = size;
    req.fd_flags = O_RDWR | O_CLOEXEC;
    if (ioctl(dma_heap_fd, DMA_HEAP_IOCTL_ALLOC, &req) < 0) {
        perror("DMA_HEAP_IOCTL_ALLOC");
        return nullptr;
    }
    dma_buf_fd = req.fd;

    void* vaddr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, dma_buf_fd, 0);
    if (vaddr == MAP_FAILED) {
        perror("mmap dma buffer");
        close(dma_buf_fd);
        dma_buf_fd = -1;
        return nullptr;
    }

    // Resolve physical address via /proc/self/pagemap
    int pagemap = open("/proc/self/pagemap", O_RDONLY);
    if (pagemap < 0) {
        perror("open /proc/self/pagemap");
        munmap(vaddr, size);
        close(dma_buf_fd);
        dma_buf_fd = -1;
        return nullptr;
    }
    uint64_t page_size = sysconf(_SC_PAGESIZE);
    uint64_t vpn       = uintptr_t(vaddr) / page_size;
    uint64_t offset    = uintptr_t(vaddr) % page_size;
    uint64_t entry;
    off_t    pos       = off_t(vpn) * off_t(sizeof(entry));
    if (lseek(pagemap, pos, SEEK_SET) != pos ||
        read(pagemap, &entry, sizeof(entry)) != sizeof(entry)) {
        perror("read pagemap");
        close(pagemap);
        munmap(vaddr, size);
        close(dma_buf_fd);
        dma_buf_fd = -1;
        return nullptr;
    }
    close(pagemap);

    // Ensure page is present
    if (!(entry & (1ULL << 63))) {
        fprintf(stderr, "allocateDmaBuffer: page not present in RAM");
        munmap(vaddr, size);
        close(dma_buf_fd);
        dma_buf_fd = -1;
        return nullptr;
    }

    const uint64_t PFN_MASK = (1ULL << 55) - 1;
    uint64_t pfn = entry & PFN_MASK;
    phys_addr = (pfn * page_size) + offset;
    return vaddr;
}

static void freeDmaBuffer(void* buf, size_t size) {
    if (buf) munmap(buf,size);
    if (dma_buf_fd>=0) { close(dma_buf_fd); dma_buf_fd=-1; }
}

bool Wspr5Transmitter::instance_exists_ = false;
std::mutex Wspr5Transmitter::instance_mutex_;

Wspr5Transmitter::Wspr5Transmitter() {
    std::lock_guard<std::mutex> lk(instance_mutex_);
    if (instance_exists_) throw std::runtime_error("only one instance allowed");
    instance_exists_ = true;
}

Wspr5Transmitter::~Wspr5Transmitter() {
    shutdownTransmitter();
    std::lock_guard<std::mutex> lk(instance_mutex_);
    instance_exists_ = false;
}

void Wspr5Transmitter::setTransmissionCallbacks(Callback on_start, Callback on_end) {
    on_start_ = std::move(on_start);
    on_end_   = std::move(on_end);
}

void Wspr5Transmitter::setupTransmission(double frequency,int power,double ppm,std::string call_sign,std::string grid_square,int power_dbm,bool use_offset) {
    trans_params_.frequency=frequency; trans_params_.power_index=power;
    trans_params_.ppm=ppm; trans_params_.call_sign=std::move(call_sign);
    trans_params_.grid_square=std::move(grid_square); trans_params_.power_dbm=power_dbm;
    trans_params_.use_offset=use_offset;
}

void Wspr5Transmitter::setThreadScheduling(int policy,int priority){
    thread_policy_=policy; thread_priority_=priority;
}

void Wspr5Transmitter::setOutputGPIO(int gpio) {
    if (gpio < 0 || gpio > 27) {
        throw std::out_of_range("GPIO out of range");
    }
    output_gpio_ = gpio;
}

void Wspr5Transmitter::configureOutputPin() {
    int idx=output_gpio_/10; int shift=(output_gpio_%10)*3;
    auto regs=reinterpret_cast<volatile uint32_t*>(reg_base_);
    uint32_t v=regs[idx]; v&=~(0x7u<<shift); v|=(5u<<shift);
    regs[idx]=v;
}

void Wspr5Transmitter::enableTransmission() {
    disableTransmission();
    std::thread([this]{
        int fd=open(PCI_RESOURCE_PATH,O_RDWR|O_SYNC);
        if(fd<0) throw std::runtime_error("open resource");
        reg_base_=mmap(nullptr,REG_MAP_SIZE,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
        close(fd); if(reg_base_==MAP_FAILED) throw std::runtime_error("mmap");
        configureOutputPin();
        uint32_t str=(trans_params_.power_index*3)/7;
        setPadDriveStrength(output_gpio_,str);
        pthread_t self=pthread_self(); sched_param sch{}; sch.sched_priority=thread_priority_;
        pthread_setschedparam(self,thread_policy_,&sch);
        if(trans_params_.call_sign.empty()){
            if (on_start_) {
                        on_start_("Test tone started");
                    }
                    runTestTone();
                    if (on_end_) {
                        on_end_("Test tone stopped");
                    }
        } else {
            if (on_start_) {
                        on_start_("WSPR TX started");
                    }
                    runWsprTransmission();
                    if (on_end_) {
                        on_end_("WSPR TX completed");
                    }
        }
    }).detach();
}

void Wspr5Transmitter::disableTransmission() {
    // Stop DMA engine if running
    if (reg_base_) {
        auto regs = reinterpret_cast<volatile uint32_t*>(reg_base_);
        regs[dmaConfigOfst(DMA_CHANNEL_INDEX)/4] = 0u;
    }
    // Free DMA buffer if allocated
    if (dma_buf_) {
        freeDmaBuffer(dma_buf_, dma_buf_size_);
        dma_buf_ = nullptr;
        dma_buf_size_ = 0;
    }
    // Mark as not transmitting
    transmitting_ = false;
}
void Wspr5Transmitter::stopTransmission() {
    // Request immediate stop of any running transmission
    transmitting_ = false;
    if (reg_base_) {
        // Stop DMA engine
        auto regs = reinterpret_cast<volatile uint32_t*>(reg_base_);
        regs[dmaConfigOfst(DMA_CHANNEL_INDEX)/4] = 0u;
    }
    if (dma_buf_) {
        // Free only the DMA buffer, keep register mapping for new starts
        freeDmaBuffer(dma_buf_, dma_buf_size_);
        dma_buf_ = nullptr;
        dma_buf_size_ = 0;
    }
}

void Wspr5Transmitter::shutdownTransmitter(){
    disableTransmission();
    if(reg_base_){ auto regs=reinterpret_cast<volatile uint32_t*>(reg_base_);
        regs[dmaConfigOfst(DMA_CHANNEL_INDEX)/4]=0; munmap(reg_base_,REG_MAP_SIZE); reg_base_=nullptr; }
    if(dma_buf_){ freeDmaBuffer(dma_buf_,dma_buf_size_); dma_buf_=nullptr; dma_buf_size_=0; }
}

bool Wspr5Transmitter::isTransmitting()const noexcept{return transmitting_;}

void Wspr5Transmitter::printParameters()const{
    std::cout<<"Freq: "<<trans_params_.frequency<<" Hz\n"
             <<"Power idx: "<<trans_params_.power_index<<"\n"
             <<"PPM: "<<trans_params_.ppm<<"\n"
             <<"Power dBm: "<<trans_params_.power_dbm<<"\n"
             <<"Call: "<<trans_params_.call_sign<<"\n"
             <<"Grid: "<<trans_params_.grid_square<<"\n"
             <<"Offset: "<<(trans_params_.use_offset?"yes":"no")<<"\n";
}

void Wspr5Transmitter::runWsprTransmission(){
    transmitting_=true; std::this_thread::sleep_for(std::chrono::seconds(111)); transmitting_=false;
}

void Wspr5Transmitter::runTestTone(){
    transmitting_=true;
    // 1) Tone frequency + PPM
    const uint32_t sr=12000; double hz=trans_params_.frequency;
    if(trans_params_.use_offset) hz*=(1+trans_params_.ppm*1e-6);
    size_t samples=sr/hz; size_t hi=(trans_params_.power_index*samples)/7;
    std::vector<uint32_t> buf(samples);
    for(size_t i=0;i<samples;++i) buf[i]=(i<hi?0xFFFFFFFFu:0u);
    // 2) DMA buffer
    dma_buf_size_=buf.size()*sizeof(uint32_t);
    uint64_t phys;
    dma_buf_=allocateDmaBuffer(dma_buf_size_,phys);
    if(!dma_buf_){ transmitting_=false; return; }
    memcpy(dma_buf_,buf.data(),dma_buf_size_);
    auto regs=reinterpret_cast<volatile uint32_t*>(reg_base_);
    regs[dmaSrcAddrOfst(DMA_CHANNEL_INDEX)/4]=uint32_t(phys);
    regs[(dmaSrcAddrOfst(DMA_CHANNEL_INDEX)+4)/4]=uint32_t(phys>>32);
    regs[dmaLliAddrOfst(DMA_CHANNEL_INDEX)/4]=0;
    uint32_t ctrl=(uint32_t)samples&0xFFFu; ctrl |= (1u << 26); // SRC_INC
    ctrl |= (2u << 18); // SWIDTH = 2 → 32-bit
    ctrl |= (2u << 21); // DWIDTH = 2 → 32-bit
    regs[dmaControlOfst(DMA_CHANNEL_INDEX)/4]=ctrl;
    regs[dmaConfigOfst(DMA_CHANNEL_INDEX)/4]=1u;
    // 3) wait until stopped
    while(transmitting_) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // 4) stop DMA
    regs[dmaConfigOfst(DMA_CHANNEL_INDEX)/4]=0u;
}

void Wspr5Transmitter::setPadDriveStrength(int gpio,uint32_t strength){
    auto pads=reinterpret_cast<volatile uint32_t*>((uint8_t*)reg_base_+PADS_BANK0_OFFSET);
    size_t idx=(PADS_GPIO0_OFFSET>>2)+gpio; uint32_t v=pads[idx];
    v=(v&~DRIVE_MASK)|((strength<<DRIVE_SHIFT)&DRIVE_MASK);
    pads[idx]=v;
}
