#pragma once

#include <string>
#include <atomic>
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
// ... or whichever headers hold these definitions
#include "merai/RAII_SharedMemory.h"

namespace hand_control
{
namespace network_api
{

class NetworkAPI
{
public:
    // Ctor, passing in names and sizes for all the shared memories
    NetworkAPI(const std::string &paramServerShmName,
               size_t paramServerShmSize,
               const std::string &rtDataShmName,
               size_t rtDataShmSize,
               const std::string &loggerShmName,
               size_t loggerShmSize);

    ~NetworkAPI();

    // Initialize network resources (ENet or raw UDP sockets, etc.)
    bool init();

    // Start the cyclic task (blocks until stopRequested_)
    void run();

    // Request to stop (thread-safe)
    void requestStop();

private:
    // The actual cyc. loop function. Called from run().
    void cyclicTask();

    // If you want to replicate your existing approach for timing:
    struct period_info
    {
        timespec next_period;
        long period_ns;
    };

    void periodic_task_init(period_info *pinfo, long periodNs);
    void inc_period(period_info *pinfo);
    void wait_rest_of_period(period_info *pinfo);

    // Shared memory RAII wrappers
    merai::RAII_SharedMemory paramServerShm_;
    merai::RAII_SharedMemory rtDataShm_;
    merai::RAII_SharedMemory loggerShm_;

    // Pointers after mapping
    const merai::ParameterServer* paramServerPtr_ = nullptr;
    merai::RTMemoryLayout* rtLayout_ = nullptr;
    merai::multi_ring_logger_memory* loggerMem_ = nullptr;

    std::atomic<bool> stopRequested_{false};

    // Possibly store your IP/port or ENet stuff here
    // e.g.: std::string ip_;
    //       unsigned short port_;
};

} // namespace network_api
} // namespace hand_control
