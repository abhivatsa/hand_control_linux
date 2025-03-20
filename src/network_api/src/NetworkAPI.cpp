#include "network_api/NetworkAPI.h"
#include <stdexcept>
#include <iostream>
#include <ctime>
// Add your networking includes (e.g. enet, sockets, etc.)

namespace hand_control
{
namespace network_api
{

//-----------------------------------------------------
// Constructor: Map shared memory segments, store pointers
//-----------------------------------------------------
NetworkAPI::NetworkAPI(const std::string &paramServerShmName,
                       size_t paramServerShmSize,
                       const std::string &rtDataShmName,
                       size_t rtDataShmSize,
                       const std::string &loggerShmName,
                       size_t loggerShmSize)
    : paramServerShm_(paramServerShmName, paramServerShmSize, true),
      rtDataShm_(rtDataShmName, rtDataShmSize, false),
      loggerShm_(loggerShmName, loggerShmSize, false)
{
    // Map the ParameterServer
    paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(paramServerShm_.getPtr());
    if (!paramServerPtr_)
    {
        throw std::runtime_error("Failed to map ParameterServer memory.");
    }

    // Map the RTMemoryLayout
    rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
    if (!rtLayout_)
    {
        throw std::runtime_error("Failed to map RTMemoryLayout memory.");
    }

    // Map the Logger memory
    loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(loggerShm_.getPtr());
    if (!loggerMem_)
    {
        throw std::runtime_error("Failed to map multi_ring_logger_memory.");
    }

    // Potentially read IP/Port from the parameter server
    // e.g.: ip_ = paramServerPtr_->network.ip;  
    //       port_ = paramServerPtr_->network.port; 
}

NetworkAPI::~NetworkAPI()
{
    // Cleanup if needed
}

//-----------------------------------------------------
// init(): set up networking resources
//-----------------------------------------------------
bool NetworkAPI::init()
{
    // If using ENet: enet_initialize() ...
    // If using raw sockets: create socket, bind, etc.

    std::cout << "[NetworkAPI] init done.\n";
    return true;
}

//-----------------------------------------------------
// run(): the main cyc. loop
//-----------------------------------------------------
void NetworkAPI::run()
{
    // Use the same approach as in your Control module
    period_info pinfo;
    periodic_task_init(&pinfo, 1'000'000L); // 1 ms for 1000Hz

    while (!stopRequested_.load(std::memory_order_relaxed))
    {
        cyclicTask();
        wait_rest_of_period(&pinfo);
    }
}

//-----------------------------------------------------
// requestStop(): signaled from outside
//-----------------------------------------------------
void NetworkAPI::requestStop()
{
    stopRequested_.store(true, std::memory_order_relaxed);
}

//-----------------------------------------------------
// The actual cyc. logic at 1 kHz
//-----------------------------------------------------
void NetworkAPI::cyclicTask()
{
    // 1) Read the data from shared memory (e.g. device state)
    // e.g. read out joint positions, forces, etc.
    // or read commands that other modules wrote into shared memory.

    // 2) Transmit relevant data via UDP/ENet
    // e.g. some kind of haptic_data_packet, serializing it out

    // 3) Receive any data from the network, parse commands
    // e.g. if (receivedCommand) { write it to rtLayout_ or wherever it needs to go }

    // 4) Possibly log to loggerMem_, if your system uses multi_ring_logger
    // e.g. loggerMem_->push("NetworkAPI: Sent data...");

    // 5) (No extra sleep here, because we do wait_rest_of_period at the bottom of run())
}

//-----------------------------------------------------
// Periodic helpers: same as in your Control.cpp
//-----------------------------------------------------
void NetworkAPI::periodic_task_init(period_info *pinfo, long periodNs)
{
    clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
    pinfo->period_ns = periodNs;
}

void NetworkAPI::inc_period(period_info *pinfo)
{
    pinfo->next_period.tv_nsec += pinfo->period_ns;
    if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
    {
        pinfo->next_period.tv_nsec -= 1'000'000'000L;
        pinfo->next_period.tv_sec++;
    }
}

void NetworkAPI::wait_rest_of_period(period_info *pinfo)
{
    inc_period(pinfo);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
}

} // namespace network_api
} // namespace hand_control
