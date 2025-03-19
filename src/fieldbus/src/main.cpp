#include <iostream>
#include <stdexcept>
#include <cstdlib>  // for EXIT_SUCCESS, EXIT_FAILURE

#include "fieldbus/EthercatMaster.h"

int main(int argc, char* argv[])
{
    try
    {
        // Hard-coded shared memory names and sizes
        std::string paramServerShmName = "/ParameterServerShm";
        size_t paramServerShmSize      = sizeof(hand_control::merai::ParameterServer);

        std::string rtDataShmName = "/RTDataShm";
        size_t rtDataShmSize      = sizeof(hand_control::merai::RTMemoryLayout);

        std::string loggerShmName = "/LoggerShm";
        size_t loggerShmSize      = sizeof(hand_control::merai::multi_ring_logger_memory);

        // Logging pointer
        hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

        // Create EthercatMaster (attaches to the three SHMs)
        //std::cout<<"22"<<std::endl;
        hand_control::fieldbus::EthercatMaster master(
            paramServerShmName, paramServerShmSize,
            rtDataShmName,      rtDataShmSize,
            loggerShmName,      loggerShmSize
        );
        //std::cout<<"28"<<std::endl;
        // Initialize the master (configure drives, domain, etc.)
        if (!master.initializeMaster())
        {
            // std::cerr << "[Error] EthercatMaster initialization failed!\n";
            log_error( loggerMem_, "Main", 3400, "EthercatMaster initialization failed!" );
            return EXIT_FAILURE;
        }


        //std::cout<<"35"<<std::endl;
        // std::cout << "[Info] Starting EtherCAT cyclic task...\n";
        log_info( loggerMem_, "Main", 3200, "[Info] Starting EtherCAT cyclic task..." );
        master.run();  // Blocks until stop() is called or loop ends

        
        // std::cout << "[Info] EtherCAT Master exiting normally.\n";
        log_info( loggerMem_, "Main", 3201, "[Info] EtherCAT Master exiting normally" );
        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        // std::cerr << "[Exception] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
