#include <iostream>
#include <thread>
#include <chrono>

#include "logic/SystemOrchestrator.h"
#include "logic/DriveManager.h"
#include "logic/SafetyManager.h"
#include "logic/ErrorManager.h"
#include "logic/ControllerModeManager.h"

int main(int argc, char** argv)
{
    // Instantiate managers (all in motion_control::logic)
    motion_control::logic::DriveManager        driveManager;
    motion_control::logic::SafetyManager       safetyManager;
    motion_control::logic::ErrorManager        errorManager;
    motion_control::logic::ControllerModeManager ctrlModeManager;

    // Create orchestrator
    motion_control::logic::SystemOrchestrator orchestrator(
        driveManager,
        safetyManager,
        errorManager,
        ctrlModeManager
    );

    // Load logic config
    motion_control::logic::LogicConfig cfg;
    cfg.updateRateHz = 1000.0;

    // Initialize orchestrator
    if (!orchestrator.init(cfg))
    {
        std::cerr << "Failed to initialize SystemOrchestrator.\n";
        return -1;
    }

    // Example loop (non-real-time for demonstration)
    while (true)
    {
        orchestrator.update();

        // Sleep or wait for ~1ms
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (orchestrator.getSystemState() == motion_control::logic::SystemState::ERROR)
        {
            std::cerr << "System is in ERROR state. Stopping...\n";
            break;
        }
    }

    return 0;
}
