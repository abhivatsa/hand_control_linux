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
    // Instantiate managers (all in hand_control::logic)
    hand_control::logic::DriveManager        driveManager;
    hand_control::logic::SafetyManager       safetyManager;
    hand_control::logic::ErrorManager        errorManager;
    hand_control::logic::ControllerModeManager ctrlModeManager;

    // Create orchestrator
    hand_control::logic::SystemOrchestrator orchestrator(
        driveManager,
        safetyManager,
        errorManager,
        ctrlModeManager
    );

    // Load logic config
    hand_control::logic::LogicConfig cfg;
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

        if (orchestrator.getSystemState() == hand_control::logic::SystemState::ERROR)
        {
            std::cerr << "System is in ERROR state. Stopping...\n";
            break;
        }
    }

    return 0;
}
