#pragma once

#include <cstddef>   // size_t
#include <string>

namespace motion_control
{
    namespace merai
    {
        /**
         * @brief A simple RAII (Resource Acquisition Is Initialization) class
         *        to manage a POSIX shared memory object, with optional read-only mode.
         *
         * Usage:
         *    // Writer: create or open for read/write
         *    RAII_SharedMemory shm("/MyShmName", desiredSize, false); // readOnly=false
         *    void* ptr = shm.getPtr(); // Map into your data struct
         *
         *    // Reader: open in read-only mode
         *    RAII_SharedMemory shm("/MyShmName", desiredSize, true);  // readOnly=true
         *    const void* ptr = shm.getPtr(); // Only mapped with PROT_READ
         *
         *    // When shm goes out of scope, the destructor automatically unmaps
         *    // and closes the shared memory file descriptor.
         */
        class RAII_SharedMemory
        {
        public:
            /**
             * @brief Create or open a POSIX shared memory region with the specified name.
             *        If readOnly is false:
             *           - The region is resized to `size`. If it doesn't exist, it will be created.
             *           - Mapped read/write.
             *        If readOnly is true:
             *           - The region is opened O_RDONLY. No resizing is done (it must exist already).
             *           - Mapped read-only.
             *
             * @param name     - The unique name/key for the shared memory object (e.g. "/MyShmName").
             *                   Typically must start with a slash on many systems.
             * @param size     - The total byte size to allocate (if readOnly=false).
             *                   If readOnly=true, the existing shm should already be at least `size`.
             * @param readOnly - Whether to open in read-only mode (default = false).
             *
             * @throw std::runtime_error if shm_open, ftruncate, or mmap fails.
             */
            RAII_SharedMemory(const std::string& name, size_t size, bool readOnly = false);

            /**
             * @brief Destructor automatically unmaps and closes the shm file descriptor.
             *
             *        If you want to remove the shared memory object entirely (i.e., no other
             *        processes should attach), you could call shm_unlink(name_.c_str()) here.
             *        By default, we leave it so other processes can still open it if needed.
             */
            ~RAII_SharedMemory();

            // Disallow copying to avoid double close/unmap issues
            RAII_SharedMemory(const RAII_SharedMemory&) = delete;
            RAII_SharedMemory& operator=(const RAII_SharedMemory&) = delete;

            // Optionally allow move semantics if you need to transfer ownership
            RAII_SharedMemory(RAII_SharedMemory&&) = delete;
            RAII_SharedMemory& operator=(RAII_SharedMemory&&) = delete;

            /**
             * @return A raw pointer to the mapped shared memory region.
             *         If readOnly=true, you should treat it as const.
             */
            void* getPtr() const
            {
                return ptr_;
            }

            /**
             * @return The total size in bytes of this shared memory region.
             */
            size_t getSize() const
            {
                return size_;
            }

        private:
            std::string name_;
            size_t size_    = 0;
            int    shmFd_   = -1;
            void*  ptr_     = nullptr;
            bool   readOnly_ = false;
        };
    } // namespace merai
} // namespace motion_control
