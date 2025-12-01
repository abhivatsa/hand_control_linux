#pragma once

#include <cstddef>  // std::size_t
#include <string>

namespace merai
{
    /**
     * @brief RAII wrapper around a POSIX shared memory object.
     *
     *  - If readOnly == false:
     *      * shm_open(O_CREAT|O_RDWR)
     *      * ftruncate to requested size
     *      * mmap PROT_READ | PROT_WRITE
     *
     *  - If readOnly == true:
     *      * shm_open(O_RDONLY)
     *      * mmap PROT_READ
     *
     * The underlying object is not unlinked here; the launcher is responsible
     * for creating/cleaning named SHM segments.
     */
    class RAII_SharedMemory
    {
    public:
        explicit RAII_SharedMemory(const std::string& name,
                                   std::size_t size,
                                   bool readOnly = false);

        ~RAII_SharedMemory() noexcept;

        RAII_SharedMemory(const RAII_SharedMemory&)            = delete;
        RAII_SharedMemory& operator=(const RAII_SharedMemory&) = delete;
        RAII_SharedMemory(RAII_SharedMemory&&)                 = delete;
        RAII_SharedMemory& operator=(RAII_SharedMemory&&)      = delete;

        /// @return Raw pointer to mapped region (treat as const if readOnly).
        void*       getPtr()  const noexcept { return ptr_;  }

        /// @return Size in bytes of the mapping.
        std::size_t getSize() const noexcept { return size_; }

    private:
        std::string name_;
        std::size_t size_     = 0;
        int         shmFd_    = -1;
        void*       ptr_      = nullptr;
        bool        readOnly_ = false;
    };

} // namespace merai
