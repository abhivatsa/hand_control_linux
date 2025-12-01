#include "merai/RAII_SharedMemory.h"

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>     // shm_open, O_*
#include <sys/mman.h>  // mmap, munmap, PROT_*, MAP_SHARED
#include <unistd.h>    // close, ftruncate

namespace merai
{
    RAII_SharedMemory::RAII_SharedMemory(const std::string& name,
                                         std::size_t        size,
                                         bool               readOnly)
        : name_(name),
          size_(size),
          readOnly_(readOnly)
    {
        int openFlags = 0;
        int mmapProt  = 0;

        if (readOnly_)
        {
            openFlags = O_RDONLY;
            mmapProt  = PROT_READ;
        }
        else
        {
            openFlags = O_CREAT | O_RDWR;
            mmapProt  = PROT_READ | PROT_WRITE;
        }

        // 1) Open the shared memory object
        shmFd_ = ::shm_open(name_.c_str(), openFlags, 0666);
        if (shmFd_ < 0)
        {
            throw std::runtime_error(
                "RAII_SharedMemory: shm_open failed for " + name_ +
                ", errno=" + std::to_string(errno));
        }

        // 2) If read/write, size the object
        if (!readOnly_)
        {
            if (::ftruncate(shmFd_, static_cast<off_t>(size_)) < 0)
            {
                ::close(shmFd_);
                shmFd_ = -1;
                throw std::runtime_error(
                    "RAII_SharedMemory: ftruncate failed for " + name_ +
                    ", errno=" + std::to_string(errno));
            }
        }

        // 3) Map into our address space
        ptr_ = ::mmap(nullptr, size_, mmapProt, MAP_SHARED, shmFd_, 0);
        if (ptr_ == MAP_FAILED)
        {
            ::close(shmFd_);
            shmFd_ = -1;
            ptr_   = nullptr;
            throw std::runtime_error(
                "RAII_SharedMemory: mmap failed for " + name_ +
                ", errno=" + std::to_string(errno));
        }
    }

    RAII_SharedMemory::~RAII_SharedMemory() noexcept
    {
        if (ptr_ != nullptr)
        {
            ::munmap(ptr_, size_);
        }

        if (shmFd_ >= 0)
        {
            ::close(shmFd_);
        }
    }

} // namespace merai
