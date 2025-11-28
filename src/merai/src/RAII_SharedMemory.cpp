#include "merai/RAII_SharedMemory.h"

#include <stdexcept>   // std::runtime_error
#include <fcntl.h>     // O_CREAT, O_RDWR, O_RDONLY
#include <sys/mman.h>  // mmap, munmap, PROT_READ, PROT_WRITE, MAP_SHARED
#include <unistd.h>    // close, ftruncate
#include <cerrno>      // errno
#include <cstring>     // strerror

    namespace merai
    {
        RAII_SharedMemory::RAII_SharedMemory(const std::string& name,
                                             size_t size,
                                             bool readOnly)
            : name_(name),
              size_(size),
              readOnly_(readOnly)
        {
            int openFlags = 0;
            int mmapProt  = 0;

            if (readOnly_)
            {
                // Open existing shm in read-only mode (it must already exist!)
                openFlags = O_RDONLY;
                mmapProt  = PROT_READ;
            }
            else
            {
                // Create or open in read/write mode
                openFlags = O_CREAT | O_RDWR;
                mmapProt  = PROT_READ | PROT_WRITE;
            }

            // 1) Open the shared memory object
            shmFd_ = shm_open(name_.c_str(), openFlags, 0666);
            if (shmFd_ < 0)
            {
                throw std::runtime_error(
                    "RAII_SharedMemory: shm_open failed for " + name_ +
                    ", errno=" + std::to_string(errno));
            }

            // 2) If we are read/write, resize (truncate) the shm object to 'size_'
            if (!readOnly_)
            {
                if (ftruncate(shmFd_, size_) < 0)
                {
                    ::close(shmFd_);
                    throw std::runtime_error(
                        "RAII_SharedMemory: ftruncate failed for " + name_ +
                        ", errno=" + std::to_string(errno));
                }
            }

            // 3) Map into our address space
            ptr_ = mmap(nullptr, size_, mmapProt, MAP_SHARED, shmFd_, 0);
            if (ptr_ == MAP_FAILED)
            {
                ::close(shmFd_);
                throw std::runtime_error(
                    "RAII_SharedMemory: mmap failed for " + name_ +
                    ", errno=" + std::to_string(errno));
            }
        }

        RAII_SharedMemory::~RAII_SharedMemory()
        {
            // Unmap the region if valid
            if (ptr_ && ptr_ != MAP_FAILED)
            {
                munmap(ptr_, size_);
            }

            // Close the file descriptor if valid
            if (shmFd_ >= 0)
            {
                ::close(shmFd_);
            }

            // Optional: remove the shared memory object (unlink) if you want
            // to ensure no other process can open it after this. Typically
            // you only do this in the "owner" process that created the memory.
            // Example:
            // if (!readOnly_) {
            //     shm_unlink(name_.c_str());
            // }
        }
    } // namespace merai
