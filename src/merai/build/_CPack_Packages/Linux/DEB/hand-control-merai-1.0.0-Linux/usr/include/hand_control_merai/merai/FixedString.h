#pragma once

#include <algorithm>  // std::min
#include <cstring>    // std::memcpy
#include <string>     // std::string

namespace hand_control
{
    namespace merai
    {
        /**
         * @brief A simple fixed-size string wrapper to avoid dynamic allocation.
         * @tparam N The total capacity including space for the null terminator.
         */
        template <size_t N>
        struct FixedString
        {
            char data[N];  // N includes space for the null terminator

            FixedString()
            {
                clear();
            }

            /**
             * @brief Clears the buffer and sets data[0] = '\0'.
             *        (Optional) Could zero-fill the whole array if needed.
             */
            void clear()
            {
                data[0] = '\0';
            }

            /**
             * @brief Sets the value from a std::string, truncating if too long.
             * @param s Source std::string to copy from.
             */
            void set(const std::string& s)
            {
                auto len = std::min(s.size(), static_cast<size_t>(N - 1));
                std::memcpy(data, s.data(), len);
                data[len] = '\0';
            }

            /**
             * @brief Returns a C-string pointer.
             */
            const char* c_str() const
            {
                return data;
            }
        };
    } // namespace merai
} // namespace hand_control
