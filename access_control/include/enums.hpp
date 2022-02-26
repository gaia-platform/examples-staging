#pragma once

#include <cstdint>
#include <string>

namespace enums
{
    namespace scan_table
    {
        enum e_scan_type: std::uint8_t
        {
            badge,
            vehicle_entering,
            vehicle_departing,
            joining_wifi,
            leaving_wifi,
            face,
            leaving
        };
    }
} // namespace enums