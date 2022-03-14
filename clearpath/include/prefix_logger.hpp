//
// Created by simone on 2/11/22.
//

#pragma once

#include <string>

#include <gaia/logger.hpp>

namespace gaia::clearpath
{

class prefix_logger_t
{
public:
    prefix_logger_t(std::string prefix)
        : m_prefix(std::move(prefix)){};

    void log(const std::string& message)
    {
        gaia_log::app().info("{} {}", m_prefix, message);
    }

private:
    std::string m_prefix;
};

} // namespace gaia::clearpath
