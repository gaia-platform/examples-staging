/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#pragma once

#include <string>

#include "gaia_access_control.h"

namespace gaia
{
namespace access_control
{
namespace communication
{

typedef void (*message_callback_t)(const std::string& topic, const std::string& payload);

std::string get_uuid();

bool init(int argc, char* argv[]);
void connect(message_callback_t callback, const std::string& init_msg);
void publish_message(const std::string& topic, const std::string& payload);

} // namespace communication
} // namespace access_control
} // namespace gaia
