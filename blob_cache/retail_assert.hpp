/////////////////////////////////////////////
// Copyright (c) Gaia Platform LLC
// All rights reserved.
/////////////////////////////////////////////

#include <exception>
#include <sstream>

class retail_assertion_failure : public std::exception
{
protected:
    std::string m_message;

public:
    retail_assertion_failure() = default;

    explicit retail_assertion_failure(const std::string& message)
    {
        m_message = message;
    }

    const char* what() const noexcept override
    {
        return m_message.c_str();
    }
};

__attribute__((noreturn)) inline void throw_retail_assertion_failure(
    const char* message, const char* file, size_t line, const char* function)
{
    std::stringstream message_stream;
    message_stream << "Assertion failed in " << file << "::" << function << "(): line " << line << ": " << message;
    throw retail_assertion_failure(message_stream.str());
}

#ifdef DISABLE_RETAIL_ASSERT
#define RETAIL_ASSERT(c, m)
#else
#define RETAIL_ASSERT(c, m) \
    if (__builtin_expect(!static_cast<bool>(c), 0)) \
    { \
        throw_retail_assertion_failure(m, __FILE__, __LINE__, __func__); \
    }
#endif
