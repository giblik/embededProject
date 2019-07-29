#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "logger.h"
#include "error.h"

#define LEVEL_DESC_SIZE                 (4)

static logger_observer_t m_observers;

static const char * log_level_to_str(log_level_t level)
{
    switch(level)
    {
        case LEVEL_ERROR:   return "[E]";
        case LEVEL_WARNING: return "[W]";
        case LEVEL_INFO:    return "[I]";
        case LEVEL_DEBAG:   return "[D]";
    }
    return "[U]";
}

error_t logger(log_level_t level, const char * frmt, ...)
{
    char log_buff[LOG_BUFFER_SIZE];

    sprintf(log_buff, "%s", log_level_to_str(level));

    va_list valist;
    va_start(valist, frmt);
    int len =  vsnprintf(log_buff + LEVEL_DESC_SIZE,
                            LOG_BUFFER_SIZE - LEVEL_DESC_SIZE, frmt, valist);
    va_end(valist);

    if (len < 0)
    {
        return ERROR_INTERNAL;
    }

    if (m_observers == NULL)
    {
        return ERROR_NOT_FOUND;
    }

    m_observers((uint8_t*)log_buff, (uint8_t)len + LEVEL_DESC_SIZE);

    return ERROR_SUCCESS;
}

error_t logget_set_observer(logger_observer_t handler)
{
    if (handler == NULL)
    {
        return ERROR_NULL_PTR;
    }

    if (m_observers == NULL)
    {
        m_observers = handler;
        return ERROR_SUCCESS;
    }

    return ERROR_NO_MEM;
}

