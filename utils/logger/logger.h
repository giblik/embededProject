#ifndef LOGGER_H__
#define LOGGER_H__

#include "error.h"

#ifndef LOG_LEVEL_DEF
#define LOG_LEVEL_DEF           LEVEL_ERROR
#endif /* LOG_LEVEL_DEF */

#define LOG_BUFFER_SIZE         (0xFF)

#define log(level, frmt, ...)                   \
    do                                          \
    {                                           \
        if (level <= LOG_LEVEL_DEF)             \
        {                                       \
            logger(level, frmt, ##__VA_ARGS__);        \
        }                                       \
    } while (0)

typedef enum
{
    LEVEL_ERROR,
    LEVEL_WARNING,
    LEVEL_INFO,
    LEVEL_DEBAG
} log_level_t;

typedef error_t (*logger_observer_t)(uint8_t *p_str, uint8_t len);
error_t logger(log_level_t level, const char * frmt, ...);
error_t logget_set_observer(logger_observer_t handler);


#endif /* LOGGER_H__ */
