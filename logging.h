#ifndef LOGGING_H
#define LOGGING_H

#include <QDebug>

enum LogLevel {Debug, Warn, Critical, Error, None};

static const LogLevel LOG_LEVEL = Debug;

void LOG(LogLevel level, const char* message);

#endif // LOGGING_H
