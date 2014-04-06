#ifndef LOGGING_H
#define LOGGING_H

#include <QDebug>

enum LogLevel {Info, Debug, Warn, Critical, Error, None};

static const LogLevel LOG_LEVEL = Info;

void LOG(LogLevel level, const char* message);
void LOG(LogLevel level, const char* message, float f);

#endif // LOGGING_H
