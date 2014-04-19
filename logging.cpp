#include "stdafx.h"

#include "logging.h"

void LOG(LogLevel level, const char* message)
{
    if (level>=LOG_LEVEL){
        qDebug() << message;
    }
}

void LOG(LogLevel level, const char* message, int i)
{
    if (level>=LOG_LEVEL){
        qDebug() << message << i;
    }
}

void LOG(LogLevel level, const char* message, float f)
{
    if (level>=LOG_LEVEL){
        qDebug() << message << f;
    }
}

void LOG(LogLevel level, const char* message, double d)
{
    if (level>=LOG_LEVEL){
        qDebug() << message << d;
    }
}