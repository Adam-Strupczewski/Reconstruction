#ifndef LOGGING_H
#define LOGGING_H

#include <QDebug>
#include <QFile>
#include <QTextStream>

#include <opencv2/core/core.hpp>

enum LogLevel {Info, Debug, Warn, Critical, Error, None};

static const LogLevel LOG_LEVEL = Info;

QString getPrefix(LogLevel level);

void LOG(LogLevel level, const char* message);
void LOG(LogLevel level, const char* message, int i);
void LOG(LogLevel level, const char* message, float f);
void LOG(LogLevel level, const char* message, double d);

void LOG(LogLevel level, const char* message1, int i1, const char* message2);
void LOG(LogLevel level, const char* message1, int i1, const char* message2, int i2);
void LOG(LogLevel level, const char* message1, int i1, const char* message2, int i2, const char* message3, int i3);

void LOG(LogLevel level, const cv::Mat &m);
void LOG(LogLevel level, const cv::Mat_<double> &m);

#endif // LOGGING_H
