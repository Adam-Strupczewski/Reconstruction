#include "stdafx.h"

#include <iostream>

#include "logging.h"

// qPrintable prevents quotation marks from being printed
// getPrefix structures the logs
QString getPrefix(LogLevel level){
	if (level == Info)
		return "   ";
	else
		return "";
}

void LOG(LogLevel level, const char* message)
{
    if (level>=LOG_LEVEL){
        qDebug() << qPrintable(getPrefix(level)) << message;
    }
}

void LOG(LogLevel level, const char* message, int i)
{
    if (level>=LOG_LEVEL){
        qDebug() << qPrintable(getPrefix(level)) << message << i;
    }
}

void LOG(LogLevel level, const char* message, float f)
{
    if (level>=LOG_LEVEL){
        qDebug() << qPrintable(getPrefix(level)) << message << f;
    }
}

void LOG(LogLevel level, const char* message, double d)
{
    if (level>=LOG_LEVEL){
        qDebug() << qPrintable(getPrefix(level)) << message << d;
    }
}

void LOG(LogLevel level, const char* message1, int i1, const char* message2)
{
    if (level>=LOG_LEVEL){
        qDebug() << qPrintable(getPrefix(level)) << message1 << i1 << message2;
    }
}

void LOG(LogLevel level, const char* message1, int i1, const char* message2, int i2)
{
    if (level>=LOG_LEVEL){
        qDebug() << qPrintable(getPrefix(level)) << message1 << i1 << message2 << i2;
    }
}

void LOG(LogLevel level, const char* message1, int i1, const char* message2, int i2, const char* message3, int i3)
{
    if (level>=LOG_LEVEL){
        qDebug() << qPrintable(getPrefix(level)) << message1 << i1 << message2 << i2 << message3 << i3;
    }
}

void LOG(LogLevel level, const cv::Mat &m)
{
    if (level>=LOG_LEVEL){

		int rows = m.rows;
		int cols = m.cols;

		// QDebug inserts new line when it is destroyed
		for(int i=0; i<rows; i++){
			QDebug debug = qDebug();
			debug << qPrintable(getPrefix(level));
			for(int j=0; j<cols; j++){
				debug << m.at<double>(i,j) << " ";
			}
		}
    }
}

void LOG(LogLevel level, const cv::Mat_<double> &m)
{
    if (level>=LOG_LEVEL){

		int rows = m.rows;
		int cols = m.cols;

		// QDebug inserts new line when it is destroyed
		for(int i=0; i<rows; i++){
			QDebug debug = qDebug();
			debug << qPrintable(getPrefix(level));
			for(int j=0; j<cols; j++){
				debug << m.at<double>(i,j) << " ";
			}
		}
    }
}