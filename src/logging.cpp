#include "stdafx.h"

#include <iostream>

#include "logging.h"

QFile *file;
QTextStream *out;

#define LOG_FILE

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
#ifndef LOG_FILE
        qDebug() << qPrintable(getPrefix(level)) << message;
#else
        (*out) << qPrintable(getPrefix(level)) << message << endl;
#endif
	}
}

void LOG(LogLevel level, const char* message, int i)
{
	if (level>=LOG_LEVEL){
#ifndef LOG_FILE
        qDebug() << qPrintable(getPrefix(level)) << message << i;
#else
        (*out) << qPrintable(getPrefix(level)) << message << i << endl;
#endif
	}
}

void LOG(LogLevel level, const char* message, float f)
{
	if (level>=LOG_LEVEL){
#ifndef LOG_FILE
        qDebug() << qPrintable(getPrefix(level)) << message << f;
#else
        (*out) << qPrintable(getPrefix(level)) << message << f << endl;
#endif
	}
}

void LOG(LogLevel level, const char* message, double d)
{
	if (level>=LOG_LEVEL){
#ifndef LOG_FILE
        qDebug() << qPrintable(getPrefix(level)) << message << d;
#else
        (*out) << qPrintable(getPrefix(level)) << message << d << endl;
#endif
	}
}

void LOG(LogLevel level, const char* message1, int i1, const char* message2)
{
	if (level>=LOG_LEVEL){
#ifndef LOG_FILE
        qDebug() << qPrintable(getPrefix(level)) << message1 << i1 << message2;
#else
        (*out) << qPrintable(getPrefix(level)) << message1 << i1 << message2 << endl;
#endif
	}
}

void LOG(LogLevel level, const char* message1, int i1, const char* message2, int i2)
{
	if (level>=LOG_LEVEL){
#ifndef LOG_FILE
        qDebug() << qPrintable(getPrefix(level)) << message1 << i1 << message2 << i2;
#else
        (*out) << qPrintable(getPrefix(level)) << message1 << i1 << message2 << i2 << endl;
#endif
	}
}

void LOG(LogLevel level, const char* message1, int i1, const char* message2, int i2, const char* message3, int i3)
{
	if (level>=LOG_LEVEL){
#ifndef LOG_FILE
        qDebug() << qPrintable(getPrefix(level)) << message1 << i1 << message2 << i2 << message3 << i3;
#else
        (*out) << qPrintable(getPrefix(level)) << message1 << i1 << message2 << i2 << message3 << i3 << endl;
#endif
	}
}

void LOG(LogLevel level, const cv::Mat &m)
{
	if (level>=LOG_LEVEL){

#ifndef LOG_FILE
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
#else
		int rows = m.rows;
		int cols = m.cols;

		// QDebug inserts new line when it is destroyed
		for(int i=0; i<rows; i++){
			(*out) << qPrintable(getPrefix(level));
			for(int j=0; j<cols; j++){
				(*out) << m.at<double>(i,j) << " ";
			}
			(*out) << endl;
		}
#endif
	}
}

void LOG(LogLevel level, const cv::Mat_<double> &m)
{
	if (level>=LOG_LEVEL){
#ifndef LOG_FILE
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
#else
		int rows = m.rows;
		int cols = m.cols;

		// QDebug inserts new line when it is destroyed
		for(int i=0; i<rows; i++){
			(*out) << qPrintable(getPrefix(level));
			for(int j=0; j<cols; j++){
				(*out) << m.at<double>(i,j) << " ";
			}
			(*out) << endl;
		}
#endif
	}
}