#include "stdafx.h"

#include <iostream>

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

void LOG(LogLevel level, const cv::Mat &m)
{
    if (level>=LOG_LEVEL){
		// TODO use QDebug()
		cv::MatConstIterator_<double> _it = m.begin<double>();
		for(;_it!=m.end<double>(); _it++){
			qDebug() << *_it;
		}

        //std::cout << m;
    }
}