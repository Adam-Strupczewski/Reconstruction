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

void LOG(LogLevel level, const char* message1, int i1, const char* message2, int i2)
{
    if (level>=LOG_LEVEL){
        qDebug() << message1 << i1 << message2 << i2;
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
			for(int j=0; j<cols; j++){
				debug << m.at<double>(i,j) << " ";
			}
		}

		//cv::MatConstIterator_<double> _it = m.begin<double>();
		//for(;_it!=m.end<double>(); _it++){
		//	qDebug() << *_it;
		//}

        //std::cout << m;
    }
}