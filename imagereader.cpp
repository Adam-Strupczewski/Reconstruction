#include "stdafx.h"

#include "imagereader.h"

#include <QImageReader>

ImageReader::ImageReader()
{
    folderPath = "images/";
    imageBaseName = "et00";
    imageAmount = 9;

    currentImage = 0;
}

ImageReader::~ImageReader()
{
}

void ImageReader::setImageBuffers(QImage *im1, QImage *im2){
    imageBuffer1 = im1;
    imageBuffer2 = im2;
}

bool ImageReader::getNextImage()
{
    // TODO read even / odd
    QString currentPath = getNextImagePath();
    QImageReader reader(currentPath);

    if (reader.canRead()){
        if ((currentImage%2)==1){
            return reader.read(imageBuffer1);
        }else{
            return reader.read(imageBuffer2);
        }
    }else{
        // TODO
        return false;
    }
}

QString ImageReader::getNextImagePath(){
    QString path = "";
    if (currentImage<imageAmount){
        path = folderPath + imageBaseName + QString::number(currentImage) + ".jpg";
        currentImage++;
    }

    return path;
}