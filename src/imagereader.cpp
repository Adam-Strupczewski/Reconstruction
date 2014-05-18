#include "stdafx.h"

#include "imagereader.h"

#include <QImageReader>

ImageReader::ImageReader()
{
    //folderPath = "images/";
    //imageBaseName = "et00";
	folderPath = "images2/";
    imageBaseName = "p00";
	//folderPath = "images3/";
    //imageBaseName = "R000";
	//folderPath = "images4/";
    //imageBaseName = "kermit00";
    imageAmount = 10;

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
    QString currentPath = getNextImagePath();
    QImageReader reader(currentPath);
	reader.setScaledSize(QSize(640,480));

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
