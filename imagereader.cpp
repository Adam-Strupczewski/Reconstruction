#include "imagereader.h"

#include <QImageReader>

ImageReader::ImageReader()
{
    folderPath = "images/";
    imageBaseName = "et00";
    imageAmount = 9;

    currentImage = 0;
}

QImage ImageReader::getNextImage()
{
    QString currentPath = getNextImagePath();
    QImageReader reader(currentPath);

    if (reader.canRead()){
        return reader.read();
    }else{
        return reader.read();
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
