#include "stdafx.h"

#include "imagereader.h"

#include <QImageReader>

ImageReader::ImageReader()
{
	folderPath = "images4/";
    currentImage = 0;
}

ImageReader::~ImageReader()
{
}

void ImageReader::initialize(SceneModel * sceneModel)
{
	this->sceneModel = sceneModel;

	QStringList nameFilter("*.jpg");
	nameFilter << "*.JPG";
	nameFilter << "*.png";
	nameFilter << "*.PNG";

	QDir directory(folderPath);
	imageFileList = directory.entryList(nameFilter, QDir::Files);

	if (imageFileList.size()==0)
		return;

	// Save image file list and descriptor file list in sceneModel
	sceneModel->folderPath = folderPath;
	sceneModel->imageFileList = imageFileList;
	keypointFileList = imageFileList;

	QString extension = imageFileList[0].right(4);
	keypointFileList.replaceInStrings(extension, ".key");
	sceneModel->keypointFileList = keypointFileList;
}

void ImageReader::setImageBuffers(QImage *im1, QImage *im2){
    imageBuffer1 = im1;
    imageBuffer2 = im2;
}

bool ImageReader::getNextImage()
{
	if (currentImage>=imageFileList.size())
		return false;

    QString currentPath = folderPath + imageFileList.at(currentImage);
	currentImage++;

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
