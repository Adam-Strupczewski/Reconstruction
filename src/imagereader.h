#ifndef IMAGEREADER_H
#define IMAGEREADER_H

#include <QImage>
#include <QString>

#include "scenemodel.h"

class ImageReader
{
public:
    explicit ImageReader();
    virtual ~ImageReader();

	void initialize(SceneModel * sceneModel);

    void setImageBuffers(QImage *im1, QImage *im2);

    bool getNextImage();

private:

    QString folderPath;

    int currentImage;
	QStringList imageFileList;
	QStringList keypointFileList;

	SceneModel * sceneModel;

    // These buffers are set and managed externally
    QImage * imageBuffer1;
    QImage * imageBuffer2;
};

#endif // IMAGEREADER_H
