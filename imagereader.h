#ifndef IMAGEREADER_H
#define IMAGEREADER_H

#include <QImage>
#include <QString>

class ImageReader
{
public:
    ImageReader();
    QImage getNextImage();

private:

    QString getNextImagePath();

    QString folderPath;
    QString imageBaseName;
    int imageAmount;

    int currentImage;
};

#endif // IMAGEREADER_H
