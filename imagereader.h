#ifndef IMAGEREADER_H
#define IMAGEREADER_H

#include <QImage>
#include <QString>

class ImageReader
{
public:
    explicit ImageReader();
    virtual ~ImageReader();

    void setImageBuffers(QImage *im1, QImage *im2);

    bool getNextImage();

private:

    QString getNextImagePath();

    QString folderPath;
    QString imageBaseName;
    int imageAmount;

    int currentImage;

    // These buffers are set and managed externally
    QImage * imageBuffer1;
    QImage * imageBuffer2;
};

#endif // IMAGEREADER_H
