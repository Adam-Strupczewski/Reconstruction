#ifndef IMAGEREADINGTHREAD_H
#define IMAGEREADINGTHREAD_H

#include <QThread>
#include <QImage>
#include <QQueue>

class ImageReadingThread : public QThread
{
    Q_OBJECT
public:
    explicit ImageReadingThread(QObject *parent = 0);
    virtual ~ImageReadingThread();

    //void setImageBuffers(QImage *image1, QImage *image);
signals:
    void imageReady(QImage *image);
public:
    void stop();
private:
    virtual void run();
private:
    bool stopped;
};

#endif // IMAGEREADINGTHREAD_H
