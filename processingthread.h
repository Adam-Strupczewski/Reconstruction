#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

#include <QThread>
#include <QImage>
#include <QQueue>

class ProcessingThread : public QThread
{
    Q_OBJECT
public:
    explicit ProcessingThread(QObject *parent = 0);
    virtual ~ProcessingThread();
signals:
    void frameProcessed();
    void queueFull();
public:
    void stop();
    void addFrameToProcessingQueue(QImage frame);
private:
    virtual void run();
private:
    QQueue<QImage> queue;
    int queueMaxLength;
    bool stopped;
};

#endif // PROCESSINGTHREAD_H
