TEMPLATE = app
TARGET = camera

QT += multimedia multimediawidgets

HEADERS = \
    videosurface.h \
    mainwindow.h \
    videowidget.h \
    processingthread.h \
    logging.h

SOURCES = \
    main.cpp \
    videosurface.cpp \
    mainwindow.cpp \
    videowidget.cpp \
    processingthread.cpp \
    logging.cpp

FORMS += \
    mainwindow.ui

target.path = $$[QT_INSTALL_EXAMPLES]/multimediawidgets/camera
INSTALLS += target

QT+=widgets
