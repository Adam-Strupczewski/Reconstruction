TEMPLATE = app
TARGET = camera

QT += multimedia multimediawidgets

HEADERS = \
    videosurface.h \
    mainwindow.h \
    videowidget.h \
    processingthread.h \
    logging.h \
    qtcvconversion.h \
    imagereader.h \
    settings.h \
    imagereadingthread.h \
    stdafx.h

SOURCES = \
    main.cpp \
    videosurface.cpp \
    mainwindow.cpp \
    videowidget.cpp \
    processingthread.cpp \
    logging.cpp \
    qtcvconversion.cpp \
    imagereader.cpp \
    imagereadingthread.cpp

FORMS += \
    mainwindow.ui

target.path = $$[QT_INSTALL_EXAMPLES]/multimediawidgets/camera
INSTALLS += target

QT+=widgets

INCLUDEPATH += $$PWD/../../opencv_246/build/include
DEPENDPATH += $$PWD/../../opencv_246/build/include

unix|win32: LIBS += -L$$PWD/../../opencv_246/build/x86/vc10/lib/ -lopencv_core246d \
             -lopencv_imgproc246d \
             -lopencv_features2d246d \
             -lopencv_flann246d

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../opencv_246/build/x86/vc10/lib/ -lopencv_calib3d246
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../opencv_246/build/x86/vc10/lib/ -lopencv_calib3d246d
#else:unix: LIBS += -L$$PWD/../../opencv_246/build/x86/vc10/lib/ -lopencv_calib3d246
