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
    imagereadingthread.h

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

unix|win32: LIBS += -L$$PWD/../../opencv_246/build/x86/mingw/lib/ -lopencv_core246 \
             -llibopencv_imgproc246 \
             -llibopencv_features2d246 \
             -llibopencv_flann246

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../opencv_246/build/x86/vc10/lib/ -lopencv_calib3d246
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../opencv_246/build/x86/vc10/lib/ -lopencv_calib3d246d
#else:unix: LIBS += -L$$PWD/../../opencv_246/build/x86/vc10/lib/ -lopencv_calib3d246
