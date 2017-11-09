//
// Created by bobin on 17-10-7.
//

#include "DUOReader.h"
#include <stdio.h>
#include <opencv2/calib3d/calib3d.hpp>


DUOReader::DUOReader() {
    _evFrame = CreateEvent(NULL, 0, 0, NULL);
}

DUOReader::~DUOReader() {

}

// Opens, sets current image format, fps and start capturing
bool DUOReader::OpenDUOCamera(int width, int height, float fps) {
    if (_duo != NULL) {
        // Stop capture
        StopDUO(_duo);
        // Close DUO
        CloseDUO(_duo);
        _duo = NULL;
    }

    // Find optimal binning parameters for given (width, height)
    // This maximizes sensor imaging area for given resolution
    int binning = DUO_BIN_NONE;
    if (width <= 752 / 2)
        binning += DUO_BIN_HORIZONTAL2;
    else if (width <= 752 / 4)
        binning += DUO_BIN_HORIZONTAL4;
    if (height <= 480 / 4)
        binning += DUO_BIN_VERTICAL4;
    else if (height <= 480 / 2)
        binning += DUO_BIN_VERTICAL2;

    // Check if we support given resolution (width, height, binning, fps)
    DUOResolutionInfo ri;
    if (!EnumerateDUOResolutions(&ri, 1, width, height, binning, fps))
        return false;

    if (!OpenDUO(&_duo))
        return false;

    char tmp[260];
    // Get and print some DUO parameter values
    GetDUODeviceName(_duo, tmp);
    printf("DUO Device Name:      '%s'\n", tmp);
    GetDUOSerialNumber(_duo, tmp);
    printf("DUO Serial Number:    %s\n", tmp);
    GetDUOFirmwareVersion(_duo, tmp);
    printf("DUO Firmware Version: v%s\n", tmp);
    GetDUOFirmwareBuild(_duo, tmp);
    printf("DUO Firmware Build:   %s\n", tmp);

    // Set selected resolution
    SetDUOResolutionInfo(_duo, ri);

//    GetStereo();
    return true;
}

void DUOReader::StartDUOFrame(DUOFrameCallback frameCallback, void *pUserData) {
    // Start capture
    if (StartDUO(_duo, frameCallback, pUserData)) {
        // Wait for any key
        _getch();
        // Stop capture
        StopDUO(_duo);
        // Close DUO
        CloseDUO(_duo);
    }
}

// Waits until the new DUO frame is ready and returns it
PDUOFrame DUOReader::GetDUOFrame() {
    if (_duo == NULL)
        return NULL;

    if (WaitForSingleObject(_evFrame, 1000) == WAIT_OBJECT_0)
        return _pFrameData;
    else
        return NULL;
}

// Stops capture and closes the camera
void DUOReader::CloseDUOCamera() {
    if (_duo == NULL)
        return;

    // Stop capture
    StopDUO(_duo);
    // Close DUO
    CloseDUO(_duo);
    _duo = NULL;
}

void DUOReader::SetAutoExpose(bool value) {
    if (_duo == NULL)
        return;
    SetDUOAutoExposure(_duo, value);

}

void DUOReader::SetExposure(float value) {
    if (_duo == NULL)
        return;
    SetDUOExposure(_duo, value);
}

void DUOReader::SetGain(float value) {
    if (_duo == NULL)
        return;
    SetDUOGain(_duo, value);
}

void DUOReader::SetLed(float value) {
    if (_duo == NULL)
        return;
    SetDUOLedPWM(_duo, value);
}


void DUOReader::SetIMURate(double rate) {
    if (_duo == NULL)
        return;
    SetDUOIMURate(_duo, rate);

}

void DUOReader::SetUndistort(bool value) {
    if (_duo == NULL)
        return;
    SetDUOUndistort(_duo, value);
}

void DUOReader::GetLeftIntrinsic(cv::Mat &left_intr) {
    left_intr.create(3, 3, CV_64F);
    memcpy(left_intr.data, mStereo.M1, 9 * sizeof(double));
}


void DUOReader::GetRightIntrinsic(cv::Mat &right_intr) {
    right_intr.create(3, 3, CV_64F);
    memcpy(right_intr.data, mStereo.M2, 9 * sizeof(double));
}

void DUOReader::GetLeftDistort(cv::Mat &left_dist) {
    left_dist.create(1, 8, CV_64F);
    memcpy(left_dist.data, mStereo.D1, 8 * sizeof(double));
}

void DUOReader::GetRightDistort(cv::Mat &right_dist) {
    right_dist.create(1, 8, CV_64F);
    memcpy(right_dist.data, mStereo.D2, 8 * sizeof(double));
}

void DUOReader::GetLeftRectRot(cv::Mat &left_rect) {
    left_rect.create(3, 3, CV_64F);
    memcpy(left_rect.data, mStereo.R1, 9 * sizeof(double));
}

void DUOReader::GetLeftRectProj(cv::Mat &left_rect) {
    left_rect.create(3, 4, CV_64F);
    memcpy(left_rect.data, mStereo.P1, 12 * sizeof(double));
}

void DUOReader::GetRightRectRot(cv::Mat &right_rect) {
    right_rect.create(3, 3, CV_64F);
    memcpy(right_rect.data, mStereo.R2, 9 * sizeof(double));
}

void DUOReader::GetRightRectProj(cv::Mat &right_rect) {
    right_rect.create(3, 4, CV_64F);
    memcpy(right_rect.data, mStereo.P2, 12 * sizeof(double));
}

void DUOReader::GetExntrinsic(cv::Mat &_extr) {
    _extr = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat rot(3, 3, CV_64F);
    cv::Mat trans(1, 3, CV_64F);
    memcpy(rot.data, mExtr.rotation, 9 * sizeof(double));
    memcpy(trans.data, mExtr.translation, 3 * sizeof(double));
    rot.copyTo(_extr(cv::Rect(0, 0, 3, 3)));
    rot.copyTo(_extr(cv::Rect(0, 3, 1, 3)));
}

void DUOReader::GetStereo() {
    if (_duo == NULL)
        return;
    bool status;
    GetDUOCalibrationPresent(_duo, &status);
    if (status == false)
        return;
    GetDUOStereoParameters(_duo, &mStereo);
    std::cout << "left intrinsic parameters from camera" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << ' ' << mStereo.M1[3 * i + 0] << ' ' << mStereo.M1[3 * i + 1] << ' ' << mStereo.M1[3 * i + 2]
                  << std::endl;
    }
    std::cout << "right intrinsic parameters from camera" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << ' ' << mStereo.M2[3 * i + 0] << ' ' << mStereo.M2[3 * i + 1] << ' ' << mStereo.M2[3 * i + 2]
                  << std::endl;
    }

    std::cout << "left distortion parameters from camera" << std::endl;
    for (int i = 0; i < 8; ++i) {
        std::cout << ' ' << mStereo.D1[i] << std::endl;
    }
    std::cout << "right distortion parameters from camera" << std::endl;
    for (int i = 0; i < 8; ++i) {
        std::cout << ' ' << mStereo.D2[i] << std::endl;
    }
    std::cout << "left Rectified projection parameters from camera" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << ' ' << mStereo.P1[4 * i + 0] << ' ' << mStereo.P1[4 * i + 1] << ' ' << mStereo.P1[4 * i + 2] << ' '
                  << mStereo.P1[4 * i + 3] << std::endl;
    }
    std::cout << "right Rectified projection parameters from camera" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << ' ' << mStereo.P2[4 * i + 0] << ' ' << mStereo.P2[4 * i + 1] << ' ' << mStereo.P2[4 * i + 2] << ' '
                  << mStereo.P2[4 * i + 3] << std::endl;
    }
}

void DUOReader::GetCalib() {
    bool status;
    GetDUOCalibrationPresent(_duo, &status);
    if (status == true) {
        GetDUOExtrinsics(_duo, &mExtr);
        GetDUOIntrinsics(_duo, &mIntr);
    }
}

void DUOReader::initTermios(int echo) {
    tcgetattr(0, &_old); /* grab old terminal i/o settings */
    _new = _old; /* make new settings same as old settings */
    _new.c_lflag &= ~ICANON; /* disable buffered i/o */
    _new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
    tcsetattr(0, TCSANOW, &_new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void DUOReader::resetTermios(void) {
    tcsetattr(0, TCSANOW, &_old);
}

/* Read 1 character - echo defines echo mode */
char DUOReader::getch_(int echo) {
    char ch;
    initTermios(echo);
    ch = getchar();
    resetTermios();
    return ch;
}

/* Read 1 character without echo */
char DUOReader::_getch(void) {
    return getch_(0);
}

int DUOReader::_kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}


event_flag *DUOReader::CreateEvent(void *lpEventAttributes, bool bManualReset, bool bInitialState, char *name) {
    struct event_flag *ev = (struct event_flag *) malloc(sizeof(struct event_flag));
    pthread_mutex_init(&ev->mutex, NULL);
    pthread_cond_init(&ev->condition, NULL);
    ev->flag = 0;
    return ev;
}

void DUOReader::SetEvent(struct event_flag *ev) {
    pthread_mutex_lock(&ev->mutex);
    ev->flag = 1;
    pthread_cond_signal(&ev->condition);
    pthread_mutex_unlock(&ev->mutex);
}

int DUOReader::WaitForSingleObject(struct event_flag *ev, int timeout) {
    pthread_mutex_lock(&ev->mutex);
    while (!ev->flag)
        pthread_cond_wait(&ev->condition, &ev->mutex);
    ev->flag = 0;
    pthread_mutex_unlock(&ev->mutex);
    return WAIT_OBJECT_0;
}
