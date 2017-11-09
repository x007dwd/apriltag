//
// Created by bobin on 17-10-9.
//


#include <iostream>

#include "opencv2/opencv.hpp"

#include "include/apriltag.h"
#include "include/tag36h11.h"
#include "include/tag36h10.h"
#include "include/tag36artoolkit.h"
#include "include/tag25h9.h"
#include "include/tag25h7.h"
#include "common/getopt.h"
#include <DUOLib.h>
#include <SDK/include/DUOLib.h>
#include <glog/logging.h>
#include "DUOReader.h"
#include <thread>
using namespace std;
using namespace cv;


void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData) {
    PDUOFrame _pFrameData = pFrameData;
    DUOReader *duoReader = (DUOReader*)pUserData;
    unique_lock<mutex> lock(duoReader->mMutexCamera);
    duoReader->left.create(_pFrameData->height, _pFrameData->width, CV_8U);
    duoReader->right.create(_pFrameData->height, _pFrameData->width, CV_8U);
    duoReader->left.data = _pFrameData->leftData;
    duoReader->right.data = _pFrameData->rightData;
    duoReader->timeStamp = pFrameData->timeStamp;
    duoReader->ready = true;
}


void detect(void *reader, void *pUserData) {

//    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
//    clahe->apply(left, _left);
    cv::Mat frame, gray;
    DUOReader *duo_reader = (DUOReader*)reader;
    while (1) {
        if (duo_reader->ready == false)
            continue;
        gray = duo_reader->left;
        cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);
        image_u8_t im = {.width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
        };
        apriltag_detector_t *td = (apriltag_detector_t *) pUserData;
        zarray_t *detections = apriltag_detector_detect(td, &im);
        cout << zarray_size(detections) << " tags detected" << endl;

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[1][0], det->p[1][1]),
                 Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                 Point(det->p[2][0], det->p[2][1]),
                 Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0xff, 0, 0), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                        &baseline);
            putText(frame, text, Point(det->c[0] - textsize.width / 2,
                                       det->c[1] + textsize.height / 2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
        zarray_destroy(detections);

        imshow("Tag Detections", frame);
        waitKey(10);
        duo_reader->ready = false;
    }

}

#define WIDTH    752
#define HEIGHT    480
#define FPS        10



int main(int argc, char **argv) {

//    if (argc != 2) {
//        LOG(INFO) << "Usage: EurocStereoVIO path_to_config" << endl;
//        return 1;
//    }

    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

//    string configFile(argv[1]);
//    cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);
//
//    if (fsSettings.isOpened() == false) {
//        LOG(FATAL) << "Cannot load the config file from " << argv[1] << endl;
//    }


    DUOReader duo_reader;
    duo_reader.OpenDUOCamera(WIDTH, HEIGHT, FPS);
    duo_reader.SetGain(10);
//    duo_reader.SetExposure(100);
    duo_reader.SetAutoExpose(true);
    duo_reader.SetLed(10);
    duo_reader.SetIMURate(100);
    duo_reader.SetUndistort(false);


    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag25h9", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
        getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    apriltag_family_t *tf = NULL;
    tf = tag25h9_create();
    tf->black_border = getopt_get_int(getopt, "border");
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");

    std::thread* detect_thread;
    detect_thread = new thread(detect, &duo_reader, td);
    duo_reader.StartDUOFrame(DUOCallback, td);
    duo_reader.CloseDUOCamera();

    apriltag_detector_destroy(td);
    tag25h9_destroy(tf);
    getopt_destroy(getopt);

    return 0;
}

