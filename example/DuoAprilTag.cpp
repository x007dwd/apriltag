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
#include "PoseEstimate.h"
using namespace std;
using namespace cv;


void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData) {
    PDUOFrame _pFrameData = pFrameData;
    DUOReader *duoReader = (DUOReader *) pUserData;
    unique_lock<mutex> lock(duoReader->mMutexCamera);
    duoReader->left.create(_pFrameData->height, _pFrameData->width, CV_8U);
    duoReader->right.create(_pFrameData->height, _pFrameData->width, CV_8U);
    duoReader->left.data = _pFrameData->leftData;
    duoReader->right.data = _pFrameData->rightData;
    duoReader->timeStamp = pFrameData->timeStamp;
    duoReader->ready = true;
    imshow("Tag", duoReader->left);
    waitKey(10);
}




void collect_corners(zarray_t *detections, vector<Point2f> &corners) {

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        for (int j = 0; j < 4; ++j) {
            corners.push_back(Point2f(det->p[j][0], det->p[j][1]));
        }

    }
}


void collect_point_3d(zarray_t *detections, const vector<vector<Point3f>> &all_points_3d, vector<Point3f> &detect_points) {
    unsigned num = zarray_size(detections);
    detect_points.resize(num * 4);
    for (int i = 0; i < num; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        for (int j = 0; j < 4; ++j) {
            detect_points[j] = all_points_3d[det->id][j];
        }

    }
}

void  draw_detect(zarray_t *detections, cv::Mat & image){
    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        line(image, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[1][0], det->p[1][1]),
             Scalar(0, 0xff, 0), 2);
        line(image, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0, 0, 0xff), 2);
        line(image, Point(det->p[1][0], det->p[1][1]),
             Point(det->p[2][0], det->p[2][1]),
             Scalar(0xff, 0, 0), 2);
        line(image, Point(det->p[2][0], det->p[2][1]),
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
        putText(image, text, Point(det->c[0] - textsize.width / 2,
                                   det->c[1] + textsize.height / 2),
                fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
    }
}


void detect(const cv::Mat gray, apriltag_detector_t *td, PoseEstimate *estimator) {

    cv::Mat color;
    cv::cvtColor(gray, color, cv::COLOR_GRAY2BGR);
    image_u8_t im = {.width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    cout << zarray_size(detections) << " tags detected" << endl;
    vector<Point2f> corners;

    vector<Point3f> detect_points;
    collect_corners(detections, corners);
    collect_point_3d(detections, estimator->GridPointXY, detect_points);
//    estimator->set_pose();
//    estimator
    estimator->estimate(detect_points, corners);
    draw_detect(detections, color);
    zarray_destroy(detections);
    imshow("Tag Detections", color);
    waitKey(10);
}

void detect_func(void *reader, void *detector, void * estimator) {

    cv::Mat frame, gray;
    DUOReader *duo_reader = (DUOReader *) reader;
    PoseEstimate *pestimator = (PoseEstimate *)estimator;
    while (1) {
//        cout << duo_reader->ready << endl;
        if (duo_reader->ready == false)
            continue;
        detect(duo_reader->left, (apriltag_detector *) detector, pestimator);
        duo_reader->ready = false;
    }

}

void DUOInit(DUOReader& duo_reader, int image_width, int image_height, int fps){
    duo_reader.OpenDUOCamera(image_width, image_height, fps);
    duo_reader.SetGain(10);
    duo_reader.SetExposure(100);
//    duo_reader.SetAutoExpose(true);
    duo_reader.SetLed(30);
    duo_reader.SetIMURate(10);
    duo_reader.SetUndistort(true);
}

void get_opt(getopt_t *getopt, int argc, char **argv){
    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag25h9", "Tag family to use");
    getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
    getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 0, "Spend more time trying to align edges of tags");
    getopt_add_bool(getopt, '1', "refine-decode", 1, "Spend more time trying to decode tags");
    getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time trying to precisely localize tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
        getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }
}


void creat_detector(apriltag_family_t *tf , apriltag_detector_t *td, getopt_t *getopt){

    tf->black_border = getopt_get_int(getopt, "border");
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
    td->refine_pose = getopt_get_bool(getopt, "refine-pose");
}

int main(int argc, char **argv) {

    if (argc != 2) {
        LOG(INFO) << "Usage: EurocStereoVIO path_to_config" << endl;
        return 1;
    }

    FLAGS_logtostderr = true;
    google::InitGoogleLogging(argv[0]);

    string configFile(argv[1]);
    cv::FileStorage fsSettings(configFile, cv::FileStorage::READ);

    if (fsSettings.isOpened() == false) {
        LOG(FATAL) << "Cannot load the config file from " << argv[1] << endl;
    }

    int image_width, image_height;
    image_width = fsSettings["Camera.width"];
    image_height = fsSettings["Camera.height"];
    int fps = fsSettings["Camera.FPS"];

    float fx, fy, cx, cy;
    fx = fsSettings["Camera.fx"];
    fy = fsSettings["Camera.fy"];
    cx = fsSettings["Camera.cx"];
    cy = fsSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) =fx;
    K.at<double>(1, 1) =fy;
    K.at<double>(0, 2) =cx;
    K.at<double>(1, 2) =cy;

    int numGridX, numGridY;
    float GridWidth, GridHeight;
    float GridX, GridY;

    GridHeight = fsSettings["Tag.width"];
    GridWidth = fsSettings["Tag.height"];
    GridX = fsSettings["Tag.gridx"];
    GridY = fsSettings["Tag.gridy"];
    numGridX = fsSettings["Tag.numx"];
    numGridY = fsSettings["Tag.numy"];

    float max_trans, max_rot;
    max_rot = fsSettings["maxRot"];
    max_trans = fsSettings["maxTrans"];

    PoseEstimate estimator(max_trans, max_rot);
    estimator.set_camera(K);

    estimator.object_points(numGridX, numGridY, GridWidth, GridHeight, GridX, GridY, 35);
//    vector<vector<Point3f>> &points = estimator.GridPointXY;

//    for (int i = 0; i < 35; ++i) {
//        for (int j = 0; j < 4; ++j) {
//            cout << points[i][j].x << '\t' << points[i][j].y << endl;
//        }
//    }

    DUOReader duo_reader;
    DUOInit(duo_reader, image_width, image_height, fps);

    getopt_t *getopt = getopt_create();
    get_opt(getopt, argc, argv);

    apriltag_family_t *tf = NULL;
    apriltag_detector_t *td = NULL;
    td = apriltag_detector_create();
    tf = tag25h9_create();
    creat_detector(tf, td, getopt);

    std::thread *detect_thread;

    detect_thread = new thread(detect_func, &duo_reader, td, &estimator);
    duo_reader.StartDUOFrame(DUOCallback, &duo_reader);
    duo_reader.CloseDUOCamera();


    apriltag_detector_destroy(td);
    tag25h9_destroy(tf);
    getopt_destroy(getopt);
    return 0;
}

