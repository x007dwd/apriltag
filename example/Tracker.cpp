//
// Created by bobin on 17-11-10.
//

#include "Tracker.h"

using namespace cv;
using namespace std;

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
    cv::imshow("Tag Detections", duoReader->left);
    cv::waitKey(10);

}

Tracker::Tracker(const cv::FileStorage &_fsSetting) {
    fsSettings = _fsSetting;
    tf = NULL;
    td = NULL;
    InitEstimator();
    InitReader();
    InitDetector();
    state = Init;
    std::thread *detectThread = new thread(&Tracker::run, this);

    viewer = new Viewer(fsSettings, this);
    std::thread *viewThread = new thread(&Viewer::Run, viewer);

    duo_reader.StartDUOFrame(DUOCallback, &duo_reader);

}

void Tracker::InitEstimator() {

    float fx, fy, cx, cy;
    fx = fsSettings["Camera.fx"];
    fy = fsSettings["Camera.fy"];
    cx = fsSettings["Camera.cx"];
    cy = fsSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    K.at<double>(0, 0) = fx;
    K.at<double>(1, 1) = fy;
    K.at<double>(0, 2) = cx;
    K.at<double>(1, 2) = cy;

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

    estimator = new PoseEstimate(max_trans, max_rot);
    estimator->set_camera(K);

    estimator->object_points(numGridX, numGridY, GridWidth, GridHeight, GridX, GridY, 35);
}

void Tracker::InitReader() {

    int image_width, image_height;
    image_width = fsSettings["Camera.width"];
    image_height = fsSettings["Camera.height"];
    int fps = fsSettings["Camera.FPS"];

    if (false == duo_reader.OpenDUOCamera(image_width, image_height, fps))
        exit(0);
    duo_reader.SetGain(10);
    duo_reader.SetExposure(100);
//    duo_reader.SetAutoExpose(true);
    duo_reader.SetLed(30);
    duo_reader.SetIMURate(10);
    duo_reader.SetUndistort(true);

}

void Tracker::get_opt(getopt_t *getopt) {
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


}

void Tracker::InitDetector() {
    getopt = getopt_create();
    get_opt(getopt);
    td = apriltag_detector_create();
    tf = tag25h9_create();
    creat_detector(tf, td, getopt);
}

void Tracker::creat_detector(apriltag_family_t *tf, apriltag_detector_t *td, getopt_t *getopt) {

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


Tracker::~Tracker() {
    if (estimator)
        delete estimator;
    estimator = NULL;
    duo_reader.CloseDUOCamera();
    apriltag_detector_destroy(td);
    tag25h9_destroy(tf);
    getopt_destroy(getopt);
}


void Tracker::collect_corners(zarray_t *detections, vector<Point2f> &corners) {
    int num = zarray_size(detections);
    corners.resize(num * 4);
    for (int i = 0; i < num; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        corners[i * 4] = Point2f(det->p[3][0], det->p[3][1]);
        corners[i * 4 + 1] = Point2f(det->p[2][0], det->p[2][1]);
        corners[i * 4 + 2] = Point2f(det->p[0][0], det->p[0][1]);
        corners[i * 4 + 3] = Point2f(det->p[1][0], det->p[1][1]);


    }
}


void Tracker::collect_point_3d(zarray_t *detections, vector<Point3f> &detect_points) {
    unsigned num = zarray_size(detections);
    detect_points.resize(num * 4);
    for (int i = 0; i < num; i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        for (int j = 0; j < 4; ++j) {
            detect_points[4 * i + j] = estimator->GridPointXY[det->id][j];
        }

    }
}

void Tracker::draw_detect(zarray_t *detections, cv::Mat &image) {
    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        line(image, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[1][0], det->p[1][1]),
             Scalar(0, 0xff, 0), 1);
        line(image, Point(det->p[0][0], det->p[0][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0, 0, 0xff), 2);
        line(image, Point(det->p[1][0], det->p[1][1]),
             Point(det->p[2][0], det->p[2][1]),
             Scalar(0xff, 0, 0), 3);
        line(image, Point(det->p[2][0], det->p[2][1]),
             Point(det->p[3][0], det->p[3][1]),
             Scalar(0xff, 0, 0), 4);

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


void Tracker::detect() {

    cv::Mat color;
    cv::Mat gray = duo_reader.left;
    cv::cvtColor(gray, color, cv::COLOR_GRAY2RGB);
    image_u8_t im = {.width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);
    int detect_num = zarray_size(detections);
    cout << detect_num << " tags detected" << endl;

    vector<int> detect_ids;
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        detect_ids.push_back(det->id);
    }
    viewer->SetCurrentDetectID(detect_ids);
    vector<Point2f> corners;
    vector<Point3f> detect_points;
    collect_corners(detections, corners);
    collect_point_3d(detections, detect_points);

    if (detect_num >= 2) {
        if (state == Init) {
            estimator->estimate(detect_points, corners);
            state = Start;
        } else {
            estimator->estimate(detect_points, corners, true);
        }

    } else {
        state = Init;
    }
    cv::Mat cur_pose = Mat::eye(4, 4, CV_32F);
    estimator->get_pose(cur_pose);
    viewer->SetCurrentCameraPose(cur_pose);
    draw_detect(detections, color);
    drawDetect = color;
    zarray_destroy(detections);

}


void Tracker::run() {

    while (1) {
//        unique_lock<mutex> lock(duoMutex);
        if (duo_reader.ready == false)
            continue;
        duo_reader.ready = false;
        detect();
    }

}

void Tracker::getAllTagPoints(std::vector<std::vector<cv::Point3f>> &points) {
    points = estimator->GridPointXY;
}
