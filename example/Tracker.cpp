//
// Created by bobin on 17-11-10.
//

#include "Tracker.h"
Tracker::Tracker(const cv::FileStorage &_fsSetting) {
    fsSettings = _fsSetting;

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

    estimator = new PoseEstimate(max_trans, max_rot);
    estimator->set_camera(K);

    estimator.object_points(numGridX, numGridY, GridWidth, GridHeight, GridX, GridY, 35);
}

Tracker::~Tracker() {
    if (estimator)
        delete estimator;
    estimator = NULL;
}