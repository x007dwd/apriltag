//
// Created by bobin on 17-11-10.
//

#ifndef APRILTAG_TRACKER_H
#define APRILTAG_TRACKER_H

#include "PoseEstimate.h"
#include "opencv2/core/core.hpp"
#include <iostream>
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

class Tracker {
public:
    Tracker(const cv::FileStorage& _fsSetting);
    ~Tracker();
    enum TrackState{
        Init,
        Start,
        Lost
    };
private:

    void InitEstimator();
    void InitReader();
    void InitDetector();
    void get_opt(getopt_t *getopt);
    void creat_detector(apriltag_family_t *tf , apriltag_detector_t *td, getopt_t *getopt);

    void collect_corners(zarray_t *detections, std::vector<cv::Point2f> &corners);
    void collect_point_3d(zarray_t *detections, std::vector<cv::Point3f> &detect_points);
    void draw_detect(zarray_t *detections, cv::Mat & image);
    void detect();
    void run();

    PoseEstimate *estimator;
    DUOReader duo_reader;
    cv::FileStorage fsSettings;
    apriltag_family_t *tf;
    apriltag_detector_t *td;
    getopt_t *getopt;

    TrackState  state;
};


#endif //APRILTAG_TRACKER_H
