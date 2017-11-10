//
// Created by bobin on 17-11-10.
//

#ifndef APRILTAG_TRACKER_H
#define APRILTAG_TRACKER_H

#include "PoseEstimate.h"
#include "opencv2/core/core.hpp"

class Tracker {
public:
    Tracker(const cv::FileStorage& _fsSetting);
    ~Tracker();
private:
    PoseEstimate *estimator;
    DUOReader duo_reader;
    cv::FileStorage fsSettings;
};


#endif //APRILTAG_TRACKER_H
