//
// Created by bobin on 17-11-10.
//

#ifndef APRILTAG_VIEWER_H
#define APRILTAG_VIEWER_H

#include "opencv2/core/core.hpp"
#include <mutex>
#include <pangolin/pangolin.h>
class Tracker;
class PoseEstimate;

class Viewer {
public:
    Viewer(const cv::FileStorage &fsSettings, Tracker* _tracker);

    void Run();

    void DrawMapPoints();

    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    void SetCurrentCameraPose(const cv::Mat &Tcw);

    void SetCurrentDetectID(const std::vector<int> &ids);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:
    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    Tracker * tracker;

    std::vector<std::vector<cv::Point3f>> vpMPs;


    std::vector<int> detect_ids;

    std::mutex mMutexCamera;
    std::mutex mMutexIds;

};


#endif //APRILTAG_VIEWER_H
