//
// Created by bobin on 17-11-9.
//

#ifndef APRILTAG_POSEESTIMATE_H
#define APRILTAG_POSEESTIMATE_H

#include <opencv2/core/core.hpp>
#include <vector>


class PoseEstimate {

public:
    PoseEstimate(float _max_trans, float _max_rot);
    static void bundleAdjustment (
            const std::vector< cv::Point3f > points_3d,
            const std::vector< cv::Point2f > points_2d,
            const cv::Mat& K,
            cv::Mat& R, cv::Mat& t );
    void estimate(std::vector<cv::Point3f> pts_3d,
                                std::vector<cv::Point2f> pts_2d,
                                const cv::Mat &K);
    void set_pose(const cv::Mat &r, const cv::Mat& t);

    float pose_distance(const cv::Mat &r, const cv::Mat& t, float& delta_rot, float& delta_trans);
private:
    cv::Mat last_r;
    cv::Mat last_t;
    float max_trans;
    float max_rot;
};


#endif //APRILTAG_POSEESTIMATE_H
