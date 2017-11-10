//
// Created by bobin on 17-11-9.
//

#include "PoseEstimate.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;

PoseEstimate::PoseEstimate(float _max_trans, float _max_rot) {
    last_r = cv::Mat::zeros(1, 3, CV_64F);
    last_t = cv::Mat::zeros(1, 3, CV_64F);
    max_trans = _max_trans;
    max_rot = _max_rot;
}


float PoseEstimate::pose_distance(const cv::Mat &r, const cv::Mat &t, float &delta_rot, float &delta_trans) {
    delta_trans = cv::norm(t - last_t);
    delta_rot = cv::norm(r - last_r);

}

void PoseEstimate::object_points(int numGridX, int numGridY,
                   float GridWidth, float GridHeight,
                   float GridX, float GridY,
                   int num_ids) {
    int cnt = 0;;
    GridPointXY.resize(num_ids);
    for (int i = 0; i < numGridY; ++i) {
        vector<Point3f> rowGridXY;
        rowGridXY.resize(4);
        for (int j = 0; j < numGridX; ++j) {
            rowGridXY[0] = Point3f(j * GridX, i * GridY, 0);
            rowGridXY[1] = Point3f(j * GridX + GridWidth, i * GridY, 0);
            rowGridXY[2] = Point3f(j * GridX, i * GridY + GridHeight, 0);
            rowGridXY[3] = Point3f(j * GridX + GridWidth, i * GridY + GridHeight, 0);
            GridPointXY[cnt] = rowGridXY;
            cnt++;
            if(cnt >= num_ids)
                break;
        }

    }
}


void PoseEstimate::set_pose(const cv::Mat &r, const cv::Mat &t) {
    last_r = r;
    last_t = t;
}

void PoseEstimate::estimate(vector<Point3f> pts_3d,
                            vector<Point2f> pts_2d,
                            const Mat &K) {
    cv::Mat r, t, R;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false); // 调用OpenCV 的 PnP 求解，可选择EPNP，DLS等方法

    cout << "R=" << endl << R << endl;
    cout << "t=" << endl << t << endl;
    float delta_r, delta_t;
    pose_distance(r, t, delta_r, delta_t);
    if ((delta_r > max_rot) || (delta_t > max_trans)) {
        r = last_r;
        t = last_t;
    }

    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵

    PoseEstimate::bundleAdjustment(pts_3d, pts_2d, K, R, t);
    cv::Rodrigues(R, r); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    set_pose(r, t);

}


void PoseEstimate::bundleAdjustment(
        const vector<Point3f> points_3d,
        const vector<Point2f> points_2d,
        const Mat &K,
        Mat &R, Mat &t) {
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3> > Block;  // pose维度为 6, landmark 维度为 3
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block *solver_ptr = new Block(linearSolver);     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
            R_mat,
            Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
    ));
    optimizer.addVertex(pose);

    int index = 1;
    for (const Point3f p:points_3d)   // landmarks
    {
        g2o::VertexSBAPointXYZ *point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }

    // parameter: camera intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(
            K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0
    );
    camera->setId(0);
    optimizer.addParameter(camera);

    // edges
    index = 1;
    for (const Point2f p:points_2d) {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *> ( optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

    cout << endl << "after optimization:" << endl;
    cout << "T=" << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}