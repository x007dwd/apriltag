//
// Created by bobin on 17-11-10.
//

#include "Viewer.h"
#include <pangolin/pangolin.h>
#include "Tracker.h"
#include <mutex>

using namespace std;

Viewer::Viewer(const cv::FileStorage &fsSettings, Tracker* _tracker) {

    tracker = _tracker;
    float fps = fsSettings["Camera.FPS"];
    if (fps < 1)
        fps = 30;
    mT = 1e3 / fps;

    mImageWidth = fsSettings["Camera.width"];
    mImageHeight = fsSettings["Camera.height"];
    if (mImageWidth < 1 || mImageHeight < 1) {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fsSettings["Viewer.ViewpointX"];
    mViewpointY = fsSettings["Viewer.ViewpointY"];
    mViewpointZ = fsSettings["Viewer.ViewpointZ"];
    mViewpointF = fsSettings["Viewer.ViewpointF"];

    mKeyFrameSize = fsSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fsSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fsSettings["Viewer.GraphLineWidth"];
    mPointSize = fsSettings["Viewer.PointSize"];
    mCameraSize = fsSettings["Viewer.CameraSize"];
    mCameraLineWidth = fsSettings["Viewer.CameraLineWidth"];


    tracker->getAllTagPoints(vpMPs);
}

void Viewer::Run() {
    pangolin::CreateWindowAndBind("AprilTag: Map Viewer", 1024, 768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();
    bool bFollow = false;

    cv::namedWindow("AprilTag: Current Frame");

    while (1) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        GetCurrentOpenGLCameraMatrix(Twc);

        if (bFollow) {

            s_cam.SetModelViewMatrix(
                    pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        DrawCurrentCamera(Twc);
//        if (menuShowKeyFrames || menuShowGraph)
//            DrawKeyFrames(menuShowKeyFrames, menuShowGraph);

        DrawMapPoints();

        cv::imshow("Tag Detections", tracker->getDrawDetect());
        cv::waitKey(mT);

        pangolin::FinishFrame();

    }

}

void Viewer::DrawMapPoints() {

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0, 0.0, 0.0);

    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
        for (int j = 0; j < 4; ++j) {
            glVertex3f(vpMPs[i][j].x, vpMPs[i][j].y, vpMPs[i][j].z);
        }


    }
    glEnd();

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    vector<int>::iterator it = detect_ids.begin();
    for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            if (find(it, detect_ids.end(), i) != detect_ids.end())
                continue;
            glVertex3f(vpMPs[i][0].x, vpMPs[i][0].y, vpMPs[i][0].z);
            glVertex3f(vpMPs[i][1].x, vpMPs[i][1].y, vpMPs[i][1].z);
            glVertex3f(vpMPs[i][0].x, vpMPs[i][0].y, vpMPs[i][0].z);
            glVertex3f(vpMPs[i][2].x, vpMPs[i][2].y, vpMPs[i][2].z);
            glVertex3f(vpMPs[i][1].x, vpMPs[i][1].y, vpMPs[i][1].z);
            glVertex3f(vpMPs[i][3].x, vpMPs[i][3].y, vpMPs[i][3].z);
            glVertex3f(vpMPs[i][2].x, vpMPs[i][2].y, vpMPs[i][2].z);
            glVertex3f(vpMPs[i][3].x, vpMPs[i][3].y, vpMPs[i][3].z);

    }

    glEnd();

    glLineWidth(mCameraLineWidth);
    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINES);
    for (size_t i = 0, iend = detect_ids.size(); i < iend; i++) {
        int cnt = detect_ids[i];
        if (cnt >= vpMPs.size())
            continue;
        glVertex3f(vpMPs[cnt][0].x, vpMPs[cnt][0].y, vpMPs[cnt][0].z);
        glVertex3f(vpMPs[cnt][1].x, vpMPs[cnt][1].y, vpMPs[cnt][1].z);
        glVertex3f(vpMPs[cnt][0].x, vpMPs[cnt][0].y, vpMPs[cnt][0].z);
        glVertex3f(vpMPs[cnt][2].x, vpMPs[cnt][2].y, vpMPs[cnt][2].z);
        glVertex3f(vpMPs[cnt][1].x, vpMPs[cnt][1].y, vpMPs[cnt][1].z);
        glVertex3f(vpMPs[cnt][3].x, vpMPs[cnt][3].y, vpMPs[cnt][3].z);
        glVertex3f(vpMPs[cnt][2].x, vpMPs[cnt][2].y, vpMPs[cnt][2].z);
        glVertex3f(vpMPs[cnt][3].x, vpMPs[cnt][3].y, vpMPs[cnt][3].z);

    }

    glEnd();

}

//void Viewer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
//    const float &w = mKeyFrameSize;
//    const float h = w * 0.75;
//    const float z = w * 0.6;
//
//    const vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
//
//    if (bDrawKF) {
//        for (size_t i = 0; i < vpKFs.size(); i++) {
//            KeyFrame *pKF = vpKFs[i];
//            cv::Mat Twc = pKF->GetPoseInverse().t();
//
//            glPushMatrix();
//
//            glMultMatrixf(Twc.ptr<GLfloat>(0));
//
//            glLineWidth(mKeyFrameLineWidth);
//            glColor3f(0.0f, 0.0f, 1.0f);
//            glBegin(GL_LINES);
//            glVertex3f(0, 0, 0);
//            glVertex3f(w, h, z);
//            glVertex3f(0, 0, 0);
//            glVertex3f(w, -h, z);
//            glVertex3f(0, 0, 0);
//            glVertex3f(-w, -h, z);
//            glVertex3f(0, 0, 0);
//            glVertex3f(-w, h, z);
//
//            glVertex3f(w, h, z);
//            glVertex3f(w, -h, z);
//
//            glVertex3f(-w, h, z);
//            glVertex3f(-w, -h, z);
//
//            glVertex3f(-w, h, z);
//            glVertex3f(w, h, z);
//
//            glVertex3f(-w, -h, z);
//            glVertex3f(w, -h, z);
//            glEnd();
//
//            glPopMatrix();
//        }
//    }
//
//    if (bDrawGraph) {
//        glLineWidth(mGraphLineWidth);
//        glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
//        glBegin(GL_LINES);
//
//        for (size_t i = 0; i < vpKFs.size(); i++) {
//            // Covisibility Graph
//            const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
//            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
//            if (!vCovKFs.empty()) {
//                for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
//                     vit != vend; vit++) {
//                    if ((*vit)->mnId < vpKFs[i]->mnId)
//                        continue;
//                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
//                    glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                    glVertex3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2));
//                }
//            }
//
//            // Spanning tree
//            KeyFrame *pParent = vpKFs[i]->GetParent();
//            if (pParent) {
//                cv::Mat Owp = pParent->GetCameraCenter();
//                glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                glVertex3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2));
//            }
//
//            // Loops
//            set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
//            for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
//                if ((*sit)->mnId < vpKFs[i]->mnId)
//                    continue;
//                cv::Mat Owl = (*sit)->GetCameraCenter();
//                glVertex3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2));
//                glVertex3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2));
//            }
//        }
//
//        glEnd();
//    }
//}

void Viewer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
    const float &w = mCameraSize;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

#ifdef HAVE_GLES
    glMultMatrixf(Twc.m);
#else
    glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
}


void Viewer::SetCurrentCameraPose(const cv::Mat &Tcw) {
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void Viewer::SetCurrentDetectID(const std::vector<int> &ids) {
    unique_lock<mutex> lock(mMutexIds);
    detect_ids.clear();
    for (auto id : ids){
        detect_ids.push_back(id);
    }
}

void Viewer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {

    if (!mCameraPose.empty()) {
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0, 3).colRange(0, 3).t();
            twc = -Rwc * mCameraPose.rowRange(0, 3).col(3);
        }
//        cout << "Rwc=" << endl << Rwc << endl;
//        cout << "twc=" << endl << twc << endl;
        M.m[0] = Rwc.at<float>(0, 0);
        M.m[1] = Rwc.at<float>(1, 0);
        M.m[2] = Rwc.at<float>(2, 0);
        M.m[3] = 0.0;

        M.m[4] = Rwc.at<float>(0, 1);
        M.m[5] = Rwc.at<float>(1, 1);
        M.m[6] = Rwc.at<float>(2, 1);
        M.m[7] = 0.0;

        M.m[8] = Rwc.at<float>(0, 2);
        M.m[9] = Rwc.at<float>(1, 2);
        M.m[10] = Rwc.at<float>(2, 2);
        M.m[11] = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15] = 1.0;
    } else
        M.SetIdentity();
}
