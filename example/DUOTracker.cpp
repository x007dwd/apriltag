//
// Created by bobin on 17-11-10.
//

#include <glog/logging.h>
#include "opencv2/core/core.hpp"
#include "string"
#include "Tracker.h"

using namespace cv;
using namespace std;




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
    Tracker tracker(fsSettings);

}