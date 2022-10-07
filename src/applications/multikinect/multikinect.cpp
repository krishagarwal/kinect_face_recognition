#include "KFRDisplay.h"
#include "MKConfig.h"
#include "MKPPersonTracker.h"
#include "AsyncMKPRSplitter.h"
#include "MultiKinectWrapper.h"
#include <pthread.h>
#include <DepthViewer.h>
#include <unistd.h>
#include "Calibration.h"
#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>

namespace po = boost::program_options;

void *mkwThread(void *data) {
    DoOnce *mkw = (DoOnce *)data;
    for(;;) {
        mkw->doOnce();
    }
}

void *depthThread(void *data){
    MultiDepthViewer* depthViewer = ((MultiDepthViewer*) data);
    depthViewer->initOpenGLWindow(); // Must init in same thread as display? GL Context not current error
    while(true){
        depthViewer->displayContent();
    }
}

#include <fstream>
#include <iostream>
#include "MKPRMergeBodyTracks.h"
int main(int argc, char **argv) {
    po::options_description desc("Multi-Kinect Display");
    desc.add_options()
        ("calib_path,c", po::value<std::string>()->required(), "Path to calibration file");
    desc.add_options()
        ("record_path,r", po::value<std::string>(), "Path to recording files");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::vector<Eigen::MatrixXd> calib = Calibration::readFile(vm.at("calib_path").as<std::string>());

    MKConfig mkc(calib.size());
    if (vm.count("record_path") > 0) {
        mkc.recordFolder = vm.at("record_path").as<std::string>();
    }
    MultiKinectWrapper mkw(mkc);

    AsyncMKPRSplitter split;
    mkw.setRecipient(&split);

    MKPPersonTracker p(mkw);
    split.addRecipient(&p);

    MKPRMergeBodyTracks merge(calib);
    split.addRecipient(&merge);

    MultiDepthViewer depthViewer(calib);
    split.addRecipient(&depthViewer);

    DoOnce *doer = &mkw;
    pthread_t thread, thread2;
    pthread_create(&thread, NULL, mkwThread, doer);
    pthread_create(&thread2, NULL, depthThread, &depthViewer);
    pthread_join(thread, NULL);
    pthread_join(thread2, NULL);
    return 0;
}