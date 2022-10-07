#include "KFRDisplay.h"
#include "MKConfig.h"
#include "AsyncMKPRSplitter.h"
#include "MKPRRecord.h"
#include "MultiKinectWrapper.h"
#include <gtk/gtk.h>
#include <pthread.h>
#include <unistd.h>
#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>
#include <string>
#include <iostream>

namespace po = boost::program_options;
using namespace std;
GtkWidget *window, *window2, *window3;
AsyncMKPRSplitter splitter;

int captures = 0;
int n = 150;
// TODO Address following error/warning for n > 150
//  [error] [t=27973] /__w/1/s/extern/Azure-Kinect-Sensor-SDK/src/record/internal/matroska_write.cpp (487): 
//              matroska_writer_thread(). Disk write speed is too low, write queue is filling up.

void *mkwThread(void *data) {
    DoOnce *mkw = (DoOnce *)data;
    while(++captures < n){
        mkw->doOnce();
        cout << "i: " << captures << endl;
    }
    cout << "Done capturing" << endl;
}

void *stopRecordThread(void *data) {
    MKPRRecord *record = (MKPRRecord *)data;
    while(captures < n){
        sleep(1);
    }
    record->close();
    cout << "Stopped recording, recording handles closed" << endl;
    exit(0);
}

gboolean exit_program(GtkWidget *widget, GdkEvent *event, gpointer data) {
    //if menu closed exit program entirely. 
    exit(0);
    return TRUE;
}

static void activateMultiWithKFRD (GtkApplication *app, gpointer user_data){
    window3 = gtk_application_window_new(app);
    gtk_window_set_title (GTK_WINDOW (window3), "Multi Kinect with AprilTag" );
    gtk_window_set_default_size(GTK_WINDOW(window3), 1920, 1080 );

    ((KFRDisplay *)user_data)->buildWidgets(window3);

    g_signal_connect(window3, "destroy", G_CALLBACK(exit_program), NULL);
    gtk_widget_show_all (window3);
}

int main(int argc, char **argv) {
    po::options_description desc("Multi-Kinect Recording");
    desc.add_options()("out_path,o", po::value<std::string>()->required(), "Path to write recordings to");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    MKConfig mkc(2);
    mkc._config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    MultiKinectWrapper mkw(mkc);
    AsyncMKPRSplitter split;    
    mkw.setRecipient(&split);

    KFRDisplay kfrd(1080, 1920, 2);
    split.addRecipient(&kfrd);

    // Init MKPRRecord with specified target path
    MKPRRecord mkprRecord(vm.at("out_path").as<std::string>());
    split.addRecipient(&mkprRecord);

    GtkApplication *app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
    g_signal_connect (app, "activate", G_CALLBACK (activateMultiWithKFRD), &kfrd);

    DoOnce *doer = &mkw;
    pthread_t thread, thread2;
    pthread_create(&thread, NULL, mkwThread, doer);
    // Thread to close recording files, could just pass into above also
    pthread_create(&thread, NULL, stopRecordThread, &mkprRecord);

    int status = g_application_run (G_APPLICATION (app), 0, argv);
    g_object_unref (app);
    return 0;
}