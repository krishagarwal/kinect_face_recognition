#include "KFRDisplay.h"
#include "MKConfig.h"
#include "AsyncMKPRSplitter.h"
#include "MultiKinectWrapper.h"
#include <gtk/gtk.h>
#include <pthread.h>
#include <unistd.h>
#include <string>
#include "MKPRAprilTag.h"
#include "MKPRCornerDisplay.h"
#include "MKPRSaveFrame.h"
#include <boost/program_options.hpp>
#include <boost/program_options/options_description.hpp>
#include <iostream>

namespace po = boost::program_options;
using namespace std;
GtkWidget *window, *window2, *window3;
AsyncMKPRSplitter splitter;
/* Display the multikinect camera view with aprilTag functionality
    and relative transformations  */

void *mkwThread(void *data) {
        DoOnce *mkw = (DoOnce *)data;
        for(;;) {
            mkw->doOnce();
        }
}

struct Data{
    KFRDisplay* kfrd;
    MKPRSaveFrame* mkprsf;
};

gboolean my_keypress_function (GtkWidget *widget, GdkEventKey *event, gpointer data) {
    if (event->keyval == GDK_KEY_space){
        cout << "x" << endl;
        ((MKPRSaveFrame*) data)->saveFrame();
        cout << "SAVING" << endl;
        return TRUE;
    }
    cout << "y" << endl;
    return FALSE;
}


gboolean exit_program(GtkWidget *widget, GdkEvent *event, gpointer data) {
    //if menu closed exit program entirely. 
    exit(0);
    return TRUE;
}

static void aprilTagSnapper (GtkApplication *app, gpointer user_data){
    window = gtk_application_window_new(app);
    gtk_window_set_title (GTK_WINDOW (window), "Multi Kinect with AprilTag" );
    gtk_window_set_default_size(GTK_WINDOW(window), 1920, 1080 );

    gtk_widget_add_events(window, GDK_KEY_PRESS_MASK);

    Data* data = (Data*) user_data;

    g_signal_connect(window, "key_press_event", G_CALLBACK (my_keypress_function), data->mkprsf);



    data->kfrd->buildWidgets(window);

    g_signal_connect(window, "destroy", G_CALLBACK(exit_program), NULL);
    gtk_widget_show_all (window);
}

// static void writeToFileCallback(GtkWidget *widget, gpointer cameraNumber){
//     char *cameras = (char *) cameraNumber;
//     if(atp->writeToFile(cameras)){
//         gtk_widget_set_sensitive (widget, FALSE);
//     }
// }

// static void readFromFileCallback(GtkWidget *widget, gpointer pointer){
//     atp->readFromFile();
// }

// static void getRelativeTransforms(GtkWidget *widget, gpointer pointer){
//     atp->relativeTransforms();
// }

static void buildButton(GtkApplication *app, gpointer atpLocal){
    GtkWidget *button0, *button1, *button2, *button3, *button4, *button5, *button6, *button7, *button8, *button9;
    window2 = gtk_application_window_new(app);
    gtk_window_set_title (GTK_WINDOW (window2), "Save Relative Transforms" );

    GtkGrid *_gtkGrid = (GtkGrid*)gtk_grid_new();
    button0 = gtk_button_new_with_label("CLEAR FILE");
    button8 = gtk_button_new_with_label("Read from file");

    button1 = gtk_button_new_with_label("Save Transform: Camera 0,1");
    button2 = gtk_button_new_with_label("Save Transform: Camera 1,2");
    button3 = gtk_button_new_with_label("Save Transform: Camera 2,3");
    button4 = gtk_button_new_with_label("Save Transform: Camera 3,4");
    button5 = gtk_button_new_with_label("Save Transform: Camera 4,5");
    button6 = gtk_button_new_with_label("Save Transform: Camera 5,6");
    button7 = gtk_button_new_with_label("Save Transform: Camera 6,7");
    button9 = gtk_button_new_with_label("Get Relative Transforms");
    

    // g_signal_connect (button1, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "0,1");
    // g_signal_connect (button2, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "1,2");
    // g_signal_connect (button3, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "2,3");
    // g_signal_connect (button4, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "3,4");
    // g_signal_connect (button5, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "4,5");
    // g_signal_connect (button6, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "5,6");
    // g_signal_connect (button7, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "6,7");

    // g_signal_connect (button8, "clicked", G_CALLBACK (readFromFileCallback), (gpointer) "6,7");
    // g_signal_connect (button0, "clicked", G_CALLBACK (writeToFileCallback), (gpointer) "9,9");
    // g_signal_connect (button9, "clicked", G_CALLBACK (getRelativeTransforms), (gpointer) "getRelativeTransform");

    // gtk_grid_attach(_gtkGrid,button1, 0, 0 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button2, 0, 1 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button3, 0, 2 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button4, 0, 3 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button5, 0, 4 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button6, 0, 5 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button7, 0, 6 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button0, 0, 7 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button8, 0, 8 , 1, 1);
    // gtk_grid_attach(_gtkGrid,button9, 0, 9 , 1, 1);

    gtk_container_add(GTK_CONTAINER (window2), (GtkWidget * ) _gtkGrid );
    gtk_window_set_default_size(GTK_WINDOW(window2), 200, 200 );
    g_signal_connect(window2, "destroy", G_CALLBACK(exit_program), NULL);
    gtk_widget_show_all (window2);
}

int main(int argc, char **argv) {
    //Used for parsing command line arguments
    po::options_description desc("April Tag Snapper Usage");
    desc.add_options()
        ("num_cameras,n", po::value<int>(),"Number of Kinects (defaults to all connected)")
        ("transform_path,t", po::value<std::string>()->required(), "File path and/or name for saving transforms");
    // cout << argc << endl;
    // cout << &argv << endl;
    

    po::variables_map vm;
    po::store(parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    size_t numCameras = k4a::device::get_installed_count();
    if(vm.count("num_cameras")){
        numCameras = (size_t)vm["num_cameras"].as<int>();
    }

    MKConfig mkc(numCameras);
    MultiKinectWrapper mkw(mkc);

    mkw.setRecipient(&splitter); 

    string output_path = vm["transform_path"].as<string>();
    MKPRSaveFrame saveFrameMKPR(output_path);
    splitter.addRecipient(&saveFrameMKPR);

    MKPRAprilTag apriltagMKPR;
    MKPRCornerDisplay cornerDisplayMKPR;

    // Passes frame through corner detection (AprilTag-based) and draws corners in packet buffer
    splitter.addRecipient(&apriltagMKPR);
    splitter.addRecipient(&cornerDisplayMKPR);

    KFRDisplay kfrDisplay(mkc._rowsC, mkc._colsC, 4);
    splitter.addRecipient(&kfrDisplay);
    
    GtkApplication *app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);

    Data args = {&kfrDisplay, &saveFrameMKPR};

    g_signal_connect (app, "activate", G_CALLBACK (aprilTagSnapper), &args);
    // g_signal_connect (app, "activate", G_CALLBACK (buildButton), atp);

    DoOnce *doer = &mkw;
    pthread_t thread, thread2;
    pthread_create(&thread, NULL, mkwThread, doer);

    int status = g_application_run (G_APPLICATION (app), 0, argv);
    g_object_unref (app);
    return 0;
}