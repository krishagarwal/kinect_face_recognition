#include "include/KFRDisplay.h"
#include "include/MKConfig.h"
#include "include/MultiKinectPacket.h"
//#include "conversions.h"

#include <iostream>
using namespace std;

KFRDisplay::KFRDisplay(int rows, int cols, int nCameras) :
    _rows(rows), _cols(cols), _initialized(false),
    _darea(NULL), _cameras(nCameras),
    //_pixbuf(NULL),
    _buf(NULL) {
    _gridRows = ceil(sqrt(nCameras));
}

KFRDisplay::~KFRDisplay() {
    if(_initialized) {

    }
}

void KFRDisplay::buildWidgets(GtkWidget *container) {
    _initialized = true;
    //Create a grid, adjust spacing 
    _gtkGrid = (GtkGrid*)gtk_grid_new();
    gtk_container_add(GTK_CONTAINER (container), (GtkWidget * ) _gtkGrid );
    gtk_grid_set_row_homogeneous(_gtkGrid, true);
    gtk_grid_set_column_homogeneous(_gtkGrid, true);

    //Add cameras
    int cells = _gridRows * _gridRows;
    for (int i = 0; i < cells; i++){
        drawingAreas[i] = gtk_drawing_area_new();
    } 

    //Callback methods
    int count = 0;
    //Two loops - rows, columns
    for (int i = 0; i < _gridRows; i++){
        for (int j = 0; j < _gridRows;j++){
            gtk_grid_attach(_gtkGrid,drawingAreas[count], j, i , 1, 1);
            g_signal_connect (G_OBJECT (drawingAreas[count++]), "draw", G_CALLBACK (drawCallback), this);
        }
    }   
}

void KFRDisplay::receiveFrame(MultiKinectPacket &mkp) {
    //Add the frame to the buffer
    _rows = 540;
    _cols = 960;
    size_t bytes = _cols * _rows * 3;
    if(_buf == NULL) {
        _buf = new unsigned char*[8];
        for (int i = 0; i < 8; i++) {
            _buf[i] = (unsigned char*)calloc(1, bytes);
        }
    }

    for (int i = 0;i<mkp.getConfig()._nCameras;i++){
        memcpy(_buf[i], mkp[i].getRGBColorPreviewScale().data, bytes);
    }
    //Draw each area
    if(_initialized) {
        for (int i = 0;i<_cameras;i++){
             gtk_widget_queue_draw(drawingAreas[i]);
        }
    }
}

gboolean KFRDisplay::drawCallback(GtkWidget *widget, cairo_t *cr, gpointer data) {
    //Scale the image based on the number of cameras to be displayed
    KFRDisplay *display = (KFRDisplay *)data;
    /*
    int _cameras = 4;
    if(_cameras == 8){
        cairo_scale(cr,.125,.125); // 8 cameras
    }
    else if (_cameras == 6){
        cairo_scale(cr,.18,.18); // 6 cameras
    }
    else if (_cameras == 4){
        cairo_scale(cr,.4,.4); // 4 cameras
    }
    */
    
    return display->doDraw(cr);
}

gboolean KFRDisplay::doDraw(cairo_t *cr) {
    //draw each camera
    static int cameraDisplayCount = 0;
    if (cameraDisplayCount == _cameras)
        cameraDisplayCount = 0;
    if(_buf != NULL) {
        GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(
            (guint8*)(_buf[cameraDisplayCount]),
            GDK_COLORSPACE_RGB,
            false,
            8,
            _cols,
            _rows,
            (int)3 * _cols, NULL, NULL);
        gdk_cairo_set_source_pixbuf(cr, pixbuf, 0, 0);
        cairo_paint(cr);
        // free(_buf[cameraDisplayCount]);
        cameraDisplayCount++;
    }
}