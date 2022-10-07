#ifndef KFR_DISPLAY_H
#define KFR_DISPLAY_H

#include "MKPRecipient.h"
#include <gtk/gtk.h>
class MKConfig;

class KFRDisplay : public MKPRecipient {
public:
    //Display rows & columns
    KFRDisplay(int rows, int cols, int nCameras);
    ~KFRDisplay();
    void receiveFrame(MultiKinectPacket &mkp);
    void buildWidgets(GtkWidget *container);
    static gboolean drawCallback (GtkWidget *widget, cairo_t *cr, gpointer data);
    gboolean doDraw(cairo_t *cr);
protected:
    int _rows, _cols, _cameras, _gridRows;
    bool _initialized;
    //cv::Mat _colorMat;
    GtkWidget *_darea;

    GtkWidget *drawingAreas[8];
    GtkGrid *_gtkGrid;
    //GdkPixbuf *_pixbuf;
    unsigned char **_buf;
};

#endif