#ifndef RAYCASTER_H
#define RAYCASTER_H

#include "cv.h"
#include "highgui.h"

typedef struct raycaster {
    IplImage* image;
} raycaster;

raycaster* raycaster_init(const char* image);
double raycaster_cast(raycaster* caster, int xx, int yy, double theta);
int raycaster_test_pixel(raycaster* caster, int xx, int yy);
void raycaster_free(raycaster*);

#endif
