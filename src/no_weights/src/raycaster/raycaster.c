
#include <stdio.h>
#include <assert.h>
#include <math.h>

#include "raycaster.h"

const double LASER_MAX   =  5.0;
const double IMAGE_SCALE = 20.0;

raycaster* 
raycaster_init(const char* image)
{
    raycaster* caster = (raycaster*) malloc(sizeof(caster));
    caster->image = cvLoadImage(image, CV_LOAD_IMAGE_COLOR);
    return caster;
}

int
raycaster_test_pixel(raycaster* caster, int xx, int yy)
{
    IplImage* image = caster->image;

    if (xx < 0 || xx >= image->width)
        return 0;
    if (yy < 0 || yy >= image->height)
        return 0;

    int idx = yy*image->widthStep + xx*image->nChannels;
    unsigned char vv = image->imageData[idx];

    return vv > 100;
}

double 
raycaster_cast(raycaster* caster, int xx, int yy, double theta)
{
    IplImage* image = caster->image;

    assert(image->depth == IPL_DEPTH_8U);

    double ray_cos = cos(theta);
    double ray_sin = sin(theta);

    int NN = floor(LASER_MAX * IMAGE_SCALE) - 1;

    for (int ii = 0; ii < NN; ++ii) {
        int dx = floor(ii * ray_cos);
        int dy = floor(ii * ray_sin);

        if (raycaster_test_pixel(caster, xx + dx, yy + dy))
            return ii / IMAGE_SCALE;
    }

    return LASER_MAX;
}

void 
raycaster_free(raycaster* caster)
{
    cvReleaseImage(&(caster->image));
    free(caster);
}
