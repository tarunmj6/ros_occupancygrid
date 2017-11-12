%module raycaster 

%{
#include "raycaster.h"
%}

raycaster* raycaster_init(const char* image);
double raycaster_cast(raycaster* caster, int xx, int yy, double theta);
int raycaster_test_pixel(raycaster* caster, int xx, int yy);
void raycaster_free(raycaster*);
