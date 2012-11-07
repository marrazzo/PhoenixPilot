#ifndef VIS_LIB_H
#define VIS_LIB_H


extern "C" {
    void * vislib_initialize();
    void vislib_start(void *g);
    void vislib_shutdown(void *g);
}

#endif /* VIS_LIB_H */