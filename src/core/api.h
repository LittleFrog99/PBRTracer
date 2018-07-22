#ifndef CORE_API
#define CORE_API

#include "utilities.h"

namespace API {
    void init(const Options &opt);
    void cleanup();
    void identity();
    void translate(float dx, float dy, float dz);
    void rotate(float angle, float ax, float ay, float az);
    void scale(float sx, float sy, float sz);
    void lookAt(float ex, float ey, float ez, float lx, float ly, float lz, float ux, float uy, float uz);
    void concatTransform(float transform[16]);
    void transform(float transform[16]);
    void coordinateSystem(const string &);
    void coordSysTransform(const string &);
    void activeTransformAll();
    void activeTransformEndTime();
    void activeTransformStartTime();
    void transformTimes(float start, float end);
    void pixelFilter(const string &name, const ParamSet &params);
    void film(const string &type, const ParamSet &params);
    void sampler(const string &name, const ParamSet &params);
    void accelerator(const string &name, const ParamSet &params);
    void integrator(const string &name, const ParamSet &params);
    void camera(const string &, const ParamSet &cameraParams);
    void makeNamedMedium(const string &name, const ParamSet &params);
    void mediumInterface(const string &insideName, const string &outsideName);
    void worldBegin();
    void attributeBegin();
    void attributeEnd();
    void transformBegin();
    void transformEnd();
    void texture(const string &name, const string &type, const string &texname, const ParamSet &params);
    void material(const string &name, const ParamSet &params);
    void makeNamedMaterial(const string &name, const ParamSet &params);
    void namedMaterial(const string &name);
    void lightSource(const string &name, const ParamSet &params);
    void areaLightSource(const string &name, const ParamSet &params);
    void shape(const string &name, const ParamSet &params);
    void reverseOrientation();
    void objectBegin(const string &name);
    void objectEnd();
    void objectInstance(const string &name);
    void worldEnd();

    void parseFile(string filename);
    void parseString(string str);
};

#endif // CORE_API
