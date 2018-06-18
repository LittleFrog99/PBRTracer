#ifndef API_H
#define API_H

#include "utilities.h"

class ParamSet;

namespace API {
    void init(const Options &opt);
    void cleanup();
    void identity();
    void translate(Float dx, Float dy, Float dz);
    void rotate(Float angle, Float ax, Float ay, Float az);
    void scale(Float sx, Float sy, Float sz);
    void lookAt(Float ex, Float ey, Float ez, Float lx, Float ly, Float lz, Float ux, Float uy, Float uz);
    void concatTransform(Float transform[16]);
    void transform(Float transform[16]);
    void coordinateSystem(const string &);
    void coordSysTransform(const string &);
    void activeTransformAll();
    void activeTransformEndTime();
    void activeTransformStartTime();
    void transformTimes(Float start, Float end);
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

#endif // API_H
