#ifndef API_H
#define API_H

#include "paramset.h"
#include "utilities.h"


class API {
public:
    static void init(const Options &opt);
    static void cleanup();
    static void identity();
    static void translate(Float dx, Float dy, Float dz);
    static void rotate(Float angle, Float ax, Float ay, Float az);
    static void scale(Float sx, Float sy, Float sz);
    static void lookAt(Float ex, Float ey, Float ez, Float lx, Float ly, Float lz, Float ux, Float uy, Float uz);
    static void concatTransform(Float transform[16]);
    static void transform(Float transform[16]);
    static void coordinateSystem(const string &);
    static void coordSysTransform(const string &);
    static void activeTransformAll();
    static void activeTransformEndTime();
    static void activeTransformStartTime();
    static void transformTimes(Float start, Float end);
    static void pixelFilter(const string &name, const ParamSet &params);
    static void film(const string &type, const ParamSet &params);
    static void sampler(const string &name, const ParamSet &params);
    static void accelerator(const string &name, const ParamSet &params);
    static void integrator(const string &name, const ParamSet &params);
    static void camera(const string &, const ParamSet &cameraParams);
    static void makeNamedMedium(const string &name, const ParamSet &params);
    static void mediumInterface(const string &insideName, const string &outsideName);
    static void worldBegin();
    static void attributeBegin();
    static void attributeEnd();
    static void transformBegin();
    static void transformEnd();
    static void texture(const string &name, const string &type, const string &texname, const ParamSet &params);
    static void material(const string &name, const ParamSet &params);
    static void makeNamedMaterial(const string &name, const ParamSet &params);
    static void namedMaterial(const string &name);
    static void lightSource(const string &name, const ParamSet &params);
    static void areaLightSource(const string &name, const ParamSet &params);
    static void shape(const string &name, const ParamSet &params);
    static void reverseOrientation();
    static void objectBegin(const string &name);
    static void objectEnd();
    static void objectInstance(const string &name);
    static void worldEnd();

    static void parseFile(string filename);
    static void parseString(string str);
};

#endif // API_H
