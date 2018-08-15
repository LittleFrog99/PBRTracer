#ifndef INTEGRATOR_PPB
#define INTEGRATOR_PPB

#include "core/integrator.h"

/* Progressive Photon Beams by Jarosz et al. 2011 */

class PPBIntegrator : public Integrator {
public:
    PPBIntegrator(shared_ptr<const Camera> &camera, int nIterations, int photonsPerIteration, int maxDepth,
                  float initialKernelSize, float tradeOff)
        : camera(camera), nIterations(nIterations), photonsPerIteration(photonsPerIteration), maxDepth(maxDepth),
          initialKernelSize(initialKernelSize), tradeOff(tradeOff) {}

    void render(const Scene &scene);

private:
    struct PhotonBeam;

    shared_ptr<const Camera> camera;
    const int nIterations;
    const int photonsPerIteration;
    const int maxDepth;
    const float initialKernelSize;
    const float tradeOff;
};

#endif // INTEGRATOR_PPB
