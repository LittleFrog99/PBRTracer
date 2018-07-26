#include "material.h"
#include "core/interaction.h"
#include "core/texture.h"

void Material::bump(const shared_ptr<Texture<float>> &d, SurfaceInteraction *si) {
    // Compute offset positions and evaluate displacement texture
    auto siEv = *si;
    float du = 0.5f * (abs(si->dudx + abs(si->dudy)));
    if (du == 0) du = 0.005f;
    siEv.p = si->p + du * si->shading.dpdu;
    siEv.uv = si->uv + Vector2f(du, 0);
    siEv.n = normalize(Normal3f(cross(si->shading.dpdu, si->shading.dpdv)) + du * si->dndu);
    float uDisplace = d->evaluate(siEv);
    float dv = 0.5f * (abs(si->dvdx + abs(si->dvdy)));
    if (dv == 0) dv = 0.005f;
    siEv.p = si->p + dv * si->shading.dpdv;
    siEv.uv = si->uv + Vector2f(0, dv);
    siEv.n = normalize(Normal3f(cross(si->shading.dpdu, si->shading.dpdv)) + dv * si->dndv);
    float vDisplace = d->evaluate(siEv);
    float displace = d->evaluate(*si);

    // Compute bump-mapped differential geometry
    Vector3f dpdu = si->shading.dpdu + (uDisplace - displace) / du * Vector3f(si->shading.n) +
                    displace * Vector3f(si->shading.dndu);
    Vector3f dpdv = si->shading.dpdv + (vDisplace - displace) / dv * Vector3f(si->shading.n) +
                    displace * Vector3f(si->shading.dndv);
    si->setShadingGeometry(dpdu, dpdv, si->shading.dndu, si->shading.dndv, false);
}
