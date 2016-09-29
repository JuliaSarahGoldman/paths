/** \file RayTracer.cpp */
// Julia Goldman + Matheus de Carvalho Souza
#include "PathTracer.h"

PathTracer::PathTracer(const shared_ptr<Scene>& scene) {};

void PathTracer::traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image) {
    Array<Color3> modulationBuffer(Array<Color3>());
    Array<Ray> rayBuffer;
    Array<shared_ptr<Surfel>> surfelBuffer;
    Array<Radiance3> biradianceBuffer;
    Array<Ray> shadowRayBuffer;
    Array<bool> lightShadowedBuffer;
};

Radiance3 PathTracer::L_in(const Point3& X, const Vector3& w_in, int pathDepth, const TriTree& triArray) const {
    // Find the first intersection 
    const shared_ptr<Surfel>& surfel(triArray.intersectRay(Ray(X, w_in)));

    // Compute the light leaving Y, which is the same as
        // the light entering X when the medium is non-absorptive
    return L_out(surfel, -w_in, pathDepth, triArray);

};
Radiance3 PathTracer::L_out(const shared_ptr<Surfel>& surfel, const Vector3& w_out, int pathDepth, const TriTree& triArray) const {
    if (notNull(surfel)) {
        const Radiance3& L(surfel->emittedRadiance); // Emitted

        const Vector3& n(surfel->shadingNormal);
        const Vector3& w_in(Vector3::cosHemiRandom(n, Random::threadCommon()));
        const Point3& X(surfel->position*EPSILON* sign((w_in).dot(n)));


        Array<shared_ptr<Light>> lights(m_scene->lightingEnvironment().lightArray);
        for (int i = 0; i < lights.size(); ++i) {
            const shared_ptr<Light>& light(lights[i]);
            const Point3& Y = light->position().xyz();

            if (!light->castsShadows() || isVisible(X, Y)) {
                const Vector3& w_in = (Y - X).direction();
                Biradiance3& Bi = light->biradiance(X);

                const Color3& f = surfel->finiteScatteringDensity(w_in, -w_out);
                L += Bi * f * abs(w_in.dot(n));
            }
        }

        Radiance3& indirectLight(Radiance3(0, 0, 0));
        if (m_maxScatters < pathDepth) {
            const Vector3& w_i(Vector3::cosHemiRandom(n, Random::threadCommon()));
            const Color3& f(surfel->finiteScatteringDensity(w_i, -w_out));
            indirectLight += L_in(X, w_i, pathDepth + 1, triArray)*f;
        }

        return L + indirectLight / m_maxScatters;

    }
    else {
        return backgroundRadiance(w_out);
    }
};

bool  PathTracer::isVisible(const Point3& X, const Point3& Y) const {};

Radiance3  PathTracer::backgroundRadiance(const Vector3& direction) const {};


void PathTracer::buildTree() {};

void PathTracer::resetImage() {};

void PathTracer::generatePrimaryRays(const Array<Ray>& rayBuffer, const Array<Color3>& modulationBuffer) const {};

void PathTracer::addEmissiveTerms(const Array<Radiance3>& biradianceBuffer, const Array<Color3>& modulationBuffer) const{};

void PathTracer::computeShadowRays(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer) const {};

void PathTracer::castShadowRays(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer) {};

void PathTracer::shadeHitpoints(const Array<Ray>& shadowRayBuffer, const Array<bool>& lightShadowedBuffer) const {};

void PathTracer::scatter() {};

