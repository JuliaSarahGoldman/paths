/** \file RayTracer.cpp */
// Julia Goldman + Matheus de Carvalho Souza
#include "PathTracer.h"

PathTracer::PathTracer(const shared_ptr<Scene>& scene) {};

void PathTracer::traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image) {};

Radiance3 PathTracer::L_in(const Point3& X, const Vector3& w_in, int pathDepth, const TriTree& triArray) const {
    // Find the first intersection 
    const shared_ptr<Surfel>& surfel(triArray.intersectRay(Ray(X, w_in)));

    // Compute the light leaving Y, which is the same as
        // the light entering X when the medium is non-absorptive
    return L_out(surfel,-w_in, 1, triArray);

};
Radiance3 PathTracer::L_out(const shared_ptr<Surfel>& surfel, const Vector3& w_out, int pathDepth, const TriTree& triArray) const {
    if (notNull(surfel)){
        const Vector3& n(surfel->shadingNormal);
        const Vector3& w_in(Vector3::cosHemiRandom(n,Random::threadCommon()));
        const Point3& X(surfel->position*EPSILON* sign((w_in).dot(n)));
        
    } else {
        return backgroundRadiance(w_out);
    }

};

bool  PathTracer::isVisible(const Point3& X, const Point3& Y) const {};

Radiance3  PathTracer::backgroundRadiance(const Vector3& direction) const {};


void PathTracer::buildTree() {};

void PathTracer::resetImage() {};

void PathTracer::generatePrimaryRays() {};

void PathTracer::addEmissiveTerms() {};

void PathTracer::computeShadowRays() {};

void PathTracer::castShadowRays() {};

void PathTracer::shadeHitpoints() {};

void PathTracer::scatter() {};

