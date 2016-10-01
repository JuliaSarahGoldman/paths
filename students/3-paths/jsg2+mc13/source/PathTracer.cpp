/** \file RayTracer.cpp */
// Julia Goldman + Matheus de Carvalho Souza
#include "PathTracer.h"

PathTracer::PathTracer(const shared_ptr<Scene>& scene) {};

void PathTracer::traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image) {
   
    
    
    //Get the Array of lights
    const Array<shared_ptr<Light>> lights(m_scene->lightingEnvironment().lightArray);
    //Make the tree
    Array<shared_ptr<Surface>> surfs;
    m_scene->onPose(surfs);
    m_triangles->setContents(surfs);

    const int& numPixels(image->height()*image->width());
    Array<Radiance3> biradianceBuffer;
    biradianceBuffer.resize(numPixels);

    Array<Color3> modulationBuffer;
    for (int i(0); i < numPixels; ++i){
        modulationBuffer.append(Color3(1/m_maxScatters));
    }

    Array<Ray> rayBuffer;
    Array<shared_ptr<Surfel>> surfelBuffer;   
    Array<Ray> shadowRayBuffer;
    Array<bool> lightShadowedBuffer;

    for (int i(0); i < m_numPaths; ++i){
        generatePrimaryRays(rayBuffer);
        
        for(int d(0); d < m_maxScatters; ++d){
            m_triangles->intersectRays(rayBuffer, surfelBuffer);   
            for(int j(0); j < numPixels; ++j){computeShadowRays(shadowRayBuffer, surfelBuffer, biradianceBuffer, lights, j); }    
            for(int j(0); j < numPixels; ++j){testVisibility(shadowRayBuffer, lightShadowedBuffer, j); }    //casts shadow rays
            // Increments image pixels -> by emissive and direct light, with importance sampling
            for(int j(0); j < numPixels; ++j){writeToImage(shadowRayBuffer, biradianceBuffer, surfelBuffer, modulationBuffer, j);}
            //Only if not last iteration.
            for(int j(0); j < numPixels; ++j){generateRecursiveRays(rayBuffer, surfelBuffer, j);}    //set up for next loop
           // for(int j(0); j < numPixels; ++j){updateModulation(modulationBuffer, rayBuffer, j);}    
              
        }
    }

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

void PathTracer::generatePrimaryRays() {};

void PathTracer::addEmissiveTerms() {};

void PathTracer::computeShadowRays() {};

void PathTracer::castShadowRays() {};

void PathTracer::shadeHitpoints() {};

void PathTracer::scatter() {};

