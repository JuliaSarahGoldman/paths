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
            for(int j(0); j < numPixels; ++j){writeToImage(shadowRayBuffer, biradianceBuffer, surfelBuffer, modulationBuffer, j, image, rayBuffer, lights);}
            //Only if not last iteration.
            for(int j(0); j < numPixels; ++j){generateRecursiveRays(rayBuffer, surfelBuffer, j);}    //set up for next loop
           // for(int j(0); j < numPixels; ++j){updateModulation(modulationBuffer, rayBuffer, j);}    
              
        }
    }

};

void PathTracer::writeToImage(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Color3>& modulationBuffer, const int j, const shared_ptr<Image>& image, const Array<Ray>& rayBuffer, const Array<shared_ptr<Light>>& lights) const{
    Color3 current;
    image->get(Point2int32(j%image->width(), j/image->width()), current);
    current += L_out(surfelBuffer[j], -rayBuffer[j].direction(), lights);
    image->set(Point2int32(j%image->width(), j/image->width()), current);
};

void PathTracer::generateRecursiveRays(Array<Ray>& rayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Color3>& modulationBuffer, const int j) const{
    Vector3 wo;
    Color3 weight;
    surfelBuffer[j]->scatter(PathDirection::EYE_TO_SOURCE, rayBuffer[j].direction(), true, Random::threadCommon(), weight, wo);
    rayBuffer[j] = Ray(surfelBuffer[j]->position, wo);
    modulationBuffer[j] *= weight;
};

//Don't use anymore
Radiance3 PathTracer::L_in(const Point3& X, const Vector3& w_in, int pathDepth, const TriTree& triArray) const {
    // Find the first intersection 
    const shared_ptr<Surfel>& surfel(triArray.intersectRay(Ray(X, w_in)));

    // Compute the light leaving Y, which is the same as
        // the light entering X when the medium is non-absorptive
    return Radiance3();//L_out(surfel, -w_in, pathDepth, triArray);

};

Radiance3 PathTracer::L_out(const shared_ptr<Surfel>& surfel, const Vector3& w_out, const Array<shared_ptr<Light>>& lights) const {
    if (notNull(surfel)) {
        const Radiance3& L(surfel->emittedRadiance); // Emitted

        const Vector3& n(surfel->shadingNormal);
        //const Vector3& w_in(Vector3::cosHemiRandom(n, Random::threadCommon()));
        const Point3& X(surfel->position*.0001* sign((w_out).dot(n)));

        //Will be replaced with importance sampling
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

        /*Radiance3& indirectLight(Radiance3(0, 0, 0));
        if (m_maxScatters < pathDepth) {
            const Vector3& w_i(Vector3::cosHemiRandom(n, Random::threadCommon()));
            const Color3& f(surfel->finiteScatteringDensity(w_i, -w_out));
            indirectLight += L_in(X, w_i, pathDepth + 1, triArray)*f;
        }*/

        return L;

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

