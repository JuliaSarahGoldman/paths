/** \file RayTracer.cpp */
// Julia Goldman + Matheus de Carvalho Souza
#include "PathTracer.h"

PathTracer::PathTracer(const shared_ptr<Scene>& scene) :
    m_scene(scene){
};

void PathTracer::traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image) {
   
    
    
    //Get the Array of lights
    Array<shared_ptr<Light>> lights;
    m_scene->getTypedEntityArray(lights);
    //((m_scene->lightingEnvironment()).lightArray());
    //Make the tree
    Array<shared_ptr<Surface>> surfs;
    m_scene->onPose(surfs);
    m_triangles->setContents(surfs);

    const int width(image->width());
    m_width = width;
    const int height(image->height());
    const int& numPixels(width*height);
  
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
        generatePrimaryRays(rayBuffer, cam, width, height, i);
        
        for(int d(0); d < m_maxScatters; ++d){
            m_triangles->intersectRays(rayBuffer, surfelBuffer);   
            for(int j(0); j < numPixels; ++j){computeShadowRays(shadowRayBuffer,modulationBuffer, surfelBuffer, biradianceBuffer, lights, j); }    
            for(int j(0); j < numPixels; ++j){testVisibility(shadowRayBuffer, surfelBuffer, lightShadowedBuffer, j); }    //casts shadow rays
            // Increments image pixels -> by emissive and direct light, with importance sampling
            for(int j(0); j < numPixels; ++j){writeToImage(shadowRayBuffer, biradianceBuffer, surfelBuffer, modulationBuffer, j, image, rayBuffer, lights);}
            //Only if not last iteration.
            if (d != m_maxScatters-1){
                for(int j(0); j < numPixels; ++j){generateRecursiveRays(rayBuffer, surfelBuffer,modulationBuffer, j);}    //set up for next loop
            }
              
        }
    }

};

void PathTracer::writeToImage(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Color3>& modulationBuffer, const int j, const shared_ptr<Image>& image, const Array<Ray>& rayBuffer, const Array<shared_ptr<Light>>& lights) const{
    Color3 current;
    image->get(Point2int32(j%image->width(), j/image->width()), current);
    current += L_out(surfelBuffer[j], -rayBuffer[j].direction(), lights, rayBuffer[j].origin(), j);
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

Radiance3 PathTracer::L_out(const shared_ptr<Surfel>& surfel, const Vector3& w_out, const Array<shared_ptr<Light>>& lights, const Point3& origin, const int j) const {
    if (notNull(surfel)) {
        Radiance3 L(surfel->emittedRadiance(w_out)); // Emitted

        const Vector3& n(surfel->shadingNormal);
        //const Vector3& w_in(Vector3::cosHemiRandom(n, Random::threadCommon()));
        const Point3& X(surfel->position*.0001* sign((w_out).dot(n)));

        //Will be replaced with importance sampling
        for (int i = 0; i < lights.size(); ++i) {
            const shared_ptr<Light>& light(lights[i]);
            const Point3& Y = light->position().xyz();

            if (!light->castsShadows() || isVisible(X, Y)) {
                const Vector3 w_in = (Y - X).direction();
                Radiance3 Bi = light->biradiance(X);

                const Radiance3 f = surfel->finiteScatteringDensity(w_in, -w_out);
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
        return backgroundRadiance(w_out, origin, j);
    }
};

bool  PathTracer::isVisible(const Point3& X, const Point3& Y) const {
    return true;
};

Radiance3  PathTracer::backgroundRadiance(const Vector3& direction, const Point3& origin, const int j) const {
    Point2int32 p(origin.x,origin.y);
    Radiance3 picCol(0.0f,0.75f,1.0f);
    float dis(sqrt(abs((p.x-(j%m_width))*(p.x-(j%m_width))+(p.y-(j/m_width))*(p.y-(j/m_width)))));
    picCol *= (dis>1) ? abs( 1/dis * 100 * direction.dot(Vector3(0.04f,0.5f,0.03f))) : abs(dis * direction.dot(Vector3(0.04f,0.5f,0.03f)) * 100);
    return picCol;
};

void PathTracer::generatePrimaryRays(Array<Ray>& rayBuffer,const shared_ptr<Camera>& cam, int width, int height, int j) const {
    Rect2D plane(Rect2D(Vector2(width, height)));
    
    for(Point2int32 pixel; pixel.y< height; ++pixel.y){
        for(pixel.x = 0; pixel.x < width; ++pixel.x){
            Ray ray(cam->worldRay(pixel.x, pixel.y, plane));
            rayBuffer[j] = ray.bumpedRay(0.0001);
        }
    }
};

void PathTracer::testVisibility(const Array<Ray>& shadowRayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<bool>& lightShadowedBuffer, const int j) const {
  shared_ptr<Surfel> other(m_triangles->intersectRay(shadowRayBuffer[j])); 
  Point3 toCheck(surfelBuffer[j]->position - other->position);
  lightShadowedBuffer[j]= ((abs(toCheck.x) <= 0.0001) && (abs(toCheck.y) <= 0.0001) && (abs(toCheck.z) <= 0.0001));
}; 



void PathTracer::computeShadowRays(Array<Ray>& shadowRayBuffer,  Array<Color3>& modulationBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Light>>& lights, const int j) const {
     const Point3& X(surfelBuffer[j]->position);
     int i(Random::threadCommon().uniform(0, lights.size()));
     const Point3& Y((lights[i]->position()).xyz());
     const Vector3& wo_y((X-Y).unit()); 
     const Point3& origin(Y*0.0001* sign((wo_y).dot(-surfelBuffer[j]->shadingNormal))); 
     shadowRayBuffer[j] = Ray(origin, wo_y);
     modulationBuffer[j] *= 1/lights.size();
};

PathTracer::~PathTracer(){};


