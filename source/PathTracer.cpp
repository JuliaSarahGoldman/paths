/** \file RayTracer.cpp */
/*
  Julia Goldman
  Matheus de Carvalho Souza
  Jose Rivas-Garcia
  Youle Chen
  */
#include "PathTracer.h"

PathTracer::PathTracer(const shared_ptr<Scene>& scene, int numPaths, int maxScatters) :
    m_scene(scene), m_numPaths(numPaths), m_maxScatters(maxScatters) {
};

void PathTracer::traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image) {
    //Get the Array of lights
    Array<shared_ptr<Light>> lights;
    m_scene->getTypedEntityArray(lights);
    
    //Make the tree
    m_triangles.setContents(m_scene);

    const int width(image->width());
    m_width = width;
    const int height(image->height());
    const int& numPixels(width*height);

    Array<Radiance3> biradianceBuffer;
    biradianceBuffer.resize(numPixels);

    Array<Color3> modulationBuffer;
    modulationBuffer.resize(numPixels);
    

    Array<Ray> rayBuffer;
    rayBuffer.resize(numPixels);

    Array<shared_ptr<Surfel>> surfelBuffer;
    surfelBuffer.resize(numPixels);

    Array<Ray> shadowRayBuffer;
    shadowRayBuffer.resize(numPixels);

    Array<bool> lightShadowedBuffer;
    lightShadowedBuffer.resize(numPixels);

    for (int i(0); i < m_numPaths; ++i) {

        debugPrintf("\npath: %d\n", i);


        modulationBuffer.setAll(Color3(1.0f / float(m_numPaths)));

        
        generatePrimaryRays(rayBuffer, cam, width, height);


        for (int d(0); d < m_maxScatters; ++d) {
            m_triangles.intersectRays(rayBuffer, surfelBuffer);
            Thread::runConcurrently(0, numPixels, [&](int j) {computeShadowRays(shadowRayBuffer, modulationBuffer, surfelBuffer, biradianceBuffer, lights, j); });
            m_triangles.intersectRays(shadowRayBuffer, lightShadowedBuffer);
            Thread::runConcurrently(0, numPixels, [&](int j) {writeToImage(shadowRayBuffer, biradianceBuffer, surfelBuffer, modulationBuffer, j, image, rayBuffer, lightShadowedBuffer); });

            if (d != m_maxScatters - 1) {
                //set up for next loop
                Thread::runConcurrently(0, numPixels, [&](int j) {generateRecursiveRays(rayBuffer, surfelBuffer, modulationBuffer, j); });
            }

        }
    }

};

void PathTracer::generatePrimaryRays(Array<Ray>& rayBuffer, const shared_ptr<Camera>& cam, int width, int height) const {
    Rect2D plane(Rect2D(Vector2(width, height)));
    Thread::runConcurrently(Point2int32(0,0),Point2int32(width, height), [&](Point2int32 pixel) {
        Ray ray(cam->worldRay(pixel.x+0.5f, pixel.y+0.5f, plane));
        rayBuffer[pixel.x+pixel.y*width] = ray; 
    });
  };

void PathTracer::writeToImage(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Color3>& modulationBuffer, const int j, const shared_ptr<Image>& image, const Array<Ray>& rayBuffer, const Array<bool>& lightShadowedBuffer) const {
    Color3 current;
    image->get(Point2int32(j%image->width(), j / image->width()), current); 
    //Should apply the modulation buffer here, but just turns black
    current += modulationBuffer[j]*L_out(surfelBuffer[j], shadowRayBuffer[j].origin(),-rayBuffer[j].direction(), biradianceBuffer[j],lightShadowedBuffer[j], rayBuffer[j].origin(), j);
    image->set(Point2int32(j%image->width(), j / image->width()), current);
};

void PathTracer::generateRecursiveRays(Array<Ray>& rayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Color3>& modulationBuffer, const int j) const {
    Vector3 win;
    Color3 weight;
    if (notNull(surfelBuffer[j])){
        surfelBuffer[j]->scatter(PathDirection::EYE_TO_SOURCE, -rayBuffer[j].direction(), false, Random::threadCommon(), weight, win);
        rayBuffer[j] = Ray(surfelBuffer[j]->position, win).bumpedRay(FLT_EPSILON);
        modulationBuffer[j] *= weight;
    }
    else{
        rayBuffer[j] = Ray::fromOriginAndDirection(Point3(1000, 1000, 1000), Vector3(1,1,1).unit(), 0.0001, 0.0002);
        modulationBuffer[j] *=0;
    }
};


Radiance3 PathTracer::L_out(const shared_ptr<Surfel>& surfel, const Point3& Y, const Vector3& w_out, const Radiance3& bi, bool notVisible, const Point3& origin, const int j) const {
    if (notNull(surfel)) {
        Radiance3 L(surfel->emittedRadiance(w_out)); 

        const Vector3& n(surfel->shadingNormal);
        const Point3& X(surfel->position);

            if (!notVisible) {
                const Vector3 w_in = (Y - X).direction();

                const Radiance3 f = surfel->finiteScatteringDensity(w_in, w_out);
                L += bi * f * abs(w_in.dot(n));
            }

        return L;

    }
    else {
        return backgroundRadiance(w_out, origin, j);
    }
};



Radiance3  PathTracer::backgroundRadiance(const Vector3& direction, const Point3& origin, const int j) const {
    Point2int32 p(origin.x, origin.y);
    Radiance3 picCol(0.0f, 0.75f, 1.0f);
    float dis(sqrt(abs((p.x - (j%m_width))*(p.x - (j%m_width)) + (p.y - (j / m_width))*(p.y - (j / m_width)))));
    picCol *= (dis > 1) ? abs(1 / dis * 100 * direction.dot(Vector3(0.04f, 0.5f, 0.03f))) : abs(dis * direction.dot(Vector3(0.04f, 0.5f, 0.03f)) * 100);
    return picCol;
};





void PathTracer::computeShadowRays(Array<Ray>& shadowRayBuffer, Array<Color3>& modulationBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Light>>& lights, const int j) const {
    if (!isNull(surfelBuffer[j])) {
       
        
        const Point3& X(surfelBuffer[j]->position);
        float total = 0.0f;
        for(int i(0); i< lights.size(); ++i){
            total += lights[i]->biradiance(X).sum();
        }

        float counter = Random::threadCommon().uniform(0,total); 
        int index(0);
        int i(0);

        while(counter > 0 && i <lights.size()){
            float lRadiance(lights[i]->biradiance(X).sum());
            counter -= lRadiance;
            if(lRadiance > 0.0){
                index = i;
            }
            ++i;
        }
        
        float weight((float)lights[index]->biradiance(X).sum()/(float)total); 
        
        biradianceBuffer[j] = (weight > 0)? lights[index]->biradiance(X)/weight : lights[index]->biradiance(X); 

        const Point3& Y((lights[index]->position()).xyz());
        const Vector3& wo_y((X - Y).unit());
        float  maxDistance((X - Y).length() - 0.0001);
        shadowRayBuffer[j] = Ray::fromOriginAndDirection(Y, wo_y, 0.0001, maxDistance);
    }
    else {
        shadowRayBuffer[j] = Ray::fromOriginAndDirection(Point3(1000, 1000, 1000), Vector3(1,1,1).unit(), 0.0001, 0.0002);
    }
};

PathTracer::~PathTracer() {};


