/** \file RayTracer.cpp */
// Julia Goldman + Matheus de Carvalho Souza
#include "PathTracer.h"

PathTracer::PathTracer(const shared_ptr<Scene>& scene, int numPaths, int maxScatters) :
    m_scene(scene), m_numPaths(numPaths), m_maxScatters(maxScatters) {
};

void PathTracer::traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image) {



    //Get the Array of lights
    Array<shared_ptr<Light>> lights;
    m_scene->getTypedEntityArray(lights);
    //((m_scene->lightingEnvironment()).lightArray());
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
    for (int i(0); i < numPixels; ++i) {
        if (m_maxScatters == 0) {
            modulationBuffer[i] = Color3(1, 1, 1);
        }
        else {
            modulationBuffer[i] = Color3(1 / m_maxScatters);
        }
    }

    Array<Ray> rayBuffer;
    rayBuffer.resize(numPixels);
    Array<shared_ptr<Surfel>> surfelBuffer;
    surfelBuffer.resize(numPixels);
    Array<Ray> shadowRayBuffer;
    shadowRayBuffer.resize(numPixels);
    Array<bool> lightShadowedBuffer;
    lightShadowedBuffer.resize(numPixels);

    for (int i(0); i < m_numPaths; ++i) {
        generatePrimaryRays(rayBuffer, cam, width, height, i);

        for (int d(0); d < m_maxScatters; ++d) {
            m_triangles.intersectRays(rayBuffer, surfelBuffer);
            Thread::runConcurrently(0, numPixels, [&](int j) {computeShadowRays(shadowRayBuffer, modulationBuffer, surfelBuffer, biradianceBuffer, lights, j); });
          //  Thread::runConcurrently(0, numPixels, [&](int j) {testVisibility(shadowRayBuffer, surfelBuffer, lightShadowedBuffer, j); });
            m_triangles.intersectRays(shadowRayBuffer, lightShadowedBuffer);
            Thread::runConcurrently(0, numPixels, [&](int j) {writeToImage(shadowRayBuffer, biradianceBuffer, surfelBuffer, modulationBuffer, j, image, rayBuffer, lightShadowedBuffer); });

            if (d != m_maxScatters - 1) {
                //set up for next loop
                Thread::runConcurrently(0, numPixels, [&](int j) {generateRecursiveRays(rayBuffer, surfelBuffer, modulationBuffer, j); });
            }

        }
    }

};

void PathTracer::writeToImage(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Color3>& modulationBuffer, const int j, const shared_ptr<Image>& image, const Array<Ray>& rayBuffer, const Array<bool>& lightShadowedBuffer) const {
    Color3 current;
    image->get(Point2int32(j%image->width(), j / image->width()), current);
    current += L_out(surfelBuffer[j], shadowRayBuffer[j].origin(),-rayBuffer[j].direction(), biradianceBuffer[j],lightShadowedBuffer[j], rayBuffer[j].origin(), j);
    image->set(Point2int32(j%image->width(), j / image->width()), current);
};

void PathTracer::generateRecursiveRays(Array<Ray>& rayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Color3>& modulationBuffer, const int j) const {
    Vector3 wo;
    Color3 weight;
    surfelBuffer[j]->scatter(PathDirection::EYE_TO_SOURCE, rayBuffer[j].direction(), true, Random::threadCommon(), weight, wo);
    rayBuffer[j] = Ray(surfelBuffer[j]->position, wo);
    modulationBuffer[j] *= weight;
};

//Don't use anymore
/*
Radiance3 PathTracer::L_in(const Point3& X, const Vector3& w_in, int pathDepth, const TriTree& triArray) const {
    // Find the first intersection
    const shared_ptr<Surfel>& surfel(triArray.intersectRay(Ray(X, w_in)));

    // Compute the light leaving Y, which is the same as
        // the light entering X when the medium is non-absorptive
    return Radiance3();//L_out(surfel, -w_in, pathDepth, triArray);

};*/

Radiance3 PathTracer::L_out(const shared_ptr<Surfel>& surfel, const Point3& Y, const Vector3& w_out, const Radiance3& bi, bool notVisible, const Point3& origin, const int j) const {
    if (notNull(surfel)) {
        Radiance3 L(surfel->emittedRadiance(w_out)); // Emitted

        const Vector3& n(surfel->shadingNormal);
        //const Vector3& w_in(Vector3::cosHemiRandom(n, Random::threadCommon()));
        const Point3& X(surfel->position*.0001* sign((w_out).dot(n)));

        //Will be replaced with importance sampling
 //       for (int i = 0; i < lights.size(); ++i) {
 //           const shared_ptr<Light>& light(lights[i]);
 //           const Point3& Y = light->position().xyz();

            if (!notVisible) {
                const Vector3 w_in = (Y - X).direction();

                const Radiance3 f = surfel->finiteScatteringDensity(w_in, -w_out);
                L += bi * f * abs(w_in.dot(n));
            }
//        }

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
    Point2int32 p(origin.x, origin.y);
    Radiance3 picCol(0.0f, 0.75f, 1.0f);
    float dis(sqrt(abs((p.x - (j%m_width))*(p.x - (j%m_width)) + (p.y - (j / m_width))*(p.y - (j / m_width)))));
    picCol *= (dis > 1) ? abs(1 / dis * 100 * direction.dot(Vector3(0.04f, 0.5f, 0.03f))) : abs(dis * direction.dot(Vector3(0.04f, 0.5f, 0.03f)) * 100);
    return picCol;
};

void PathTracer::generatePrimaryRays(Array<Ray>& rayBuffer, const shared_ptr<Camera>& cam, int width, int height, int j) const {
    Rect2D plane(Rect2D(Vector2(width, height)));

    for (int y(0); y < height; ++y) {
        for (int x(0); x < width; ++x) {
            Ray ray(cam->worldRay(x, y, plane));
            rayBuffer[j] = ray;
        }
    }
};

void PathTracer::testVisibility(const Array<Ray>& shadowRayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<bool>& lightShadowedBuffer, const int j) const {
    if (!isNull(surfelBuffer[j])) {
        Point3 origin(shadowRayBuffer[j].origin());
        Vector3 direction(shadowRayBuffer[j].direction());
        Point3 X(surfelBuffer[j]->position);
        float  maxDistance((X - origin).length() - 0.0001);
        Ray shortRay(Ray::fromOriginAndDirection(origin, direction, 0.0001, maxDistance));
        lightShadowedBuffer[j] = m_triangles.intersectRay(shortRay) == surfelBuffer[j];
    } else { 
        
    };
    /* shared_ptr<Surfel> other(m_triangles->intersectRay(shadowRayBuffer[j]));
     Point3 toCheck(surfelBuffer[j]->position - other->position);
     lightShadowedBuffer[j]= ((abs(toCheck.x) <= 0.0001) && (abs(toCheck.y) <= 0.0001) && (abs(toCheck.z) <= 0.0001));
     */
};



void PathTracer::computeShadowRays(Array<Ray>& shadowRayBuffer, Array<Color3>& modulationBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Light>>& lights, const int j) const {
    if (!isNull(surfelBuffer[j])) {
        const Point3& X(surfelBuffer[j]->position);
        int i(Random::threadCommon().uniform(0, lights.size()));
        const Point3& Y((lights[i]->position()).xyz());
        const Vector3& wo_y((X - Y).unit());
        float  maxDistance((X - Y).length() - 0.0001);
       // const Point3& origin(Y*0.0001* sign((wo_y).dot(-surfelBuffer[j]->shadingNormal)));
        shadowRayBuffer[j] = Ray::fromOriginAndDirection(Y, wo_y, 0.0001, maxDistance);
        modulationBuffer[j] *= 1 / lights.size();
        biradianceBuffer[j] = lights[i]->biradiance(surfelBuffer[j]->position);
    }
    else {
        shadowRayBuffer[j] = Ray::fromOriginAndDirection(Point3(0, 0, 0), Vector3(1,1,1).unit(), 0.0001, 0.0002);
    }
};

PathTracer::~PathTracer() {};


