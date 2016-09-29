/** \file RayTracer.cpp */
#include "RayTracerM.h"
#include "Math.h"

RayTracer::RayTracer(const shared_ptr<Scene>& s, float e) :
    m_scene(s), m_epsilon(e) {};


void RayTracer::traceImage(const shared_ptr<Camera>& camera, const shared_ptr<Image>& img) {

    Array<shared_ptr<Surface>> surfs; // Pass Arrays down 
    m_scene->onPose(surfs);
    m_triTree = shared_ptr<TriTree>(new TriTree());
    m_triTree->setContents(surfs);
    const int width(img->width());
    const int height(img->height());

    Array<shared_ptr<Light>> lights;
    m_scene->getTypedEntityArray(lights);

    if (m_spheresOn) {
        Point3 center1(0, 0 , -1);
        Point3 center2(0, -3.5 , -1);
        Point3 center3(0, -4.25 , -1);
        shared_ptr<Sphere> sphere1(new Sphere(center1,1 ));
        shared_ptr<Sphere> sphere2(new Sphere(center2,0.5));
        shared_ptr<Sphere> sphere3(new Sphere(center3,0.25));
        m_mySpheres.append(sphere1);
        m_mySpheres.append(sphere2);
        m_mySpheres.append(sphere3);
        
  };

    const Vector2 dimensions((float)width, (float)height);
    const Rect2D& plane(dimensions);

    int maxRays(1);
    Thread::runConcurrently(Point2int32(0, 0), Point2int32(width, height), [&](Point2int32 pixel) {
        Ray ray(camera->worldRay(pixel.x, pixel.y, plane));
        Radiance3 sum(0, 0, 0);
        Thread::runConcurrently(Point2int32(0, 0), Point2int32(1, maxRays), [&](Point2int32 i) {
            ray = ray.bumpedRay(m_epsilon);
            sum += L_i(ray.origin(), ray.direction(), lights, 1);
        }, !m_runConcurrent);

        img->set(pixel.x, pixel.y, sum / (float)maxRays);
    }, !m_runConcurrent);

};

// Adapted from C++ Direct Illumination [_rn_dirctIllm] from http://graphicscodex.com 
Radiance3 RayTracer::L_i(const Point3& X, const Vector3& wi, const Array<shared_ptr<Light>>& lights, int depth) const {
    // Find the first intersection 
    const shared_ptr<Surfel>& surfelY(intersects(X, wi));

    if (notNull(surfelY)) {
        // Compute the light leaving Y, which is the same as
        // the light entering X when the medium is non-absorptive
        return L_o(surfelY, -wi, lights, depth);
    }
    else {
        return Radiance3(0, 0, 0);
    }
}

shared_ptr<Surfel> RayTracer::intersects(const Point3& P, const Vector3& w) const {
    TriTree::Hit hit;
    shared_ptr<UniversalSurfel> surfel(new UniversalSurfel);

    CPUVertexArray vertexArray(m_triTree->vertexArray());

    float t_t(0);
    float t_s(0);
    float b[3];
    Point3 V[3];
    Point3 X(0, 0, 0);
    Vector3 n(0, 0, 0);
    float dTri(inf());
    float dCir(inf());


    for (int i(0); i < m_triTree->size(); ++i) {
        Tri tri(m_triTree->operator[](i));
        V[0] = tri.position(vertexArray, 0);
        V[1] = tri.position(vertexArray, 1);
        V[2] = tri.position(vertexArray, 2);


        if (intersectTriangle(P, w, V, b, t_t)) {
            if (t_t < dTri) {
                dTri = t_t;
                hit.distance = t_t;
                hit.backface = (w.dot(tri.normal(vertexArray)) > 0);
                hit.triIndex = i;
                hit.u = b[0];
                hit.v = b[1];
            };
        };
    };

    if (m_spheresOn) {
        Point3 X(0, 0, 0);
        Vector3 n(0, 0, 0);
        for (int i(0); i < m_mySpheres.size(); ++i) {
            shared_ptr<Sphere> sphere(m_mySpheres[i]);
            if (intersectSphere(P, w, t_s, sphere, Color3(0, 0, 0))) {
                if (t_s < dCir) {
                    dCir = t_s;
                    X = P + t_s*w;
                    n = (X - sphere->center).direction();
                };
            };
        };

        if (dCir < dTri) {
            shared_ptr<Surfel> toReturn(new UniversalSurfel);
            surfel->lambertianReflectivity = Color3::white();
            surfel->geometricNormal = n;
            surfel->shadingNormal = n;
            surfel->position = X;

            return toReturn;
        };
    };

    if (dTri < inf()) {
        return m_triTree->sample(hit);
    }
    else {
        return nullptr;
    };
};

bool RayTracer::intersectTriangle(const Point3& P, const Vector3& w, const Point3 V[3], float b[3], float& t) const {
    const Vector3& e1(V[1] - V[0]);
    const Vector3& e2(V[2] - V[0]);
    const Vector3& n(e1.cross(e2).direction());

    const Vector3& q(w.cross(e2));
    float a(e1.dot(q));

    if (n.dot(w) >= 0 || abs(a) <= m_epsilon) {
        return nullptr;
    };


    const Vector3& s((P - V[0]) / a);

    const Vector3& r(s.cross(e1));

    b[0] = s.dot(q);
    b[1] = r.dot(w);
    b[2] = 1 - b[0] - b[1];

    if (b[0] < 0 || b[1] < 0 || b[2] < 0) {
        return nullptr;
    };

    t = e2.dot(r);
    return (t >= 0);
};

bool RayTracer::intersectSphere(const Point3& P, const Vector3& w, float& t, const shared_ptr<Sphere>& sphere, const Color3& color) const {
    const Point3& C(sphere->center);
    float r(sphere->radius);
    float b (2.0f*w.dot(P - C));
    float c ((P - C).dot(P - C) - r*r);
    float inRadical( b*b - 4.0f * c);

    if (inRadical < 0 || sphere->contains(P)) {
        return false;
    }
    else {
        float t_0( (-b + sqrt(inRadical)) / 2.0f);
        float t_1 ((-b - sqrt(inRadical)) / 2.0f);
        t = t_0 < t_1 ? t_0 : t_1;
        return true;
    };
};

// Adapted from C++ Direct Illumination [_rn_dirctIllm] from http://graphicscodex.com 
Radiance3 RayTracer::L_o(const shared_ptr<Surfel>& surfelX, const Vector3& wo, const Array<shared_ptr<Light>>& lights, int depth) const {
    // Begin with the emitted radiance
    Radiance3 L(surfelX->emittedRadiance(wo));
    const Point3& X(surfelX->position);
    const Vector3& n(surfelX->shadingNormal);

    Thread::runConcurrently(Point2int32(0, 0), Point2int32(lights.size(), 1), [&](Point2int32 i) {
        const shared_ptr<Light>& light(lights[i.x]);
        const Point3& Y(light->position().xyz());

        if (isVisible(X, Y, surfelX->geometricNormal.direction())) {
            const Vector3& wi((Y - X*(light->position().w)).direction());
            Biradiance3& Bi(light->biradiance(X));
            const Color3& f(surfelX->finiteScatteringDensity(wi, wo));
            L += Bi * f * abs(wi.dot(n)) + surfelX->reflectivity(Random::threadCommon()) * 0.05f;
        };

        Radiance3 indirectLight(Radiance3::black());
        if (depth > 0) {
            Thread::runConcurrently(Point2int32(0, 0), Point2int32(1, m_raysPerPixel), [&](Point2int32 i) {
                indirectLight += doIndirectLight(surfelX, wo, lights, depth);
            }, !m_runConcurrent);

            if (m_raysPerPixel > 0) {
                indirectLight /= (float)m_raysPerPixel;
                L += indirectLight;
            };
        };
    }, !m_runConcurrent);

    return L;
}

Radiance3 RayTracer::doIndirectLight(const shared_ptr<Surfel>& surfelX, const Vector3& wo, const Array<shared_ptr<Light>>& lights, int depth) const {
    Radiance3 r(0, 0, 0);
    const Vector3& wi(Vector3::cosHemiRandom(surfelX->shadingNormal, Random::threadCommon()).direction());
    const Color3& f(surfelX->finiteScatteringDensity(wi, wo));
    r += L_i(surfelX->position, wi, lights, depth - 1)*f;
    return r;
};

bool RayTracer::isVisible(const Point3& X, const Point3& Y, const Vector3& n) const {
    const Point3& X_prime(X + n*m_epsilon);
    const Vector3& w_i((X - Y).direction());
    const Point3& origin(Y + m_epsilon*(-1)*w_i);

    const shared_ptr<Surfel>& intersect(intersects(origin, w_i));
    return (intersect != nullptr) && (((intersect->position) - X).length() <= m_epsilon);
};