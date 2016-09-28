#pragma once
#include <G3D/G3DAll.h>
#define EPSILON 0.0001f

class PathTracer {
    protected:
        shared_ptr<TriTree> m_triangles;
        shared_ptr<Scene> m_scene;
        float m_lastTreeBuildTime;

        //Will be passed, not stored
        Array<Color3> m_modulationBuffer;
        Array<Ray> m_rayBuffer;
        Array<Surfel> m_surfelBuffer;
        Array<Radiance3> m_biradianceBuffer;
        Array<Ray> m_shadowRayBuffer;
        Array<bool> m_lightShadowedBuffer;


        Radiance3 measureLight(const shared_ptr<Scene>& scene, const Ray& ray, int numScatters) const;
        Radiance3 shade(const Ray& ray, const shared_ptr<Surfel>& surfel) const;
        bool isVisible(const Point3& X, const Point3& Y) const;
        
        /*extract surfaces from the scene
        rebuild the tree*/
        void buildTree();
        /*set the image to all black*/
        void resetImage();
        /*generate primary rays*/
        void generatePrimaryRays();
        /*add emissive terms (use the sky's radiance for missed rays and
                apply the modulation buffer)*/
        void addEmissiveTerms();
        /*choose which light to sample, measuring biradiance and
                   computing a shadow ray*/
        void computeShadowRays();
        /*cast all shadow rays*/
        void castShadowRays();
        /*shade all hit points*/
        void shadeHitpoints();
        /*scatter the rays, multiplying the modulation buffer by
                   the scattering weights*/
        void scatter();

    public:
        int m_numPaths;
        int m_maxScatters;

        PathTracer(const shared_ptr<Scene>& scene);
        ~PathTracer();
        void traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image);
        
        //Radiance3 colorSky(const Ray& ray,  const Point2int32& location);
};