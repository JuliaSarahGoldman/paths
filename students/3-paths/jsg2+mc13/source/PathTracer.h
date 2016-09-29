#pragma once
#include <G3D/G3DAll.h>
#define EPSILON 0.0001f

class PathTracer {
    protected:
        shared_ptr<TriTree> m_triangles;
        shared_ptr<Scene> m_scene;
        float m_lastTreeBuildTime;

        // Radiance3 measureLight(const Ray& ray, int numScatters) const;
        // Radiance3 shade(const Ray& ray, const shared_ptr<Surfel>& surfel) const;
        
        Radiance3 L_in(const Point3& X, const Vector3& w_in, int pathDepth, const TriTree& triArray) const; 
        Radiance3 L_out(const shared_ptr<Surfel>& surfel,const Vector3& w_out, int pathDepth,const TriTree& triArray) const; 
      
        bool isVisible(const Point3& X, const Point3& Y) const;
    
        Radiance3 backgroundRadiance(const Vector3& direction) const; 
                
        /*extract surfaces from the scene
        rebuild the tree*/
        void buildTree();
        /*set the image to all black*/
        void resetImage();
        /*generate primary rays*/
        void generatePrimaryRays(const Array<Ray>& rayBuffer, const Array<Color3>& modulationBuffer) const;
        /*add emissive terms (use the sky's radiance for missed rays and
                apply the modulation buffer)*/
        void addEmissiveTerms(const Array<Radiance3>& biradianceBuffer, const Array<Color3>& modulationBuffer) const;
        /*choose which light to sample, measuring biradiance and
                   computing a shadow ray*/
        void computeShadowRays(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer) const;
        /*cast all shadow rays*/
        void castShadowRays(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer);
        /*shade all hit points*/
        void shadeHitpoints(const Array<Ray>& shadowRayBuffer, const Array<bool>& lightShadowedBuffer) const;
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