#pragma once
#include <G3D/G3DAll.h>
#define EPSILON 0.0001f

class PathTracer {
    protected:
        shared_ptr<TriTree> m_triangles;
        shared_ptr<Scene> m_scene;
        float m_lastTreeBuildTime;
        int m_width;

        // Radiance3 measureLight(const Ray& ray, int numScatters) const;
        // Radiance3 shade(const Ray& ray, const shared_ptr<Surfel>& surfel) const;
        
        Radiance3 L_in(const Point3& X, const Vector3& w_in, int pathDepth, const TriTree& triArray) const; 
        Radiance3 L_out(const shared_ptr<Surfel>& surfel, const Vector3& w_out, const Array<shared_ptr<Light>>& lights, const Point3& origin, const int j) const;
      
        bool isVisible(const Point3& X, const Point3& Y) const;
    
        Radiance3 backgroundRadiance(const Vector3& direction, const Point3& origin, const int j) const; 
                
        /*generate primary rays*/
        void generatePrimaryRays(Array<Ray>& rayBuffer,const shared_ptr<Camera>& cam, int width, int height, int j) const;
      
        /*choose which light to sample, measuring biradiance and
                   computing a shadow ray*/
        void computeShadowRays(Array<Ray>& shadowRayBuffer,  Array<Color3>& modulationBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Light>>& lights, const int j) const;
     
        /*scatter the rays, multiplying the modulation buffer by
                   the scattering weights*/
        void testVisibility(const Array<Ray>& shadowRayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<bool>& lightShadowedBuffer, const int j) const; 
        
        // Generates new set of rays for the next iteration
        void generateRecursiveRays(Array<Ray>& rayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Color3>& modulationBuffer, const int j) const;

        // Writes emmitted and sampled direct light to image for each iteration
        void writeToImage(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Color3>& modulationBuffer, const int j, const shared_ptr<Image>& image, const Array<Ray>& rayBuffer, const Array<shared_ptr<Light>>& lights) const;

    public:
        int m_numPaths;
        int m_maxScatters;

        PathTracer(const shared_ptr<Scene>& scene);
        ~PathTracer();
        void traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image);
        
        //Radiance3 colorSky(const Ray& ray,  const Point2int32& location);
};