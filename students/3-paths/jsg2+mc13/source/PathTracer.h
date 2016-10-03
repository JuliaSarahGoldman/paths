/** \file RayTracer.h */
/*
  Julia Goldman
  Matheus de Carvalho Souza
  Jose Rivas-Garcia
  Youle Chen
  */

#pragma once
#include <G3D/G3DAll.h>
#define EPSILON 0.0001f

class PathTracer {
    protected:
        mutable TriTree m_triangles;
        shared_ptr<Scene> m_scene;
        /** Image width*/
        int m_width;

        /** Called by writeToImage()
            Returns backgroundRadiance() if surfel is null */
        Radiance3 L_out(const shared_ptr<Surfel>& surfel, const Point3& Y, const Vector3& w_out, const Radiance3& bi, bool notVisible, const Point3& origin, const int j) const;
    
        /** Called by L_out()*/
        Radiance3 backgroundRadiance(const Vector3& direction, const Point3& origin, const int j) const; 
                
        /** Populates rayBuffer */
        void generatePrimaryRays(Array<Ray>& rayBuffer,const shared_ptr<Camera>& cam, int width, int height) const;
      
        /** Samples light
            Populates shadowRayBuffer*/
        void computeShadowRays(Array<Ray>& shadowRayBuffer,  Array<Color3>& modulationBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Light>>& lights, const int j) const;
     
        /** Repopulates rayByffer 
            Updates modulationBuffer */
        void generateRecursiveRays(Array<Ray>& rayBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, Array<Color3>& modulationBuffer, const int j) const;

        /** Called by traceImage()
            Calls L_out() */
        void writeToImage(const Array<Ray>& shadowRayBuffer, const Array<Radiance3>& biradianceBuffer, const Array<shared_ptr<Surfel>>& surfelBuffer, const Array<Color3>& modulationBuffer, const int j, const shared_ptr<Image>& image, const Array<Ray>& rayBuffer, const Array<bool>& lightShadowedBuffer) const;

    public:
        int m_numPaths;
        int m_maxScatters;

        PathTracer(const shared_ptr<Scene>& scene, int numPaths, int maxScatters);
        ~PathTracer();
        void traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image);
};