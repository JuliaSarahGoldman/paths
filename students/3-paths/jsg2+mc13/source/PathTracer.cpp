/** \file RayTracer.cpp */
// Julia Goldman + Matheus de Carvalho Souza
#include "PathTracer.h"

PathTracer::PathTracer(const shared_ptr<Scene>& scene){};

void PathTracer::traceImage(const shared_ptr<Camera>& cam, const shared_ptr<Image>& image){};

Radiance3 PathTracer::L_in(const Point3& X, const Vector3& w_in, int pathDepth, const TriTree& triArray) const {};
Radiance3 PathTracer::L_out(const Point3& X, const Vector3& w_out, const Vector3& n, const Color3& f, const Radiance3& L_e, int pathDepth, const TriTree& triArray) const {};

bool  PathTracer::isVisible(const Point3& X, const Point3& Y) const {};

Radiance3  PathTracer::backgroundRadiance(const Vector3& direction) const {};


void PathTracer::buildTree(){};

void PathTracer::resetImage(){};

void PathTracer::generatePrimaryRays(){};

void PathTracer::addEmissiveTerms(){};

void PathTracer::computeShadowRays(){};

void PathTracer::castShadowRays(){};

void PathTracer::shadeHitpoints(){};

void PathTracer::scatter(){};

