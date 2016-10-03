/**
  \file App.h

  The G3D 10.00 default starter app is configured for OpenGL 4.1 and
  relatively recent GPUs.
 */
/*  Julia Goldman
  Matheus de Carvalho Souza
  Jose Rivas-Garcia
  Youle Chen
  */
#pragma once
#include <G3D/G3DAll.h>

/** \brief Application framework. */
class App : public GApp {
protected:

    bool m_hasFixedPrimitives;
    bool m_isMultithreaded;
    int m_indirectRaysPPx;
    float m_heightfieldYScale;
    float m_heightfieldXZScale;
    float m_radius;
    float m_height;
    int m_slices;
    String m_heightfieldSource;
    int m_numPaths;
    int m_maxScatters;
    /** Allocated by expose and render */
    shared_ptr<Texture> m_result;
    /** Called from onInit */
    void makeGUI();

public:
    void makeTriangles(int numTris);
    App(const GApp::Settings& settings = GApp::Settings());

    void makeCylinder(float radius, float height, int numVertices);
    void makeFountainPiece();
    void makeSplash();
    void makeTree();
    void makeGlass(int slices);
    String makeTube(Array<float>& radii, Array<float>& heights, int slices);
    virtual void onInit() override;
    virtual void onAI() override;
    virtual void onNetwork() override;
    virtual void onSimulation(RealTime rdt, SimTime sdt, SimTime idt) override;
    virtual void onPose(Array<shared_ptr<Surface> >& posed3D, Array<shared_ptr<Surface2D> >& posed2D) override;

    // You can override onGraphics if you want more control over the rendering loop.
    // virtual void onGraphics(RenderDevice* rd, Array<shared_ptr<Surface> >& surface, Array<shared_ptr<Surface2D> >& surface2D) override;

    virtual void onGraphics3D(RenderDevice* rd, Array<shared_ptr<Surface> >& surface3D) override;
    virtual void onGraphics2D(RenderDevice* rd, Array<shared_ptr<Surface2D> >& surface2D) override;

    virtual bool onEvent(const GEvent& e) override;
    virtual void onUserInput(UserInput* ui) override;
    virtual void onCleanup() override;
};

