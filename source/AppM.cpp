/** \file App.cpp */


#include "AppM.h"
#include "RayTracerM.h"


using namespace std;
// Tells C++ to invoke command-line main() function even on OS X and Win32.
G3D_START_AT_MAIN();

int m_raysPerPixel;
bool m_fixedPrimitives;
bool m_multiThreading;
shared_ptr<Image> m_currentImage;
shared_ptr<RayTracer> m_rayTracer;
int m_indexPointer;


int main(int argc, const char* argv[]) {
    {
        G3DSpecification g3dSpec;
        g3dSpec.audio = false;
        initGLG3D(g3dSpec);
    }

    GApp::Settings settings(argc, argv);

    // Change the window and other startup parameters by modifying the
    // settings class.  For example:
    settings.window.caption = argv[0];

    // Set enable to catch more OpenGL errors
    // settings.window.debugContext     = true;

    // Some common resolutions:
    // settings.window.width            =  854; settings.window.height       = 480;
    // settings.window.width            = 1024; settings.window.height       = 768;
    settings.window.width = 1280; settings.window.height = 720;
    //settings.window.width             = 1920; settings.window.height       = 1080;
    // settings.window.width            = OSWindow::primaryDisplayWindowSize().x; settings.window.height = OSWindow::primaryDisplayWindowSize().y;
    settings.window.fullScreen = false;
    settings.window.resizable = !settings.window.fullScreen;
    settings.window.framed = !settings.window.fullScreen;

    // Set to true for a significant performance boost if your app can't render at 60fps, or if
    // you *want* to render faster than the display.
    settings.window.asynchronous = false;

    settings.hdrFramebuffer.depthGuardBandThickness = Vector2int16(64, 64);
    settings.hdrFramebuffer.colorGuardBandThickness = Vector2int16(0, 0);
    settings.dataDir = FileSystem::currentDirectory();
    settings.screenshotDirectory = "../journal/";

    settings.renderer.deferredShading = true;
    settings.renderer.orderIndependentTransparency = false;

    return App(settings).run();
}


App::App(const GApp::Settings& settings) : GApp(settings) {
}

// Called before the application loop begins.  Load data here and
// not in the constructor so that common exceptions will be
// automatically caught.
void App::onInit() {
    debugPrintf("Target frame rate = %f Hz\n", realTimeTargetDuration());

    GApp::onInit();
    setFrameDuration(1.0f / 120.0f);


    //Call setScene(shared_ptr<Scene>()) or setScene(MyScene::create()) to replace
    // the default scene here.

    showRenderingStats = false;

    makeGUI();



    // For higher-quality screenshots:
    // developerWindow->videoRecordDialog->setScreenShotFormat("PNG");
    // developerWindow->videoRecordDialog->setCaptureGui(false);
    developerWindow->cameraControlWindow->moveTo(Point2(developerWindow->cameraControlWindow->rect().x0(), 0));
    loadScene(
        //"G3D Sponza"
        "G3D Cornell Box" // Load something simple
        //developerWindow->sceneEditorWindow->selectedSceneName()  // Load the first scene encountered 
    );


    // I initialized it again
    m_rayTracer = shared_ptr<RayTracer>(new RayTracer(scene(), 0.0001));

    const shared_ptr<Entity>&       entity = shared_ptr<Entity>();

}



void App::onRender() {
    message("rendering...");
    StopWatch timer(StopWatch("Timer"));
    timer.tick();
    m_rayTracer->traceImage(activeCamera(), m_currentImage);
    timer.tock();
    debugPrintf("%f s \n", timer.elapsedTime());
    String time(format("%f s \n", timer.elapsedTime()));
    show(m_currentImage,time);

     // Post-process
   const shared_ptr<Texture>& src = Texture::fromImage("Source", m_currentImage);
    if (m_result) {
        m_result->resize(m_currentImage->width(), m_currentImage->height());
    };

    m_film->exposeAndRender(renderDevice, m_debugCamera->filmSettings(), src, settings().hdrFramebuffer.colorGuardBandThickness.x/* + settings().hdrFramebuffer.depthGuardBandThickness.x*/, settings().hdrFramebuffer.depthGuardBandThickness.x, m_result);
    m_result->toImage()->save("result.png"); 
};

void App::message(const String& msg) const {
    renderDevice->clear();
    renderDevice->push2D();
    debugFont->draw2D(renderDevice, msg, renderDevice->viewport().center(), 12,
        Color3::white(), Color4::clear(), GFont::XALIGN_CENTER, GFont::YALIGN_CENTER);
    renderDevice->pop2D();

    // Force update so that we can see the message
    renderDevice->swapBuffers();
}


void App::makeGUI() {
    // Initialize the developer HUD
    shared_ptr<GuiWindow> window = GuiWindow::create("Controls", debugWindow->theme(), Rect2D::xywh(1025, 175, 0, 0), GuiTheme::TOOL_WINDOW_STYLE);
    GuiPane* pane = window->pane();
    pane->addLabel("Use WASD keys + right mouse to move");
    Array<String> resolution;
    resolution.append("1x1", "320x200", "640x400");
    pane->addDropDownList("resolution", resolution, &m_indexPointer);
    pane->addCheckBox("fixed primitives", &m_fixedPrimitives);
    pane->addCheckBox("multithreading", &m_multiThreading);
    pane->addNumberBox("Rays per pixel", &m_raysPerPixel, "", GuiTheme::LINEAR_SLIDER, 0, 2048, 1);
    pane->addButton("reload", [this]() {loadScene(

        developerWindow->sceneEditorWindow->selectedSceneName()  // Load the first scene encountered 
    ); });

    pane->addButton("render", [this]() {

        try {
            
            switch (m_indexPointer) {
            case 0:m_currentImage = (Image::create(1, 1, ImageFormat::RGB8()));
                break;
            case 1:m_currentImage = (Image::create(320, 200, ImageFormat::RGB8()));
                break;
            case 2:m_currentImage = (Image::create(640, 400, ImageFormat::RGB8()));
                break;
            }
            
        }
        catch (...) {
            msgBox("Unable to render the image.");
        }
        m_rayTracer->m_runConcurrent= m_multiThreading;
        m_rayTracer->m_spheresOn = m_fixedPrimitives;
        m_rayTracer->m_raysPerPixel = m_raysPerPixel;
        onRender();

    });

    window->pack();

    window->setVisible(true);
    addWidget(window);

    //break;
    createDeveloperHUD();

    debugWindow->setVisible(true);
    developerWindow->videoRecordDialog->setEnabled(true);

    GuiPane* infoPane = debugPane->addPane("Info", GuiTheme::ORNATE_PANE_STYLE);

    // Example of how to add debugging controls
    infoPane->addLabel("You can add GUI controls");
    infoPane->addLabel("in App::onInit().");
    infoPane->addButton("Exit", [this]() { m_endProgram = true; });
    infoPane->pack();
}


// This default implementation is a direct copy of GApp::onGraphics3D to make it easy
// for you to modify. If you aren't changing the hardware rendering strategy, you can
// delete this override entirely.
void App::onGraphics3D(RenderDevice* rd, Array<shared_ptr<Surface> >& allSurfaces) {

    if (!scene()) {
        if ((submitToDisplayMode() == SubmitToDisplayMode::MAXIMIZE_THROUGHPUT) && (!rd->swapBuffersAutomatically())) {
            swapBuffers();
        }
        rd->clear();
        rd->pushState(); {
            rd->setProjectionAndCameraMatrix(activeCamera()->projection(), activeCamera()->frame());
            drawDebugShapes();
        } rd->popState();
        return;
    }

    GBuffer::Specification gbufferSpec = m_gbufferSpecification;
    extendGBufferSpecification(gbufferSpec);
    m_gbuffer->setSpecification(gbufferSpec);
    m_gbuffer->resize(m_framebuffer->width(), m_framebuffer->height());
    m_gbuffer->prepare(rd, activeCamera(), 0, -(float)previousSimTimeStep(), m_settings.hdrFramebuffer.depthGuardBandThickness, m_settings.hdrFramebuffer.colorGuardBandThickness);

    m_renderer->render(rd, m_framebuffer, scene()->lightingEnvironment().ambientOcclusionSettings.enabled ? m_depthPeelFramebuffer : shared_ptr<Framebuffer>(),
        scene()->lightingEnvironment(), m_gbuffer, allSurfaces);

    // Debug visualizations and post-process effects
    rd->pushState(m_framebuffer); {
        // Call to make the App show the output of debugDraw(...)
        rd->setProjectionAndCameraMatrix(activeCamera()->projection(), activeCamera()->frame());
        drawDebugShapes();
        const shared_ptr<Entity>& selectedEntity = (notNull(developerWindow) && notNull(developerWindow->sceneEditorWindow)) ? developerWindow->sceneEditorWindow->selectedEntity() : shared_ptr<Entity>();
        scene()->visualize(rd, selectedEntity, allSurfaces, sceneVisualizationSettings(), activeCamera());

        // Post-process special effects
        m_depthOfField->apply(rd, m_framebuffer->texture(0), m_framebuffer->texture(Framebuffer::DEPTH), activeCamera(), m_settings.hdrFramebuffer.depthGuardBandThickness - m_settings.hdrFramebuffer.colorGuardBandThickness);

        m_motionBlur->apply(rd, m_framebuffer->texture(0), m_gbuffer->texture(GBuffer::Field::SS_EXPRESSIVE_MOTION),
            m_framebuffer->texture(Framebuffer::DEPTH), activeCamera(),
            m_settings.hdrFramebuffer.depthGuardBandThickness - m_settings.hdrFramebuffer.colorGuardBandThickness);
    } rd->popState();

    // We're about to render to the actual back buffer, so swap the buffers now.
    // This call also allows the screenshot and video recording to capture the
    // previous frame just before it is displayed.
    if (submitToDisplayMode() == SubmitToDisplayMode::MAXIMIZE_THROUGHPUT) {
        swapBuffers();
    }

    // Clear the entire screen (needed even though we'll render over it, since
    // AFR uses clear() to detect that the buffer is not re-used.)
    rd->clear();

    // Perform gamma correction, bloom, and SSAA, and write to the native window frame buffer
    m_film->exposeAndRender(rd, activeCamera()->filmSettings(), m_framebuffer->texture(0), settings().hdrFramebuffer.colorGuardBandThickness.x + settings().hdrFramebuffer.depthGuardBandThickness.x, settings().hdrFramebuffer.depthGuardBandThickness.x);

}


void App::onAI() {
    GApp::onAI();
    // Add non-simulation game logic and AI code here
}


void App::onNetwork() {
    GApp::onNetwork();
    // Poll net messages here
}


void App::onSimulation(RealTime rdt, SimTime sdt, SimTime idt) {
    GApp::onSimulation(rdt, sdt, idt);

    // Example GUI dynamic layout code.  Resize the debugWindow to fill
    // the screen horizontally.
    debugWindow->setRect(Rect2D::xywh(0, 0, (float)window()->width(), debugWindow->rect().height()));
}


bool App::onEvent(const GEvent& event) {
    // Handle super-class events
    if (GApp::onEvent(event)) { return true; }

    // If you need to track individual UI events, manage them here.
    // Return true if you want to prevent other parts of the system
    // from observing this specific event.
    //
    // For example,
    // if ((event.type == GEventType::GUI_ACTION) && (event.gui.control == m_button)) { ... return true; }
    // if ((event.type == GEventType::KEY_DOWN) && (event.key.keysym.sym == GKey::TAB)) { ... return true; }
    // if ((event.type == GEventType::KEY_DOWN) && (event.key.keysym.sym == 'p')) { ... return true; }

    return false;
}


void App::onUserInput(UserInput* ui) {
    GApp::onUserInput(ui);
    (void)ui;
    // Add key handling here based on the keys currently held or
    // ones that changed in the last frame.
}


void App::onPose(Array<shared_ptr<Surface> >& surface, Array<shared_ptr<Surface2D> >& surface2D) {
    GApp::onPose(surface, surface2D);

    // Append any models to the arrays that you want to later be rendered by onGraphics()
}


void App::onGraphics2D(RenderDevice* rd, Array<shared_ptr<Surface2D> >& posed2D) {
    // Render 2D objects like Widgets.  These do not receive tone mapping or gamma correction.
    Surface2D::sortAndRender(rd, posed2D);
}


void App::onCleanup() {
    // Called after the application loop ends.  Place a majority of cleanup code
    // here instead of in the constructor so that exceptions can be caught.
}
//void App::makeHeightfield(const int& xScale, const int& zScale, const Image& img, const float& max){
//    TextOutput::Settings opt;
//    opt.wordWrap = TextOutput::Settings::WRAP_NONE;
//    const String filename("model/heightfield.off");
//    TextOutput output(filename , opt); // Create file
//
//    //Change these while testing
//    int h(img.height());
//    int w(img.width());
//    int nVerts(w*h);
//    int nFaces((w-1)*(h-1));
//
//    
//    const String header("OFF\n# NVertices NFaces NEdges\n%d %d 0\n");
//    output.printf(header,nVerts,nFaces);
//    
//    // Vertices
//    for(int x(0); x<w; ++x){
//        for(int z(0); z<h; ++z){
//            Point2int32 pos(x,z);
//            Color3 color(G3D::Color3::gray());
//            img.get(pos, color);
//            float y(color.average());
//            output.printf("%f %f %f\n", (float)x*xScale, (float)y*max, (float)z*zScale);
//        };
//    };
//
//    //Faces
//    for(int x(0); x<w-1; ++x){
//        int i(x*h);
//        for(int z(0); z<h-1; ++z){
//            int c(i+z);
//            output.printf("%d %d %d %d %d\n",4, c,c+1,c+h+1,c+h);
//        };
//    };
//    output.commit();
//};
//
//void App::makeCylinder(const float& r, const float& h){
//    TextOutput::Settings opt;
//    opt.wordWrap = TextOutput::Settings::WRAP_NONE;
//    const String filename("model/cylinder.off");
//    TextOutput output(filename , opt); // Create file
//
//    //Change these while testing
//    int nVerts(72);
//    int nFaces(104);
//
//    const String header("OFF\n# NVertices NFaces NEdges\n%d %d 0\n");
//    output.printf(header,nVerts,nFaces);
//    
//    int n(nVerts/2);
//    //Print vertices 
//    float y(0.0);
//    while(y <= h){
//        for (int i(0); i<n; ++i){
//            float a(2.0*pi()*r*(float)i/(float)n);
//            double x = sin(a)*r;
//            double z = cos(a)*r;
//            output.printf("%f %f %f\n", x, y-(h/2), z);
//        };
//        y = y+h;
//    };
//
//    //Print faces
//    //top/bottom
//    int base(0);
//    while(base <= n){
//        for (int i(0); i<n-2; ++i){
//            //top and bottom
//            int x(2);
//            int y(1);
//            if(base>0){
//                x=1;
//                y=2;
//            };
//            output.printf("%d %d %d %d\n",3, base, (i+x)%n+base, (i+y)%n+base);
//        };
//        base = base+n;
//    };
//
//    int m(n); 
//    for(int i(0); i<m; ++i){
//        //sides
//        output.printf("%d %d %d %d %d\n",4, i%(nVerts-2), (i+1)%m, (i+1)%m+m,i%m+m);
//    }
// 
//
//    output.commit();
//};
//
//void App::staircase() {
//    // Helper funcition to output Staircase.Scene.Any
//    TextOutput::Settings opt;
//    opt.wordWrap = TextOutput::Settings::WRAP_NONE;
//    const String filename("scene/Staircase.Scene.Any");
//    TextOutput output(filename , opt); // Create file
//
//    const String header("// -*- c++ -*-\n{\n    name = \"Staircase\";\n    models = {\n        stepModel = ArticulatedModel::Specification {\n            filename = \"model/crate/crate.obj\";\n            preprocess = {\n                setMaterial(all(),\"model/crate/blackMarble.jpg\");\n                transformGeometry(all(), Matrix4::scale(2.5,0.4,1.2));\n                transformCFrame(root(), Vector3(3, 0, 0 ) );\n            };\n        };\n    };\n\n    entities = {\n        skybox = Skybox {\n            texture = \"cubemap/whiteroom/whiteroom-*.png\";\n        };\n\n        sun = Light {\n            attenuation = ( 0, 0, 1 );\n            bulbPower = Power3(4e+006);\n            frame = CFrame::fromXYZYPRDegrees(-15, 207, -41, -164, -77, 77);\n            shadowMapSize = Vector2int16(2048, 2048);\n            spotHalfAngleDegrees = 5;\n            spotSquare = true;\n            type = \"SPOT\";\n        };\n\n        camera = Camera {\n            frame = CFrame::fromXYZYPRDegrees(0, 6, 18);\n        };\n");
//    output.printf(header);
//
//    // Write step0-49
//    for (int i(0); i < 50; ++i){
//        float level((float)i*2.0/10.0); // vertical level (20cm overlapping)
//        int angle((i*15)%360); // yaw rotation 0 to 360 by 15 degrees
//        output.printf("\nstep%d = VisibleEntity {\n            model = \"stepModel\";\n            frame = CFrame::fromXYZYPRDegrees(0, %.1f, 0, %d, 0, 0);\n         };",i,level,angle);
//    };
//    output.printf("};\n};");
//    output.commit();
// };
//
//void App::pineTree(){
//    // Helper funcition to output PineTree.Scene.Any
//    TextOutput::Settings opt;
//    opt.wordWrap = TextOutput::Settings::WRAP_NONE;
//    const String filename("scene/PineTree.Scene.Any");
//    TextOutput output(filename , opt); // Create file
//
//    const String model("// -*- c++ -*-\n{\n    name = \"Pine Tree\";\n    models = {\n        logModel = ArticulatedModel::Specification {\n            filename = \"model/crate/crate.obj\";\n            preprocess = {\n                setMaterial(all(),Color3(0.55,0.27,0.07));\n                transformGeometry(all(), Matrix4::scale(1,4.0,1));\n};\n};\n");
//    output.printf(model);
//    
//    for (int i(0); i < 100; ++i){
//        float side((100.0 - (float)i)/10.0);
//        output.printf("cubeModel%d = ArticulatedModel::Specification {\n            filename = \"model/crate/crate.obj\";\n            preprocess = {\n                setMaterial(all(),Color3(0,0.39,0));\n                transformGeometry(all(), Matrix4::scale(%.1f,0.2,%.1f));\n};\n        };\n",i,side,side);
//    };
//    output.printf("};\n\n    entities = {\n              skybox = Skybox {\n            texture = \"cubemap/whiteroom/whiteroom-*.png\";\n        };\n\n        sun = Light {\n            attenuation = ( 0, 0, 1 );\n            bulbPower = Power3(4e+006);\n            frame = CFrame::fromXYZYPRDegrees(-15, 207, -41, -164, -77, 77);\n            shadowMapSize = Vector2int16(2048, 2048);\n            spotHalfAngleDegrees = 5;\n            spotSquare = true;\n            type = \"SPOT\";\n        };\n\n        camera = Camera {\n            frame = CFrame::fromXYZYPRDegrees(0, 60, 100, 0,-45);\n        };\n");
//
//    
//    for(int x(0); x < 10; ++x){
//        for(int z(0); z < 10; ++z){
//            output.printf("log%d%d = VisibleEntity {\n            model = \"logModel\";\n            frame = CFrame::fromXYZYPRDegrees(%d, 2.0, %d, 0, 0, 0);\n         };\n", x, z,-15*x,-15*z);
//            for (int i(0); i<100; i = i + 2){
//                float level((float)i/20.0);
//                int angle((i*14)%360);
//                output.printf("\ncube%d%d%d = VisibleEntity {\n            model = \"cubeModel%d\";\n            frame = CFrame::fromXYZYPRDegrees(%d, %.2f, %d, %d, 0, 0);\n         };",i,x,z,i,-15*x,level+4.0,-15*z,angle);
//                output.printf("\ncube%d%d%d = VisibleEntity {\n            model = \"cubeModel%d\";\n            frame = CFrame::fromXYZYPRDegrees(%d, %.2f, %d, %d, 0, 0);\n         };",i,x,z,i,-15*x,2.0*level+4.0,-15*z,-angle);
//            };
//        };
//    };
//    
//    output.printf("};\n};");
//    output.commit();
//};

