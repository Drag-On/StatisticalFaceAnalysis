//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <DBGL/System/Log/Log.h>
#include <DBGL/System/Properties/Properties.h>
#include <DBGL/Window/WindowManager.h>
#include <DBGL/Window/SimpleWindow.h>
#include <DBGL/Rendering/RenderContext.h>
#include <DBGL/Rendering/Mesh/Mesh.h>
#include <DBGL/Rendering/ShaderProgram.h>
#include <DBGL/Rendering/Camera.h>
#include <DBGL/Math/Utility.h>
#include <DBGL/Math/Matrix3x3.h>
#include <DBGL/Math/Matrix4x4.h>
#include <DBGL/Math/Quaternion.h>
#include "SFA/Model.h"
#include "NearestNeighbor/SimpleNearestNeighbor.h"
#include "ICP/RigidPointICP.h"
#include "ICP/PCA_ICP.h"

using namespace std;
using namespace Eigen;
using namespace dbgl;
using namespace sfa;

Window* pWnd;
Model* pSourceModel, *pDestModel;
ShaderProgram* pShader;
Vec3f colorSrc(1.0f, 0.0f, 0.0f), colorDest(0.0f, 1.0f, 0.0f);
Camera* pCam;
float camDist = 3;
Mat4f view, projection;
float moveSpeed = 2.5;

bool showSource = true, showDest = true;

SimpleNearestNeighbor nn;
RigidPointICP icp(nn);
PCA_ICP pca_icp;

Properties properties;

void updateViewProjection()
{
    view = Mat4f::makeView(pCam->position(), pCam->rotation() * Vec3f(0, 0, 1),
	    pCam->rotation() * Vec3f(0, 1, 0));
    projection = Mat4f::makeProjection(pCam->getFieldOfView(),
	    float(pWnd->getFrameWidth()) / pWnd->getFrameHeight(), pCam->getNear(), pCam->getFar());
}

void moveCamera(double x, double y)
{
    pCam->rotate(x, y);
    Vec3f dir;
    pCam->getOrientation(&dir, nullptr, nullptr);
    pCam->position() = -dir * camDist;
}

void scrollCallback(Window::ScrollEventArgs const& args)
{
    // Zoom
    pCam->setFieldOfView(pCam->getFieldOfView() + 0.1f * args.yOffset);
    updateViewProjection();
}

void framebufferResizeCallback(Window::FramebufferResizeEventArgs const& /*args*/)
{
    updateViewProjection();
}

void keyCallback(Window::KeyEventArgs const& args)
{
    // Check if next ICP step should be executed
    if (args.key == GLFW_KEY_I && args.action == GLFW_PRESS)
    {
	LOG->info("Calculating next ICP step!");
	// Compute next step
	icp.calcNextStep(*pSourceModel, *pDestModel);
	pSourceModel->getBasePointer()->updateBuffers();
	LOG->info("Done!");
    }
    // Check if "PCA ICP" should be executed
    if (args.key == GLFW_KEY_U && args.action == GLFW_PRESS)
    {
	LOG->info("Calculating PCA matching!");
	// Compute next step
	pca_icp.calcNextStep(*pSourceModel, *pDestModel);
	pSourceModel->getBasePointer()->updateBuffers();
	nn.clearCache();
	LOG->info("Done!");
    }
    // Toggle source and destination mesh visibility
    if(args.key == GLFW_KEY_O && args.action == GLFW_PRESS)
    {
	showSource = !showSource;
    }
    if(args.key == GLFW_KEY_P && args.action == GLFW_PRESS)
    {
	showDest = !showDest;
    }
    // Reload meshes
    if (args.key == GLFW_KEY_R && args.action == GLFW_PRESS)
    {
	LOG->info("Reloading meshes...");
	delete pSourceModel;
	delete pDestModel;
	pSourceModel = new Model(properties.getStringValue("src"));
	pDestModel = new Model(properties.getStringValue("dest"));
	pSourceModel->getBasePointer()->updateBuffers();
	pDestModel->getBasePointer()->updateBuffers();
	pca_icp.reset();
    }
    // Log matching error
    if(args.key == GLFW_KEY_L && args.action == GLFW_PRESS)
    {
	auto error = nn.computeError(*pSourceModel, *pDestModel);
	LOG->info("Matching error: %.20f", error);
    }
    // Modify point selection
    if(args.key == GLFW_KEY_F1 && args.action == GLFW_PRESS)
    {
	icp.setSelectionMethod(0);
	LOG->info("Using all points.");
    }
    else if(args.key == GLFW_KEY_F2 && args.action == GLFW_PRESS)
    {
	icp.selectionMethod().toggle(IterativeClosestPoint::EVERY_SECOND);
	if(icp.getSelectionMethod().isSet(IterativeClosestPoint::EVERY_SECOND))
	    LOG->info("Adding filter \"Every second\".");
	else
	    LOG->info("Removing filter \"Every second\".");
    }
    else if(args.key == GLFW_KEY_F3 && args.action == GLFW_PRESS)
    {
	icp.selectionMethod().toggle(IterativeClosestPoint::EVERY_THIRD);
	if(icp.getSelectionMethod().isSet(IterativeClosestPoint::EVERY_THIRD))
	    LOG->info("Adding filter \"Every third\".");
	else
	    LOG->info("Removing filter \"Every third\".");
    }
    else if(args.key == GLFW_KEY_F4 && args.action == GLFW_PRESS)
    {
	icp.selectionMethod().toggle(IterativeClosestPoint::EVERY_FOURTH);
	if(icp.getSelectionMethod().isSet(IterativeClosestPoint::EVERY_FOURTH))
	    LOG->info("Adding filter \"Every fourth\".");
	else
	    LOG->info("Removing filter \"Every fourth\".");
    }
    else if(args.key == GLFW_KEY_F5 && args.action == GLFW_PRESS)
    {
	icp.selectionMethod().toggle(IterativeClosestPoint::EVERY_FIFTH);
	if(icp.getSelectionMethod().isSet(IterativeClosestPoint::EVERY_FIFTH))
	    LOG->info("Adding filter \"Every fifth\".");
	else
	    LOG->info("Removing filter \"Every fifth\".");
    }
    else if(args.key == GLFW_KEY_F6 && args.action == GLFW_PRESS)
    {
	icp.selectionMethod().toggle(IterativeClosestPoint::NO_EDGES);
	if(icp.getSelectionMethod().isSet(IterativeClosestPoint::NO_EDGES))
	    LOG->info("Adding filter \"No edges\".");
	else
	    LOG->info("Removing filter \"No edges\".");
    }
    else if(args.key == GLFW_KEY_F7 && args.action == GLFW_PRESS)
    {
	icp.selectionMethod().toggle(IterativeClosestPoint::RANDOM);
	if(icp.getSelectionMethod().isSet(IterativeClosestPoint::RANDOM))
	    LOG->info("Adding filter \"Random\".");
	else
	    LOG->info("Removing filter \"Random\".");
    }
    // DEBUG!
    if(args.key == GLFW_KEY_H && args.action == GLFW_PRESS)
    {
	LOG->info("DEBUG!");
	for(unsigned int i = 0; i < pDestModel->getAmountOfVertices(); i++)
	{
	    auto vertex = pDestModel->getVertex(i);
	    if(vertex.isEdge)
	    {
		auto coords = vertex.coords + 0.5f * vertex.normal;
		pDestModel->setVertex(i, coords, vertex.normal);
	    }
	}
	pDestModel->getBasePointer()->updateBuffers();
    }
}

void updateCallback(Window::UpdateEventArgs const& args)
{
    auto deltaTime = args.deltaTime;

    // Update camera position and rotation
    double x = 0, y = 0;
    if (pWnd->getKey(GLFW_KEY_W) == GLFW_PRESS)
	y += deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_S) == GLFW_PRESS)
	y -= deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_A) == GLFW_PRESS)
	x -= deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_D) == GLFW_PRESS)
	x += deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_E) == GLFW_PRESS)
    	camDist -= deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_Q) == GLFW_PRESS)
	camDist += deltaTime * moveSpeed;
    moveCamera(x, y);

    // Update view matrix
    updateViewProjection();
}

void renderCallback(Window::RenderEventArgs const& args)
{
    // Instruct shader
    pShader->use();
    // MVP matrix
    Mat4f mvp = projection * view; // Unit model matrix
    GLint mvpId = pShader->getDefaultUniformHandle(ShaderProgram::Uniform::MVP);
    if (mvpId >= 0)
    {
	pShader->setUniformFloatMatrix4Array(mvpId, 1, GL_FALSE, mvp.getDataPointer());
    }
    // ITMV matrix
    GLint itmvId = pShader->getDefaultUniformHandle(
	    ShaderProgram::Uniform::ITMV);
    if (itmvId >= 0)
    {
	// In this case inverse transpose of view equals view
	pShader->setUniformFloatMatrix4Array(itmvId, 1, GL_FALSE, view.getDataPointer());
    }

    // Draw
    if (showSource)
    {
	pShader->setUniformFloat3(pShader->getDefaultUniformHandle(ShaderProgram::COLOR), colorSrc.getDataPointer());
	args.rc->draw(*(pSourceModel->getBasePointer()));
    }
    if (showDest)
    {
	pShader->setUniformFloat3(pShader->getDefaultUniformHandle(ShaderProgram::COLOR), colorDest.getDataPointer());
	args.rc->draw(*(pDestModel->getBasePointer()));
    }
}

int main(int argc, char** argv)
{
    LOG->setLogLevel(dbgl::DBG);
    LOG->info("Starting...");

    // Set default properties in case none have been passed
    properties.setValue("src", "Resources/Plane_Transformed.obj");
    properties.setValue("dest", "Resources/Plane.obj");
    // Interpret arguments
    // Skip first argument as it's the executable's path
    properties.interpret(argc-1, argv+1);

    // Create window
    pWnd = WindowManager::get()->createWindow<SimpleWindow>("Statistical Face Analysis");
    // Initialize it
    pWnd->init(Window::DepthTest);
    // Add a camera
    Vec3f pos = Vec3f(0, 0, camDist);
    Vec3f dir = -pos;
    pCam = new Camera(pos, dir, Vec3f(1, 0, 0).cross(dir), pi_4(), 0.1, 100);
    // Load mesh, shader and texture
    pSourceModel = new Model(properties.getStringValue("src"));
    pDestModel = new Model(properties.getStringValue("dest"));
    pSourceModel->getBasePointer()->updateBuffers();
    pDestModel->getBasePointer()->updateBuffers();
    pShader = ShaderProgram::createSimpleColorShader();
    // Add callbacks
    pWnd->addUpdateCallback(std::bind(&updateCallback, std::placeholders::_1));
    pWnd->addRenderCallback(std::bind(&renderCallback, std::placeholders::_1));
    pWnd->addScrollCallback(std::bind(&scrollCallback, std::placeholders::_1));
    pWnd->addFramebufferResizeCallback(std::bind(&framebufferResizeCallback, std::placeholders::_1));
    pWnd->addKeyCallback(std::bind(&keyCallback, std::placeholders::_1));
    // Show window
    pWnd->show();
    // Run update loop
    while (WindowManager::get()->isRunning())
    {
	WindowManager::get()->update();
    }
    // Clean up
    delete pSourceModel;
    delete pDestModel;
    delete pShader;
    delete pCam;
    // delete pWnd; // No need for this as windows will delete themselves when closed
    // Free remaining internal resources
    WindowManager::get()->terminate();

    LOG->info("That's it!");
    return 0;
}

