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
#include "System/Log/Log.h"
#include "Window/WindowManager.h"
#include "Window/SimpleWindow.h"
#include "Rendering/RenderContext.h"
#include "Rendering/Mesh.h"
#include "Rendering/ShaderProgram.h"
#include "Rendering/Texture.h"
#include "Rendering/Camera.h"
#include "Math/Utility.h"
#include "Math/Quaternion.h"
#include "SFA/Model.h"
#include "NearestNeighbor/SimpleNearestNeighbor.h"
#include "ICP/RigidPointICP.h"

using namespace std;
using namespace Eigen;
using namespace dbgl;
using namespace sfa;

Window* pWnd;
Model* pSourceModel, *pDestModel;
ShaderProgram* pShader;
Texture* pTexture;
Camera* pCam;
Mat4f view, projection;
float mouseSpeed = 1.5, moveSpeed = 2.5;

bool showSource = true, showDest = true;

SimpleNearestNeighbor nn;
RigidPointICP icp;

void scrollCallback(Window::ScrollEventArgs const& args)
{
    // Zoom
    pCam->setFieldOfView(pCam->getFieldOfView() + 0.1f * args.yOffset);
    projection = Mat4f::makeProjection(pCam->getFieldOfView(),
	    float(pWnd->getFrameWidth()) / pWnd->getFrameHeight(),
	    pCam->getNear(), pCam->getFar());
}

void framebufferResizeCallback(Window::FramebufferResizeEventArgs const& args)
{
    projection = Mat4f::makeProjection(pCam->getFieldOfView(),
	    float(args.width) / args.height, pCam->getNear(), pCam->getFar());
}

void keyCallback(Window::KeyEventArgs const& args)
{
    // Check if next ICP step should be executed
    if (args.key == GLFW_KEY_I && args.action == GLFW_PRESS)
    {
	// Compute next step
	icp.calcNextStep(*pSourceModel, *pDestModel);
	pSourceModel->getBasePointer()->updateBuffers();
    }
    if(args.key == GLFW_KEY_O && args.action == GLFW_PRESS)
    {
	showSource = !showSource;
    }
    if(args.key == GLFW_KEY_P && args.action == GLFW_PRESS)
    {
	showDest = !showDest;
    }
}

void updateCallback(Window::UpdateEventArgs const& args)
{
    auto deltaTime = args.deltaTime;

    // Update mouse
    double x, y;
    pWnd->getCursorPos(x, y);
    float horizontal = deltaTime * mouseSpeed
	    * float(pWnd->getFrameWidth() / 2 - x);
    float vertical = deltaTime * mouseSpeed
	    * float(pWnd->getFrameHeight() / 2 - y);
    pCam->rotate(horizontal, -vertical);
    // Camera vectors
    Vec3f direction = pCam->rotation() * Vec3f(0, 0, 1);
    Vec3f right = pCam->rotation() * Vec3f(-1, 0, 0);
    // Reset mouse position to center of the screen
    pWnd->setCursorPos(pWnd->getFrameWidth() / 2, pWnd->getFrameHeight() / 2);

    // Update view matrix
    view = Mat4f::makeView(pCam->position(), pCam->rotation() * Vec3f(0, 0, 1),
	    pCam->rotation() * Vec3f(0, 1, 0));

    // Update keyboard
    if (pWnd->getKey(GLFW_KEY_W) == GLFW_PRESS)
	pCam->position() += direction * deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_A) == GLFW_PRESS)
	pCam->position() -= right * deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_S) == GLFW_PRESS)
	pCam->position() -= direction * deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_D) == GLFW_PRESS)
	pCam->position() += right * deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_E) == GLFW_PRESS)
	pCam->position() += Vec3f(0, 1, 0) * deltaTime * moveSpeed;
    if (pWnd->getKey(GLFW_KEY_Q) == GLFW_PRESS)
	pCam->position() -= Vec3f(0, 1, 0) * deltaTime * moveSpeed;
}

void renderCallback(Window::RenderEventArgs const& args)
{
    // Instruct shader
    pShader->use();
    // Diffuse texture
    GLint diffuseId = pShader->getDefaultUniformHandle(
	    ShaderProgram::TEX_DIFFUSE);
    if (diffuseId >= 0)
    {
	// Bind diffuse texture to unit 0
	pShader->bindTexture(GL_TEXTURE0, GL_TEXTURE_2D, pTexture->getTextureHandle());
	pShader->setUniformSampler(diffuseId, 0);
    }
    // MVP matrix
    Mat4f mvp = projection * view * Mat4f::makeScale(0.01f);
    GLint mvpId = pShader->getDefaultUniformHandle(ShaderProgram::Uniform::MVP);
    if (mvpId >= 0)
    {
	pShader->setUniformFloatMatrix4Array(mvpId, 1, GL_FALSE,
		mvp.getDataPointer());
    }
    // ITMV matrix
    GLint itmvId = pShader->getDefaultUniformHandle(
	    ShaderProgram::Uniform::ITMV);
    if (itmvId >= 0)
    {
	// In this case inverse transpose of view equals view
	pShader->setUniformFloatMatrix4Array(itmvId, 1, GL_FALSE,
		view.getDataPointer());
    }

    // Draw
    if (showSource)
	args.rc->draw(*(pSourceModel->getBasePointer()));
    if (showDest)
	args.rc->draw(*(pDestModel->getBasePointer()));
}

int main(int argc, char** argv)
{
    LOG->setLogLevel(dbgl::DBG);
    LOG->info("Starting...");

    // Create window
    pWnd = WindowManager::get()->createWindow<SimpleWindow>();
    // Initialize it
    pWnd->init(true, false, false);
    // Add a camera
    Vec3f direction = Vec3f(1, -2.5, -3);
    pCam = new Camera(Vec3f(-1, 2, 3), direction, Vec3f(1, 0, 0).cross(direction), pi_4(), 0.1, 100);
    // Calculate model and view matrix
    view = Mat4f::makeView(pCam->position(),
	    pCam->rotation() * Vec3f(0, 0, 1), pCam->rotation() * Vec3f(0, 1, 0));
    projection = Mat4f::makeProjection(pCam->getFieldOfView(),
	    float(pWnd->getFrameWidth()) / pWnd->getFrameHeight(), pCam->getNear(), pCam->getFar());
    // Load mesh, shader and texture
    pSourceModel = new Model("Resources/Plane_Transformed.obj");
    pDestModel = new Model("Resources/Plane.obj");
    pSourceModel->getBasePointer()->updateBuffers();
    pDestModel->getBasePointer()->updateBuffers();
    pShader = ShaderProgram::createSimpleShader();
    pTexture = new Texture(Texture::BOGUS, "");
    // Add render callback so we can draw the mesh
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
    delete pTexture;
    delete pCam;
    // delete pWnd; // No need for this as windows will delete themselves when closed
    // Free remaining internal resources
    WindowManager::get()->terminate();

    LOG->info("That's it!");
    return 0;
}

