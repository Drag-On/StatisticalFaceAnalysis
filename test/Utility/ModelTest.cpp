//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include <stdexcept>
#include <assert.h>
#include <DBGL/System/Log/Log.h>
#include <SFA/Utility/Log.h>
#include <SFA/Utility/Model.h>

using namespace sfa;

void testModel()
{
    LOG.info("Starting Model test suite...");
    // Load cube model
    Model model("Resources/Cube.obj", false);
    assert(model.getAmountOfVertices() == 8);
    assert(model.getBasePointer()->getVertices().size() == 6 * 6);

    // Check normals
    auto oldSFAVert = model.getVertex(0);
    decltype(oldSFAVert.normal) avrgBaseNormal(0, 0, 0);
    for(auto i : oldSFAVert.baseVertices)
    {
	auto dbglNormal = model.getBasePointer()->getNormals()[i];
	avrgBaseNormal += decltype(oldSFAVert.normal)(dbglNormal[0], dbglNormal[1], dbglNormal[2]);
    }
    avrgBaseNormal.normalize();
    assert(dbgl::isSimilar((oldSFAVert.normal - avrgBaseNormal).norm(), 0, 0.0001f));

    // Rotate by 90° around y
    Eigen::AngleAxis<double> aa(dbgl::pi_2(), Eigen::Vector3d(0, 1, 0));
    model.setVertex(0, oldSFAVert.coords, aa.toRotationMatrix() * oldSFAVert.normal);

    // Check again
    auto newSFAVert = model.getVertex(0);
    assert(dbgl::isSimilar((aa.toRotationMatrix() * oldSFAVert.normal - newSFAVert.normal).norm(), 0, 0.0001f));

    avrgBaseNormal = decltype(oldSFAVert.normal)(0, 0, 0);
    for(auto i : newSFAVert.baseVertices)
    {
	auto dbglNormal = model.getBasePointer()->getNormals()[i];
	avrgBaseNormal += decltype(newSFAVert.normal)(dbglNormal[0], dbglNormal[1], dbglNormal[2]);
    }
    avrgBaseNormal.normalize();
    assert(dbgl::isSimilar((newSFAVert.normal - avrgBaseNormal).norm(), 0, 0.0001f));
}
