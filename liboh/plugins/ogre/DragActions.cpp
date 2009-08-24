/*  Sirikata liboh -- Ogre Graphics Plugin
 *  DragActions.cpp
 *
 *  Copyright (c) 2009, Patrick Reiter Horn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of Sirikata nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <util/Standard.hh>
#include <oh/Platform.hpp>
#include "OgreSystem.hpp"
#include "CameraEntity.hpp"
#include "LightEntity.hpp"
#include "MeshEntity.hpp"
#include "input/SDLInputManager.hpp"
#include "DragActions.hpp"
#include <oh/SpaceTimeOffsetManager.hpp>
#include <task/Time.hpp>

namespace Sirikata {
namespace Graphics {

using namespace Input;
using namespace Task;


////////////////////////////////////////////////////////////////////////////////
// RelativeDrag
////////////////////////////////////////////////////////////////////////////////

class RelativeDrag : public ActiveDrag {
    PointerDevicePtr mDevice;
public:
    RelativeDrag(const PointerDevicePtr &dev) : mDevice(dev) {
        if (dev) {
            mDevice->pushRelativeMode();
        }
    }
    ~RelativeDrag() {
        if (mDevice) {
            mDevice->popRelativeMode();
        }
    }
};


////////////////////////////////////////////////////////////////////////////////
// NullDrag
////////////////////////////////////////////////////////////////////////////////

class NullDrag : public ActiveDrag {
public:
    NullDrag(const DragStartInfo &info) {}
    void mouseMoved(MouseDragEventPtr ev) {}
};

DragAction nullDragAction (ActiveDrag::factory<NullDrag>);


////////////////////////////////////////////////////////////////////////////////
// DragActionRegistry
////////////////////////////////////////////////////////////////////////////////

DragActionRegistry &DragActionRegistry::getSingleton() {
    static DragActionRegistry *sSingleton = NULL;
    if (!sSingleton) {
        sSingleton = new DragActionRegistry;
    }
    return *sSingleton;
}

void DragActionRegistry::set(const std::string &name, const DragAction &obj) {
    getSingleton().mRegistry[name] = obj;
}

void DragActionRegistry::unset(const std::string &name) {
    std::tr1::unordered_map<std::string, DragAction>::iterator iter =
        getSingleton().mRegistry.find(name);
    if (iter != getSingleton().mRegistry.end()) {
        getSingleton().mRegistry.erase(iter);
    }
}

const DragAction &DragActionRegistry::get(const std::string &name) {
    std::tr1::unordered_map<std::string, DragAction>::iterator iter =
        getSingleton().mRegistry.find(name);
    if (iter == getSingleton().mRegistry.end()) {
        return nullDragAction;
    }
    return iter->second;
}


////////////////////////////////////////////////////////////////////////////////
// Utilities
////////////////////////////////////////////////////////////////////////////////

void pixelToRadians(CameraEntity *cam, float deltaXPct, float deltaYPct, float &xRadians, float &yRadians) {
    // This function is useless and hopelessly broken, since radians have no meaning in perspective. Use pixelToDirection instead!!!
    SILOG(input,info,"FOV Y Radians: "<<cam->getOgreCamera()->getFOVy().valueRadians()<<"; aspect = "<<cam->getOgreCamera()->getAspectRatio());
    xRadians = cam->getOgreCamera()->getFOVy().valueRadians() * cam->getOgreCamera()->getAspectRatio() * deltaXPct;
    yRadians = cam->getOgreCamera()->getFOVy().valueRadians() * deltaYPct;
    SILOG(input,info,"X = "<<deltaXPct<<"; Y = "<<deltaYPct);
    SILOG(input,info,"Xradian = "<<xRadians<<"; Yradian = "<<yRadians);
}
// Uses perspective
Vector3f pixelToDirection(CameraEntity *cam, Quaternion orient, float xPixel, float yPixel) {
    float xRadian, yRadian;
    //pixelToRadians(cam, xPixel/2, yPixel/2, xRadian, yRadian);
    xRadian = sin(cam->getOgreCamera()->getFOVy().valueRadians()*.5) * cam->getOgreCamera()->getAspectRatio() * xPixel;
    yRadian = sin(cam->getOgreCamera()->getFOVy().valueRadians()*.5) * yPixel;

    return Vector3f(-orient.zAxis()*cos(cam->getOgreCamera()->getFOVy().valueRadians()*.5) +
                    orient.xAxis() * xRadian +
                    orient.yAxis() * yRadian);
}


void rotateCamera(CameraEntity *camera, float radianX, float radianY) {
    Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());

    Quaternion orient(camera->getProxy().globalLocation(now).getOrientation());
    Quaternion dragStart (camera->getProxy().extrapolateLocation(now).getOrientation());
    Quaternion dhorient = Quaternion(Vector3f(0,1,0),radianX);
    Quaternion dvorient = Quaternion(dhorient * orient * Vector3f(1,0,0),-radianY);

    Location location = camera->getProxy().extrapolateLocation(now);
    location.setOrientation((dvorient * dhorient * dragStart).normal());
    camera->getProxy().resetLocation(now, location);
}

void panCamera(CameraEntity *camera, const Vector3d &oldLocalPosition, const Vector3d &toPan) {
    Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());

    Quaternion orient(camera->getProxy().globalLocation(now).getOrientation());

    Location location (camera->getProxy().extrapolateLocation(now));
    location.setPosition(orient * toPan + oldLocalPosition);
    camera->getProxy().resetLocation(now, location);
}


////////////////////////////////////////////////////////////////////////////////
// MoveObjectDrag
////////////////////////////////////////////////////////////////////////////////

class MoveObjectDrag : public ActiveDrag {
    std::vector<ProxyObjectWPtr> mSelectedObjects;
    std::vector<Vector3d> mPositions;
    CameraEntity *camera;
    OgreSystem *mParent;
    Vector3d mMoveVector;
public:
    // Constructor
    MoveObjectDrag(const DragStartInfo &info)
        : mSelectedObjects(info.objects.begin(), info.objects.end()) {
        camera = info.camera;
        mParent = info.sys;
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        float moveDistance = 0.f; // Will be reset on first foundObject
        bool foundObject = false;
        Location cameraLoc = camera->getProxy().globalLocation(now);
        Vector3f cameraAxis = -cameraLoc.getOrientation().zAxis();
        mMoveVector = Vector3d(0,0,0);

        // Get initial positions for all of the objects.
        // Find the closest one, and compute the vector to it from the camera (mMoveVector).
        for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
            ProxyObjectPtr obj (mSelectedObjects[i].lock());
            if (!obj) {
                mPositions.push_back(Vector3d(0,0,0));
                continue;
            }
            mPositions.push_back(obj->extrapolateLocation(now).getPosition());
            Vector3d deltaPosition (obj->globalLocation(now).getPosition() - cameraLoc.getPosition());
            double dist = deltaPosition.dot(Vector3d(cameraAxis));
            if (!foundObject || dist < moveDistance) {
                foundObject = true;
                moveDistance = dist;
                mMoveVector = deltaPosition;
            }
        }
        SILOG(input,insane,"moveSelection: Moving selected objects at distance " << mMoveVector);
    }

    void mouseMoved(MouseDragEventPtr ev) {
        std::cout << "MOVE: mX = "<<ev->mX<<"; mY = "<<ev->mY<<". mXStart = "<< ev->mXStart<<"; mYStart = "<<ev->mYStart<<std::endl;
        if (mSelectedObjects.empty()) {
            SILOG(input,insane,"moveSelection: Found no selected objects");
            return;
        }
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());

        Vector3d toMove(0,0,0);
        double sensitivity = 20.0;
        Location cameraLoc = camera->getProxy().globalLocation(now);
        Vector3f cameraAxis = -cameraLoc.getOrientation().zAxis();

        // Constrained motion, only along coordinate axes.
        /// dbm new way: ignore camera, just move along global axes
        if (mParent->getInputManager()->isModifierDown(Input::MOD_ALT)) sensitivity = 5.0;  // Alt cranks up the constrained speed
        if (mParent->getInputManager()->isModifierDown(Input::MOD_SHIFT &&
                mParent->getInputManager()->isModifierDown(Input::MOD_CTRL))) {             // Control-shift constrains to Y
            toMove.y = ev->deltaY()*sensitivity;
        }
        else if (mParent->getInputManager()->isModifierDown(Input::MOD_SHIFT)) {            // Shift constrains to X
            if (cameraAxis.z > 0) sensitivity *=-1;
            toMove.x = ev->deltaX()*sensitivity;
        }
        else if (mParent->getInputManager()->isModifierDown(Input::MOD_CTRL)) {             // Control constrains to Z
            if (cameraAxis.x < 0) sensitivity *=-1;
            toMove.z = ev->deltaX()*sensitivity;
        }

        // Unconstrained motion.
        else {                                                                              // Move in a plane parallel to the viewport
            Vector3d startAxis (pixelToDirection(camera, cameraLoc.getOrientation(), ev->mXStart, ev->mYStart));    // Start direction
            Vector3d endAxis (pixelToDirection(camera, cameraLoc.getOrientation(), ev->mX, ev->mY));                // End direction
            Vector3d start, end;
            float moveDistance = mMoveVector.dot(Vector3d(cameraAxis));                     // Distnce to the object in the view direction.
            start = startAxis * moveDistance; // / cameraAxis.dot(startAxis);
            end = endAxis * moveDistance; // / cameraAxis.dot(endAxis);
            toMove = (end - start);                                                         // Motion vector
            // Prevent moving outside of a small radius so you don't shoot an object into the horizon.
            if (toMove.length() > 10*mParent->getInputManager()->mWorldScale->as<float>()) {
                // moving too much.
                toMove *= (10*mParent->getInputManager()->mWorldScale->as<float>()/toMove.length());
            }
        }

        // Add the "toMove" offset to each object
        for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
            ProxyObjectPtr obj (mSelectedObjects[i].lock());
            if (obj) {
                Location toSet (obj->extrapolateLocation(now));
                SILOG(input,debug,"moveSelection: OLD " << toSet.getPosition());
                toSet.setPosition(mPositions[i] + toMove);
                SILOG(input,debug,"moveSelection: NEW " << toSet.getPosition());
                obj->setLocation(now, toSet);
            }
        }
    }
};
DragActionRegistry::RegisterClass<MoveObjectDrag> moveobj("moveObject");


////////////////////////////////////////////////////////////////////////////////
// RotateObjectDrag
////////////////////////////////////////////////////////////////////////////////

class RotateObjectDrag : public ActiveDrag {
    OgreSystem *mParent;
    std::vector<ProxyObjectWPtr> mSelectedObjects;
    std::vector<Quaternion > mOriginalRotation;
    std::vector<Vector3d> mOriginalPosition;
    CameraEntity *camera;
public:
    RotateObjectDrag(const DragStartInfo &info)
            : mParent (info.sys),
            mSelectedObjects (info.objects.begin(), info.objects.end()) {
        camera = info.camera;
        mOriginalRotation.reserve(mSelectedObjects.size());
        mOriginalPosition.reserve(mSelectedObjects.size());
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
            ProxyObjectPtr obj (mSelectedObjects[i].lock());
            if (obj) {
                Location currentLoc = obj->extrapolateLocation(now);
                mOriginalRotation.push_back(currentLoc.getOrientation());
                mOriginalPosition.push_back(currentLoc.getPosition());
            } else {
                mOriginalRotation.push_back(Quaternion::identity());
                mOriginalPosition.push_back(Vector3d::nil());
            }
        }
    }
    void mouseMoved(MouseDragEventPtr ev) {
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        
        Location cameraLoc = camera->getProxy().globalLocation(now);
        Vector3f cameraAxis = -cameraLoc.getOrientation().zAxis();
        float radianX = 0;
        float radianY = 0;
        float radianZ = 0;
        float sensitivity = 0.25;
        Vector3d avgPos(0,0,0);
        for (size_t i = 0; i< mSelectedObjects.size(); ++i) {
            ProxyObjectPtr ent (mSelectedObjects[i].lock());
            Location loc (ent->extrapolateLocation(now));
            avgPos += loc.getPosition();
        }
        avgPos /= mSelectedObjects.size();

        int ctlX, ctlZ;
        if ((cameraAxis.x > 0 && cameraAxis.z > 0)
                || (cameraAxis.x <= 0 && cameraAxis.z <= 0) ) {
            ctlX = Input::MOD_SHIFT;
            ctlZ = Input::MOD_CTRL;
        }
        else {
            ctlX = Input::MOD_CTRL;
            ctlZ = Input::MOD_SHIFT;
        }
        if (mParent->getInputManager()->isModifierDown(Input::MOD_ALT)) {
            sensitivity = 0.1;
        }
        if (mParent->getInputManager()->isModifierDown(ctlX)) {
            if (mParent->getInputManager()->isModifierDown(ctlZ)) {
                radianZ = 3.14159 * 2 * -ev->deltaX() * sensitivity;
            }
            else {
                if (cameraAxis.z > 0) sensitivity *=-1;
            }
            radianX = 3.14159 * 2 * -ev->deltaY() * sensitivity;
        }
        else if (mParent->getInputManager()->isModifierDown(ctlZ)) {
            if (cameraAxis.x <= 0) sensitivity *=-1;
            radianZ = 3.14159 * 2 * -ev->deltaY() * sensitivity;
        }
        else {
            radianY = 3.14159 * 2 * ev->deltaX() * sensitivity;
        }
        Quaternion dragRotation (   Quaternion(Vector3f(1,0,0),radianX)*
                                    Quaternion(Vector3f(0,1,0),radianY)*
                                    Quaternion(Vector3f(0,0,1),radianZ));

        for (size_t i = 0; i< mSelectedObjects.size(); ++i) {
            ProxyObjectPtr ent (mSelectedObjects[i].lock());
            if (!ent) continue;
            Location loc (ent->extrapolateLocation(now));
            Vector3d localTrans = mOriginalPosition[i] - avgPos;
            loc.setPosition(avgPos + dragRotation*localTrans);
            loc.setOrientation(dragRotation*mOriginalRotation[i]);
            ent->resetLocation(now, loc);
        }
    }
};
DragActionRegistry::RegisterClass<RotateObjectDrag> rotateobj("rotateObject");


////////////////////////////////////////////////////////////////////////////////
// ScaleObjectDrag
////////////////////////////////////////////////////////////////////////////////

class ScaleObjectDrag : public RelativeDrag {
    OgreSystem *mParent;
    std::vector<ProxyObjectWPtr> mSelectedObjects;
    float dragMultiplier;
    std::vector<Vector3d> mOriginalPosition;
    float mTotalScale;
    CameraEntity *camera;
public:
    ScaleObjectDrag(const DragStartInfo &info)
            : RelativeDrag(info.ev->getDevice()),
            mParent (info.sys),
            mSelectedObjects (info.objects.begin(), info.objects.end()),
            mTotalScale(1.0) {
        camera = info.camera;
        mOriginalPosition.reserve(mSelectedObjects.size());
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
            ProxyObjectPtr obj(mSelectedObjects[i].lock());
            Location currentLoc = obj->extrapolateLocation(now);
            mOriginalPosition.push_back(currentLoc.getPosition());
        }
        dragMultiplier = mParent->getInputManager()->mDragMultiplier->as<float>();
    }
    void mouseMoved(MouseDragEventPtr ev) {
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        Vector3d avgPos(0,0,0);
        if (ev->deltaLastY() != 0) {
            float scaleamt = exp(dragMultiplier*ev->deltaLastY());
            mTotalScale *= scaleamt;
            int count = 0;
            for (size_t i = 0; i< mSelectedObjects.size(); ++i) {
                ProxyObjectPtr ent (mSelectedObjects[i].lock());
                if (!ent) {
                    continue;
                }
                Location loc (ent->extrapolateLocation(now));
                avgPos += loc.getPosition();
                ++count;
            }
            if (!count) return;
            avgPos /= count;
            for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
                ProxyObjectPtr ent (mSelectedObjects[i].lock());
                if (!ent) {
                    continue;
                }
                Location loc (ent->extrapolateLocation(now));
                Vector3d localTrans = mOriginalPosition[i] - avgPos;
                loc.setPosition(avgPos + localTrans*mTotalScale);
                std::cout << "debug avgPos: " << avgPos << " localTrans" << localTrans << " scale: " << mTotalScale << std::endl;
                ent->resetLocation(now, loc);
                std::tr1::shared_ptr<ProxyMeshObject> meshptr (
                    std::tr1::dynamic_pointer_cast<ProxyMeshObject>(ent));
                if (meshptr) {
                    meshptr->setScale(meshptr->getScale() * scaleamt);
                }
            }
        }
        if (ev->deltaLastX() != 0) {
            //rotateXZ(AxisValue::fromCentered(ev->deltaLastX()));
            //rotateCamera(mParent->mPrimaryCamera, ev->deltaLastX() * AXIS_TO_RADIANS, 0);
        }
    }
};
DragActionRegistry::RegisterClass<ScaleObjectDrag> scaleobj("scaleObject");


////////////////////////////////////////////////////////////////////////////////
// RotateCameraDrag
////////////////////////////////////////////////////////////////////////////////

class RotateCameraDrag : public RelativeDrag {
    CameraEntity *camera;
public:
    RotateCameraDrag(const DragStartInfo &info): RelativeDrag(info.ev->getDevice()) {
        camera = info.camera;
    }
    void mouseMoved(MouseDragEventPtr mouseev) {
        float radianX, radianY;
        pixelToRadians(camera, 2*mouseev->deltaLastX(), 2*mouseev->deltaLastY(), radianX, radianY);
        rotateCamera(camera, radianX, radianY);
    }
};
DragActionRegistry::RegisterClass<RotateCameraDrag> rotatecam("rotateCamera");


////////////////////////////////////////////////////////////////////////////////
// PanCameraDrag
////////////////////////////////////////////////////////////////////////////////

class PanCameraDrag : public ActiveDrag {
    Vector3d mStartPan;
    CameraEntity *camera;
    bool mRelativePan;
    double mPanDistance;
    OgreSystem *mParent;
    Vector3f toMove;
public:
    PanCameraDrag(const DragStartInfo &info) {
        int hitCount=0;
        camera = info.camera;
        mParent = info.sys;
        double distance;
        Vector3f normal;
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        Location cameraLoc = camera->getProxy().globalLocation(now);
        toMove = Vector3f(
            pixelToDirection(camera, cameraLoc.getOrientation(), info.ev->mXStart, info.ev->mYStart));
        mRelativePan = false;
        mStartPan = camera->getProxy().extrapolateLocation(now).getPosition();
		if (mParent->getInputManager()->isModifierDown(Input::MOD_CTRL)) {
			float WORLD_SCALE = mParent->getInputManager()->mWorldScale->as<float>();
            mPanDistance = WORLD_SCALE;
		} else if (!mParent->getInputManager()->isModifierDown(Input::MOD_SHIFT) &&
				   info.sys->rayTrace(cameraLoc.getPosition(), toMove, hitCount, distance, normal)) {
            mPanDistance = distance;
        } else if (!info.objects.empty()) {
            Vector3d totalPosition(averageSelectedPosition(now, info.objects.begin(), info.objects.end()));

            mPanDistance = (totalPosition - cameraLoc.getPosition()).length();
        } else {
            float WORLD_SCALE = mParent->getInputManager()->mWorldScale->as<float>();
            mPanDistance = WORLD_SCALE;
        }
    }
    void mouseMoved(MouseDragEventPtr ev) {
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        Location cameraLoc = camera->getProxy().globalLocation(now);
        if (mPanDistance) {
            float radianX, radianY;
            pixelToRadians(camera, ev->deltaX(), ev->deltaY(), radianX, radianY);
            panCamera(camera, mStartPan, Vector3d(-radianX*mPanDistance, -radianY*mPanDistance, 0));
        }
    }
};
DragActionRegistry::RegisterClass<PanCameraDrag> pancamera("panCamera");


////////////////////////////////////////////////////////////////////////////////
// ZoomCameraDrag
////////////////////////////////////////////////////////////////////////////////

void zoomInOut(Input::AxisValue value, const Input::InputDevicePtr &dev, CameraEntity *camera, const std::set<ProxyObjectWPtr>& objects, OgreSystem *parent) {
    if (!dev) return;
    float floatval = value.getCentered();
    Vector2f axes(
        dev->getAxis(Input::AXIS_CURSORX).getCentered(),
        dev->getAxis(Input::AXIS_CURSORY).getCentered()
    );
    zoomInOut(floatval, axes, camera, objects, parent);
}

void zoomInOut(float value, const Vector2f& axes, CameraEntity *camera, const std::set<ProxyObjectWPtr>& objects, OgreSystem *parent) {
    SILOG(input,debug,"zoom "<<value);

    Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
    Location cameraLoc = camera->getProxy().globalLocation(now);
    Vector3d toMove;

    toMove = Vector3d(pixelToDirection(camera, cameraLoc.getOrientation(), axes.x, axes.y));

    double distance;
    Vector3f normal;
    float WORLD_SCALE = parent->getInputManager()->mWorldScale->as<float>();
    int hitCount=0;
    if (!parent->getInputManager()->isModifierDown(Input::MOD_CTRL) &&
        !parent->getInputManager()->isModifierDown(Input::MOD_SHIFT)) {
        toMove *= WORLD_SCALE;
    } else if (parent->rayTrace(cameraLoc.getPosition(), direction(cameraLoc.getOrientation()), hitCount, distance, normal) &&
               (distance*.75 < WORLD_SCALE || parent->getInputManager()->isModifierDown(Input::MOD_SHIFT))) {
        toMove *= distance*.75;
    } else if (!objects.empty()) {
        Vector3d totalPosition (averageSelectedPosition(now, objects.begin(), objects.end()));
        toMove *= (totalPosition - cameraLoc.getPosition()).length() * .75;
    } else {
        toMove *= WORLD_SCALE;
    }
    toMove *= value; // up == zoom in
    cameraLoc.setPosition(cameraLoc.getPosition() + toMove);
    camera->getProxy().resetLocation(now, cameraLoc);
}

class ZoomCameraDrag : public RelativeDrag {
    OgreSystem *mParent;
    CameraEntity *mCamera;
    std::set<ProxyObjectWPtr> mSelection;
public:
    ZoomCameraDrag(const DragStartInfo &info)
        : RelativeDrag(info.ev->getDevice()) {
        mParent = info.sys;
        mCamera = info.camera;
        mSelection = info.objects;
    }
    void mouseMoved(MouseDragEventPtr ev) {
        if (ev->deltaLastY() != 0) {
            float dragMultiplier = mParent->getInputManager()->mDragMultiplier->as<float>();
            zoomInOut(AxisValue::fromCentered(dragMultiplier*ev->deltaLastY()), ev->getDevice(), mCamera, mSelection, mParent);
        }
    }
};
DragActionRegistry::RegisterClass<ZoomCameraDrag> zoomCamera("zoomCamera");


////////////////////////////////////////////////////////////////////////////////
// OrbitObjectDrag
////////////////////////////////////////////////////////////////////////////////

/*
    void orbitObject_BROKEN(AxisValue value) {
        SILOG(input,debug,"rotate "<<value);

        if (mSelectedObjects.empty()) {
            SILOG(input,debug,"rotateXZ: Found no selected objects");
            return;
        }
        CameraEntity *camera = mParent->mPrimaryCamera;
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());

        Location cameraLoc = camera->getProxy().globalLocation(now);
        Vector3d totalPosition (averageSelectedPosition(now));
        Vector3d distance (cameraLoc.getPosition() - totalPosition);

        float radianX = value.getCentered() * 1.0;
        Quaternion dhorient = Quaternion(Vector3f(0,1,0), radianX);
        distance = dhorient * distance;
        Quaternion dhorient2 = Quaternion(Vector3f(0,1,0), -radianX);
        cameraLoc.setPosition(totalPosition + dhorient * distance);
        cameraLoc.setOrientation(dhorient2 * cameraLoc.getOrientation());
        camera->getProxy().resetLocation(now, cameraLoc);
    }

 */

class OrbitObjectDrag : public RelativeDrag {
    OgreSystem *mParent;
    CameraEntity *camera;
    std::vector<ProxyObjectWPtr> mSelectedObjects;
//    Vector3d mOrbitCenter;
//    Vector3d mOriginalPosition;
public:
    OrbitObjectDrag(const DragStartInfo &info)
        : RelativeDrag(info.ev->getDevice()),
         mSelectedObjects(info.objects.begin(), info.objects.end()) {
        mParent = info.sys;
        camera = info.camera;
    }
    void mouseMoved(MouseDragEventPtr ev) {
        double distance;
        Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
        Location cameraLoc = camera->getProxy().globalLocation(now);
        Vector3d amount (ev->deltaX(), ev->deltaY(), 0);
/*
        Vector3f toMove (
            pixelToDirection(camera, cameraLoc.getOrientation(),
                             dev->getAxis(PointerDevice::CURSORX).getCentered(),
                             dev->getAxis(PointerDevice::CURSORY).getCentered()));
        if (mParent->rayTrace(cameraLoc.getPosition(), toMove, normal distance)) {
            rotateCamera(camera, amount.x, amount.y);
            panCamera(camera, amount * distance);
        } else */
        if (!mSelectedObjects.empty()) {
            Vector3d totalPosition (averageSelectedPosition(now, mSelectedObjects.begin(), mSelectedObjects.end()));
            double multiplier = (totalPosition - cameraLoc.getPosition()).length();
            rotateCamera(camera, amount.x, amount.y);
            panCamera(camera, camera->getProxy().extrapolateLocation(now).getPosition(), amount * multiplier);
        }
    }
};

DragActionRegistry::RegisterClass<OrbitObjectDrag> orbit("orbitObject");


////////////////////////////////////////////////////////////////////////////////
// Plane
////////////////////////////////////////////////////////////////////////////////

class Plane : public Vector4d {
public:
    // Set a plane from a vector and a point on the plane.
    // FIXME: There should be different classes for vectors and points.
    void set(const Vector3d &nrml, const Vector3d &point) {
        normal() = nrml;
        normal().normalizeThis();
        w = -normal().dot(point);
    }

    // This interface uses a single precision normal and a double precision point.
    void set(const Vector3f &nrml, const Vector3d &point) {
        set(Vector3d(nrml.x, nrml.y, nrml.z), point);
    }
    
    // This gets a single precision normal.
    void getNormal(Vector3f *nrml) const {
        nrml->x = x;
        nrml->y = y;
        nrml->z = z;
    }

    // Return the normal to the plane.
    const Vector3d& normal() const {
        return reinterpret_cast<const Vector3d&>(x);
    }

    // Move a plane in the direction of its normal.
    void parallelTransport(double distance) {
        w -= distance;
    }
    
    // Return the distance of the point to the plane.
    double distance(const Vector3d &point) const {
        return x * point.x + y * point.y + z * point.z + w;
    }

    // Intersect the ray (origin, direction) with the plane and return the intersection point.
    // We assume that the ray direction has been normalized.
    // False is returned if the ray is parallel to the plane.
    bool intersectRay(const Vector3d &origin, const Vector3d &direction, Vector3d *point) const {
        double f = normal().dot(direction);
        if (f == 0)
            return false;   // Ray is parallel to plane, and either has no intersection or an infinity of intersections.
        *point = origin - (direction * distance(origin) / f);
        return true;
    }

private:    
    // Return a mutable reference to the normal of the plane.
    Vector3d& normal() { return reinterpret_cast<Vector3d&>(x); }
};


////////////////////////////////////////////////////////////////////////////////
// MoveObjectOnWallDrag
////////////////////////////////////////////////////////////////////////////////

class MoveObjectOnWallDrag : public ActiveDrag {
public:
    // Constructor
    MoveObjectOnWallDrag(const DragStartInfo &info);

    // Dragger
    void mouseMoved(MouseDragEventPtr ev);

private:
    // Return the plane of the wall, in the direction of the camera.
    // If no wall is found, return false.
    bool getPlaneOfWall(const Vector3d &viewDirection, Plane *plane) const;
    
    // Check to see if this object is in the list of those to be moved.
    bool isObjectToBeMoved(const Entity *obj) const;
    
    std::vector<ProxyObjectWPtr> mSelectedObjects;
    std::vector<Vector3d> mPositions;
    std::vector<Quaternion> mOrientations;
    CameraEntity *camera;
    OgreSystem *mParent;
    Vector3d mVectorToObject;   // Vector from the camera to the object
    Location mCameraLocation;
    float mDistanceFrontOfWall;  // The distance of the object from the wall.
    Vector3d mStartPosition;
    Quaternion mStartOrientation;
};


MoveObjectOnWallDrag::MoveObjectOnWallDrag(const DragStartInfo &info)
    : mSelectedObjects(info.objects.begin(), info.objects.end())
{
    camera = info.camera;
    mParent = info.sys;
    Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
    float distanceToObject = 0.f; // Will be reset on first foundObject
    bool foundObject = false;
    const float kDistanceFromWall = 1;
    mDistanceFrontOfWall = kDistanceFromWall;
    
    mCameraLocation = camera->getProxy().globalLocation(now);
    Vector3f cameraAxis = -mCameraLocation.getOrientation().zAxis();
    mVectorToObject = Vector3d(0,0,0);

    // Get initial positions for all of the objects.
    // Find the closest one, and compute the vector to it from the camera (mVectorToObject).
    for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
        ProxyObjectPtr obj (mSelectedObjects[i].lock());    // This is unlocked when obj goes out of scope.
        if (!obj) {
            mPositions.push_back(Vector3d(0,0,0));  // FIXME: What purpose does this serve? A placeholder, maybe?
            continue;
        }
        mPositions.push_back(obj->extrapolateLocation(now).getPosition());
        mOrientations.push_back(obj->extrapolateLocation(now).getOrientation());
        Vector3d objPosition = obj->globalLocation(now).getPosition();
        Vector3d deltaPosition (objPosition - mCameraLocation.getPosition());
        double dist = deltaPosition.dot(Vector3d(cameraAxis));
        if (!foundObject || dist < distanceToObject) {
            foundObject = true;
            distanceToObject = dist;
            mVectorToObject = deltaPosition;
            mStartPosition = objPosition;
            mStartOrientation = obj->globalLocation(now).getOrientation();
        }
    }
    SILOG(input,insane,"moveSelection: Moving selected objects at distance " << mVectorToObject);
}


void MoveObjectOnWallDrag::mouseMoved(MouseDragEventPtr ev) {
    if (mSelectedObjects.empty()) {
        SILOG(input,insane,"moveSelection: Found no selected objects");
        return;
    }

    Time now = SpaceTimeOffsetManager::getSingleton().now(camera->getProxy().getObjectReference().space());
    mCameraLocation = camera->getProxy().globalLocation(now); // Camera doesn't move
    Vector3f cameraAxis = -mCameraLocation.getOrientation().zAxis();
    Vector3d startVec(pixelToDirection(camera, mCameraLocation.getOrientation(), ev->mXStart, ev->mYStart));
    Vector3d   endVec(pixelToDirection(camera, mCameraLocation.getOrientation(), ev->mX,      ev->mY));
    Plane startPlane, endPlane;
    if (!getPlaneOfWall(startVec, &startPlane)) {
        SILOG(input, error, "MoveObjectOnWall: no start wall");
        return;
    }

    // Compute end location
    if (!getPlaneOfWall(endVec, &endPlane)) {
        SILOG(input, error, "MoveObjectOnWall: no end wall");
        return;
    }
    endPlane.parallelTransport(mDistanceFrontOfWall);       // Plane where picture should lie, offset a given distance from the wall
    Vector3d endPosition;
    if (!endPlane.intersectRay(mCameraLocation.getPosition(), endVec, &endPosition))
        return; // Ray is parallel to plane

    // Compute translation
    Vector3d translation = endPosition - mStartPosition;
    
    // Compute rotation
    Vector3f xAxis, yAxis, zAxis;
    endPlane.getNormal(&zAxis);
    xAxis = Vector3f::unitY().cross(zAxis);
    yAxis = zAxis.cross(xAxis);
    Quaternion rotation(xAxis, yAxis, zAxis);
    rotation = mStartOrientation.inverse() * rotation;
    std::cout   << "MOVE: mX = " << ev->mX
                << "; mY = " << ev->mY
                << ". mXStart = " << ev->mXStart
                << "; mYStart = " << ev->mYStart
                << "; trans = " << translation
                << "; rot = " << rotation
                << std::endl;

    // Apply the translation and rotation to each object
    for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
        ProxyObjectPtr obj (mSelectedObjects[i].lock());
        if (obj) {
            Location toSet (obj->extrapolateLocation(now));
            toSet.setPosition(mPositions[i] + translation);
            toSet.setOrientation(mOrientations[i] * rotation);
            obj->setLocation(now, toSet);
        }
    }
}


bool MoveObjectOnWallDrag::getPlaneOfWall(const Vector3d &viewDirection, Plane *plane) const {
    Vector3d dViewPosition  =  mCameraLocation.getPosition();
    Vector3f fViewDirection(viewDirection.x, viewDirection.y, viewDirection.z);
    int numHits = 1, i;

    for (i = 0; i < numHits; i++) {
        double distance;        // Distance along the ray
        Vector3f normal;
        const Entity *obj = mParent->rayTrace(dViewPosition, fViewDirection, numHits, distance, normal, i);
        if (obj == NULL) {      // No object found
            if (i < numHits)    // Why not?
                continue;       // Still more objects: keep looking
            break;              // No more objects: return failure
        }
        if (!isObjectToBeMoved(obj)) {
            Vector3d surfacePoint = dViewPosition + distance * viewDirection;
            if (fViewDirection.dot(normal) > 0) // Normal is pointing away from the camera
                normal = -normal;               // Get normal pointing toward the camera
            plane->set(normal, surfacePoint);
#ifdef DEBUG
            Vector3d intPt;
            plane->intersectRay(mCameraLocation.getPosition(), viewDirection, &intPt);
            std::cout   << "viewDir = " << viewDirection
                        << "; plane = " << *plane
                        << "; intersection = " << intPt
                        << std::endl;
#endif // DEBUG
           return true;
        }
    }
    
    return false;
}


bool MoveObjectOnWallDrag::isObjectToBeMoved(const Entity *testEntity) const {
    ProxyObjectPtr testObject(testEntity->getProxyPtr());
    for (size_t i = 0; i < mSelectedObjects.size(); ++i) {
        ProxyObjectPtr obj(mSelectedObjects[i].lock());
        if (obj && testObject.get() == obj.get())
            return true;
    }
    return false;
}


DragActionRegistry::RegisterClass<MoveObjectOnWallDrag> moveobjectonwall("moveObjectOnWall");


} // namespace Graphics
} // namespace Sirikata
