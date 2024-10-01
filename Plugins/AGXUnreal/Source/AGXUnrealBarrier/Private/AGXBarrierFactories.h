// Copyright 2024, Algoryx Simulation AB.

#pragma once

#include "RigidBodyBarrier.h"
#include "Shapes/AnyShapeBarrier.h"
#include "Shapes/SphereShapeBarrier.h"
#include "Shapes/BoxShapeBarrier.h"
#include "Shapes/CylinderShapeBarrier.h"
#include "Shapes/CapsuleShapeBarrier.h"
#include "Shapes/EmptyShapeBarrier.h"
#include "Shapes/TrimeshShapeBarrier.h"
#include "Constraints/AnyConstraintBarrier.h"
#include "Constraints/HingeBarrier.h"
#include "Constraints/PrismaticBarrier.h"
#include "Constraints/BallJointBarrier.h"
#include "Constraints/CylindricalJointBarrier.h"
#include "Constraints/DistanceJointBarrier.h"
#include "Constraints/LockJointBarrier.h"
#include "Constraints/TwistRangeControllerBarrier.h"
#include "Contacts/ShapeContactBarrier.h"
#include "Contacts/ContactPointBarrier.h"
#include "Materials/ShapeMaterialBarrier.h"
#include "Materials/ContactMaterialBarrier.h"
#include "Materials/TerrainMaterialBarrier.h"
#include "Tires/TwoBodyTireBarrier.h"
#include "Terrain/ShovelBarrier.h"
#include "Wire/WireBarrier.h"
#include "Wire/WireRef.h"
#include "Wire/WireNodeBarrier.h"
#include "Wire/WireNodeRef.h"
#include "Wire/WireWinchBarrier.h"
#include "Wire/WireWinchRef.h"
#include "Vehicle/TrackBarrier.h"

class FTerrainBarrier;

namespace agx
{
	class RigidBody;
	class Constraint;
	class Hinge;
	class Prismatic;
	class BallJoint;
	class CylindricalJoint;
	class DistanceJoint;
	class TwistRangeController;
}

namespace agxCollide
{
	class Box;
	class Cylinder;
	class Capsule;
	class Shape;
	class Sphere;
	class Trimesh;

	class GeometryContact;
	class ContactPoint;
}

namespace agxModel
{
	class TwoBodyTire;
}

namespace agxTerrain
{
	class Shovel;
	class Terrain;
	class TerrainMaterial;
}

namespace agxWire
{
	class Wire;
	class Node;
	class WireWinchController;
}

namespace agxVehicle
{
	class Track;
}

/**
 * These factory functions are required because it's not possible to create a Barrier constructor
 * that takes the AGX Dynamics type as a parameter because we may not use AGX Dynamics types in
 * public header files. This header file is private to the Barrier module so here we can use AGX
 * Dynamics types. The factory functions sidestep the limitation by passing a unique_ptr to an
 * instance of the forward-declared Refs-type to the Barrier constructor, which the constructor
 * implementation will move from.
 */
namespace AGXBarrierFactories
{
	FRigidBodyBarrier AGXUNREALBARRIER_API CreateRigidBodyBarrier(agx::RigidBody* Body);

	FEmptyShapeBarrier CreateEmptyShapeBarrier(agxCollide::Geometry* Geometry);

	FAnyShapeBarrier CreateAnyShapeBarrier(agxCollide::Shape* Shape);

	FSphereShapeBarrier CreateSphereShapeBarrier(agxCollide::Sphere* Sphere);

	FBoxShapeBarrier CreateBoxShapeBarrier(agxCollide::Box* Box);

	FCylinderShapeBarrier CreateCylinderShapeBarrier(agxCollide::Cylinder* Cylinder);

	FCapsuleShapeBarrier CreateCapsuleShapeBarrier(agxCollide::Capsule* Capsule);

	FTrimeshShapeBarrier CreateTrimeshShapeBarrier(agxCollide::Trimesh* Trimesh);

	FAnyConstraintBarrier CreateAnyConstraintBarrier(agx::Constraint* Constraint);

	FHingeBarrier CreateHingeBarrier(agx::Hinge* Hinge);

	FPrismaticBarrier CreatePrismaticBarrier(agx::Prismatic* Prismatic);

	FBallJointBarrier CreateBallJointBarrier(agx::BallJoint* BallJoint);

	FCylindricalJointBarrier CreateCylindricalJointBarrier(agx::CylindricalJoint* CylindricalJoint);

	FDistanceJointBarrier CreateDistanceJointBarrier(agx::DistanceJoint* DistanceJoint);

	FLockJointBarrier CreateLockJointBarrier(agx::LockJoint* LockJoint);

	FTwistRangeControllerBarrier CreateTwistRangeControllerBarrier(
		agx::TwistRangeController* Controller);

	FShapeMaterialBarrier CreateShapeMaterialBarrier(agx::Material* Material);

	FContactMaterialBarrier CreateContactMaterialBarrier(agx::ContactMaterial* ContactMaterial);

	FShapeContactBarrier CreateShapeContactBarrier(agxCollide::GeometryContact GeometryContact);

	FContactPointBarrier CreateContactPointBarrier(agxCollide::ContactPoint ContactPoint);

	FTwoBodyTireBarrier CreateTwoBodyTireBarrier(agxModel::TwoBodyTire* Tire);

	FTerrainBarrier CreateTerrainBarrier(agxTerrain::Terrain* Terrain);

	FTerrainMaterialBarrier CreateTerrainMaterialBarrier(agxTerrain::TerrainMaterial* Material);

	FWireBarrier CreateWireBarrier(agxWire::Wire* Wire);

	FWireNodeBarrier CreateWireNodeBarrier(agxWire::Node* Node);

	FWireWinchBarrier CreateWireWinchBarrier(agxWire::WireWinchController* Winch);

	FShovelBarrier CreateShovelBarrier(agxTerrain::Shovel* Shovel);

	FTrackBarrier CreateTrackBarrier(agxVehicle::Track* Track);
}
