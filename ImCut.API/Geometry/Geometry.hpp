#pragma once

// ImCut::Geometry - the central geometric kernel.
//
// Pure C++: no COM, no VGCore, no Runtime, no UI, no ImGui. Consumers depend on the
// kernel, never the other way round. The only component allowed to know about
// CorelDRAW is Geometry/Corel/CorelGeometryAdapter, which is not included here.
//
// Units are millimetres and coordinates are double throughout; conversion happens at
// the integration boundary only. See Geometry/README.md.

#include "GeometryContext.hpp"
#include "GeometryCertification.hpp"
#include "GeometryDiagnostics.hpp"
#include "GeometryResult.hpp"
#include "GeometrySession.hpp"
#include "GeometryTolerance.hpp"
#include "GeometryTypes.hpp"
#include "PreparedGeometry.hpp"
#include "Quantization.hpp"

#include "Math/Bounds2.hpp"
#include "Math/Predicates.hpp"
#include "Math/Transform2.hpp"
#include "Math/Vec2.hpp"

#include "Curves/CubicBezier.hpp"
#include "Curves/Flatten.hpp"

#include "Spatial/SegmentBVH.hpp"

#include "Intersection/Intersection.hpp"
#include "Intersection/SelfIntersection.hpp"

#include "Topology/ContainmentTree.hpp"
#include "Topology/Winding.hpp"

#include "Polygon/Boolean.hpp"
#include "Polygon/ConvexDecomposition.hpp"
#include "Polygon/DecompositionPortfolio.hpp"
#include "Polygon/ConvexHull.hpp"
#include "Polygon/Minkowski.hpp"
#include "Polygon/Nfp.hpp"
#include "Polygon/NfpAuto.hpp"
#include "Polygon/NfpBatch.hpp"
#include "Polygon/NfpCover.hpp"
#include "Polygon/NfpConvolution.hpp"
#include "Polygon/NfpHoleFilter.hpp"
#include "Polygon/NfpLattice.hpp"
#include "Polygon/NfpPrepared.hpp"
#include "Polygon/Offset.hpp"
#include "Polygon/PolygonMetrics.hpp"

#include "Canonical/CanonicalGeometry.hpp"
#include "Canonical/GeometryFingerprint.hpp"

#include "Distance/Collision.hpp"
#include "Distance/Distance.hpp"
#include "Distance/PreparedCollision.hpp"

#include "Validation/GeometryValidation.hpp"
