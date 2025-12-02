# Physics Properties Comparison: coco_one vs Isaac_Houndbot_End

This document compares the physical properties between `coco_one.usda` and `Isaac_Houndbot_End.usda` to identify differences that may require adjustment for proper simulation behavior.

---

## ✅ Modifications Applied (2024-12-02)

The following changes have been applied to `coco_one.usda` to match `Isaac_Houndbot_End.usda`:

| Property | Before | After | Location |
|----------|--------|-------|----------|
| physics:dynamicFriction | 0.6 | **0.7** | PhysicsMaterial |
| physics:staticFriction | 0.7 | **1.0** | PhysicsMaterial |
| physxMaterial:frictionCombineMode | "average" | **"max"** | PhysicsMaterial |
| Wheel drive:angular:physics:damping | 1,000,000 | **10,000** | All 4 wheel joints |
| Wheel drive:angular:physics:maxForce | inf | **9** | All 4 wheel joints |
| Wheel drive:angular:physics:stiffness | 0 | **100** | All 4 wheel joints |
| Wheel drive:angular:physics:type | "force" | **"acceleration"** | All 4 wheel joints |
| Shock drive:linear:physics:damping | 100,000 | **5,000** | All 4 shock joints |
| Steering drive:angular:physics:damping | 0 | **1,000** | base_to_front_axle_joint |
| Steering drive:angular:physics:stiffness | 1,000,000 | **10,000,000** | base_to_front_axle_joint |
| physxArticulation:solverVelocityIterationCount | 1 | **16** | Articulation root |

**Backup file:** `coco_one.usda.backup`

---

## 1. Articulation Root Configuration

| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| Location | `/Root/coco_one/coco_one/base_link` | `/Root/Isaac_Houndbot_End` (top-level) |
| API Schemas | PhysxRigidBodyAPI, PhysicsRigidBodyAPI, PhysicsMassAPI, PhysicsArticulationRootAPI, PhysxArticulationAPI, PhysxContactReportAPI | PhysicsArticulationRootAPI, PhysxArticulationAPI |
| `solverPositionIterationCount` | 32 (articulation), 4 (rigid body) | Not specified (default) |
| `solverVelocityIterationCount` | 1 (articulation), 0 (rigid body) | 16 |
| `articulationEnabled` | Not explicitly set | true |
| `enabledSelfCollisions` | false | false |

### ⚠️ Recommendation
- **coco_one** has higher position iteration count (32) which may provide more stable joint solutions but at computational cost
- **Isaac_Houndbot_End** has higher velocity iteration count (16) which improves velocity-based constraint accuracy

---

## 2. Mass and Inertia Properties

### coco_one Mass Distribution
| Component | Mass (kg) | Diagonal Inertia |
|-----------|-----------|------------------|
| base_link | 50 | (2.5, 2.5, 2.5) |
| body_link | 25 | (2.5, 2.5, 2.5) |
| wheel_link (each) | 1 | (0.1, 0.1, 0.1) |
| shock_link (each) | 0.000001 | (0.00001, 0.00001, 0.00001) |
| axle_link (each) | 0.000001 | (0.00001, 0.00001, 0.00001) |

### Isaac_Houndbot_End Mass Distribution
| Component | Mass (kg) | Method |
|-----------|-----------|--------|
| Body parts | Uses `physics:density = 1800` | Density-based calculation |
| Wheel components | 4.32558, 1, 0.01, 5, 1.15 | Explicit mass values |

### ⚠️ Recommendation
- **coco_one** has explicit mass/inertia values making physics more predictable
- **Isaac_Houndbot_End** relies partially on density calculations
- Total estimated mass: coco_one ~79kg, Isaac_Houndbot_End varies by geometry

---

## 3. Physics Material (Friction)

| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| Material Name | `PhysicsMaterial` | `wheel_PhysicsMaterial` |
| Dynamic Friction | **0.6** | **0.7** |
| Static Friction | **0.7** | **1.0** |
| Restitution | 0 | Not specified |
| Friction Combine Mode | `average` | `max` |
| Restitution Combine Mode | `min` | Not specified |

### ⚠️ Recommendation
- **Isaac_Houndbot_End** has higher friction values (static=1.0, dynamic=0.7) which provides better traction
- **coco_one** may slip more easily due to lower friction values
- Consider increasing coco_one friction to match Isaac_Houndbot_End for similar traction behavior

---

## 4. Wheel Joint Drive Properties

### coco_one Wheel Joints
```
drive:angular:physics:damping = 1000000
drive:angular:physics:maxForce = inf
drive:angular:physics:stiffness = 0
drive:angular:physics:type = "force"
physxJoint:maxJointVelocity = 1000000
physxLimit:angular:damping = 0.3
```

### Isaac_Houndbot_End Wheel Joints
```
drive:angular:physics:damping = 10000
drive:angular:physics:maxForce = 9
drive:angular:physics:stiffness = 100
drive:angular:physics:type = "acceleration"
```

### ⚠️ Critical Differences
| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| Damping | 1,000,000 | 10,000 |
| Max Force | infinite | 9 |
| Stiffness | 0 | 100 |
| Drive Type | **force** | **acceleration** |
| Max Joint Velocity | 1,000,000 | Not specified |

### ⚠️ Recommendation
- **coco_one** uses velocity control (`stiffness=0`) with pure force mode
- **Isaac_Houndbot_End** uses position/velocity hybrid (`stiffness=100`) with acceleration mode
- The `acceleration` type in Isaac_Houndbot_End provides mass-independent control
- coco_one's extremely high damping (1M) may cause overdamped wheel response

---

## 5. Rigid Body Velocity Limits

| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| maxLinearVelocity | 1000 | Not specified (default) |
| maxAngularVelocity | 1000 | Not specified (default) |
| angularDamping | 0 | Not specified (default) |

### ⚠️ Recommendation
- **coco_one** explicitly sets velocity limits which prevents physics instability
- Consider adding similar limits to Isaac_Houndbot_End if experiencing simulation issues

---

## 6. Suspension/Shock Joint Configuration

### coco_one Shock Joints (PrismaticJoint)
```
drive:linear:physics:damping = 100000
drive:linear:physics:stiffness = 10000
physics:lowerLimit = 0
physics:upperLimit = 0.05
physxJoint:maxJointVelocity = 1
```

### Isaac_Houndbot_End Spring Joints (PrismaticJoint)
```
drive:linear:physics:damping = 5000
drive:linear:physics:stiffness = 10000
```

### ⚠️ Recommendation
- **coco_one** has 20x higher suspension damping (100000 vs 5000)
- This may cause stiffer, less responsive suspension in coco_one
- Consider reducing coco_one damping for smoother ride

---

## 7. Steering Joint Configuration

### coco_one (base_to_front_axle_joint)
```
drive:angular:physics:damping = 0
drive:angular:physics:stiffness = 1000000
physics:lowerLimit = -20.053522°
physics:upperLimit = 20.053522°
physxJoint:maxJointVelocity = 57.29578
```

### Isaac_Houndbot_End (joint_front_bar)
```
drive:angular:physics:damping = 1000
drive:angular:physics:stiffness = 10000000
physics:lowerLimit = -20°
physics:upperLimit = 20°
```

### ⚠️ Recommendation
- Both have similar steering angle limits (~20°)
- **Isaac_Houndbot_End** has 10x higher stiffness and non-zero damping for steering
- coco_one may have less responsive steering due to lower stiffness

---

## 8. Remaining Differences (Not Yet Applied)

### 8.1 Solver Iteration Counts

| Property | coco_one | Isaac_Houndbot_End | Status |
|----------|----------|-------------------|--------|
| physxArticulation:solverPositionIterationCount | 32 | Not specified (default) | Keep as-is |
| physxArticulation:solverVelocityIterationCount | ~~1~~ **16** | 16 | ✅ Applied |
| physxRigidBody:solverPositionIterationCount | 4 | Not specified | Keep as-is |
| physxRigidBody:solverVelocityIterationCount | 0 | Not specified | Keep as-is |

**Impact:** Higher velocity iteration count improves joint constraint accuracy during motion. ✅ Fixed

### 8.2 Break Force/Torque

| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| physics:breakForce | 3.4028235e38 (float max) | inf |
| physics:breakTorque | 3.4028235e38 (float max) | inf |

**Impact:** Both are effectively unbreakable. No change needed.

### 8.3 Joint State Properties

| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| physics:jointEnabled | Not specified | true (explicit) |
| drive:angular:physics:targetVelocity | Not specified | 0 (explicit) |

**Impact:** Minor - defaults should work correctly.

### 8.4 Wheel Joint Limit Damping

| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| physxLimit:angular:damping | 0.3 | Not specified |

**Impact:** coco_one has extra limit damping on wheel joints. May want to remove for consistency.

### 8.5 Velocity Limits

| Property | coco_one | Isaac_Houndbot_End |
|----------|----------|-------------------|
| physxRigidBody:maxLinearVelocity | 1000 | Not specified |
| physxRigidBody:maxAngularVelocity | 1000 | Not specified |
| physxRigidBody:angularDamping | 0 | Not specified |

**Impact:** coco_one has explicit velocity limits which can prevent instability. Keep as-is.

### 8.6 Collision Approximation

Both use `convexHull` for most parts. Isaac_Houndbot_End uses `convexDecomposition` for one body part.

**Impact:** No change needed - both are compatible.

---

## Summary

### ✅ Applied Fixes (Matching Isaac_Houndbot_End)
1. ~~Increase wheel friction~~ ✅ Done (static=1.0, dynamic=0.7)
2. ~~Change friction combine mode~~ ✅ Done ("max")
3. ~~Reduce wheel damping~~ ✅ Done (10,000)
4. ~~Change drive type~~ ✅ Done ("acceleration")
5. ~~Reduce shock damping~~ ✅ Done (5,000)
6. ~~Increase steering stiffness~~ ✅ Done (10,000,000)
7. ~~Add steering damping~~ ✅ Done (1,000)
8. ~~Increase solverVelocityIterationCount~~ ✅ Done (16)

### ⚠️ Optional Further Changes
9. Consider removing `physxLimit:angular:damping = 0.3` from wheel joints
10. Consider adding explicit `physics:jointEnabled = 1` to joints

