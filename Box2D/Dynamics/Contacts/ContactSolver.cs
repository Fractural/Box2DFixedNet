/*
  Box2DX Copyright (c) 2009 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

//#define B2_DEBUG_SOLVER

using Box2DX.Collision;
using Box2DX.Common;
using FixMath.NET;
using System;

namespace Box2DX.Dynamics
{
    public struct ContactConstraintPoint
    {
        public Vec2 LocalPoint;
        public Vec2 RA;
        public Vec2 RB;
        public Fix64 NormalImpulse;
        public Fix64 TangentImpulse;
        public Fix64 NormalMass;
        public Fix64 TangentMass;
        public Fix64 EqualizedMass;
        public Fix64 VelocityBias;
    }

    public class ContactConstraint
    {
        public ContactConstraintPoint[] Points = new ContactConstraintPoint[Settings.MaxManifoldPoints];
        public Vec2 LocalPlaneNormal;
        public Vec2 LocalPoint;
        public Vec2 Normal;
        public Mat22 NormalMass;
        public Mat22 K;
        public Body BodyA;
        public Body BodyB;
        public ManifoldType Type;
        public Fix64 Radius;
        public Fix64 Friction;
        public Fix64 Restitution;
        public int PointCount;
        public Manifold Manifold;

        //public ContactConstraint()
        //{
        //	for (int i = 0; i < Settings.MaxManifoldPoints; i++)
        //		Points[i] = new ContactConstraintPoint();
        //}
    }

    public class ContactSolver : IDisposable
    {
        public TimeStep _step;
        public ContactConstraint[] _constraints;
        public int _constraintCount;

        private static readonly Fix64 MAX_CONDITION_NUMBER = Fix64.From(100);

        public ContactSolver(TimeStep step, Contact[] contacts, int contactCount)
        {
            _step = step;
            _constraintCount = contactCount;

            _constraints = new ContactConstraint[_constraintCount];
            for (int i = 0; i < _constraintCount; i++)
                _constraints[i] = new ContactConstraint();

            int count = 0;
            for (int i = 0; i < _constraintCount; ++i)
            {
                Contact contact = contacts[i];

                Fixture fixtureA = contact._fixtureA;
                Fixture fixtureB = contact._fixtureB;
                Shape shapeA = fixtureA.Shape;
                Shape shapeB = fixtureB.Shape;
                Fix64 radiusA = shapeA._radius;
                Fix64 radiusB = shapeB._radius;
                Body bodyA = fixtureA.Body;
                Body bodyB = fixtureB.Body;
                Manifold manifold = contact.Manifold;

                Fix64 friction = Settings.MixFriction(fixtureA.Friction, fixtureB.Friction);
                Fix64 restitution = Settings.MixRestitution(fixtureA.Restitution, fixtureB.Restitution);

                Vec2 vA = bodyA._linearVelocity;
                Vec2 vB = bodyB._linearVelocity;
                Fix64 wA = bodyA._angularVelocity;
                Fix64 wB = bodyB._angularVelocity;

                Box2DXDebug.Assert(manifold.PointCount > 0);

                WorldManifold worldManifold = new WorldManifold();
                worldManifold.Initialize(manifold, bodyA._xf, radiusA, bodyB._xf, radiusB);

                ContactConstraint cc = _constraints[i];
                cc.BodyA = bodyA;
                cc.BodyB = bodyB;
                cc.Manifold = manifold;
                cc.Normal = worldManifold.Normal;
                cc.PointCount = manifold.PointCount;
                cc.Friction = friction;
                cc.Restitution = restitution;

                cc.LocalPlaneNormal = manifold.LocalPlaneNormal;
                cc.LocalPoint = manifold.LocalPoint;
                cc.Radius = radiusA + radiusB;
                cc.Type = manifold.Type;

                unsafe
                {
                    fixed (ContactConstraintPoint* ccPointsPtr = cc.Points)
                    {
                        for (int j = 0; j < cc.PointCount; ++j)
                        {
                            ManifoldPoint cp = manifold.Points[j];
                            ContactConstraintPoint* ccp = &ccPointsPtr[j];

                            ccp->NormalImpulse = cp.NormalImpulse;
                            ccp->TangentImpulse = cp.TangentImpulse;

                            ccp->LocalPoint = cp.LocalPoint;

                            ccp->RA = worldManifold.Points[j] - bodyA._sweep.C;
                            ccp->RB = worldManifold.Points[j] - bodyB._sweep.C;

                            Fix64 rnA = Vec2.Cross(ccp->RA, cc.Normal);
                            Fix64 rnB = Vec2.Cross(ccp->RB, cc.Normal);
                            rnA *= rnA;
                            rnB *= rnB;

                            Fix64 kNormal = bodyA._invMass + bodyB._invMass + bodyA._invI * rnA + bodyB._invI * rnB;

                            Box2DXDebug.Assert(kNormal > Common.Settings.FLT_EPSILON);
                            ccp->NormalMass = Fix64.One / kNormal;

                            Fix64 kEqualized = bodyA._mass * bodyA._invMass + bodyB._mass * bodyB._invMass;
                            kEqualized += bodyA._mass * bodyA._invI * rnA + bodyB._mass * bodyB._invI * rnB;

                            Box2DXDebug.Assert(kEqualized > Common.Settings.FLT_EPSILON);
                            ccp->EqualizedMass = Fix64.One / kEqualized;

                            Vec2 tangent = Vec2.Cross(cc.Normal, Fix64.One);

                            Fix64 rtA = Vec2.Cross(ccp->RA, tangent);
                            Fix64 rtB = Vec2.Cross(ccp->RB, tangent);
                            rtA *= rtA;
                            rtB *= rtB;

                            Fix64 kTangent = bodyA._invMass + bodyB._invMass + bodyA._invI * rtA + bodyB._invI * rtB;

                            Box2DXDebug.Assert(kTangent > Common.Settings.FLT_EPSILON);
                            ccp->TangentMass = Fix64.One / kTangent;

                            // Setup a velocity bias for restitution.
                            ccp->VelocityBias = Fix64.Zero;
                            Fix64 vRel = Vec2.Dot(cc.Normal, vB + Vec2.Cross(wB, ccp->RB) - vA - Vec2.Cross(wA, ccp->RA));
                            if (vRel < -Common.Settings.VelocityThreshold)
                            {
                                ccp->VelocityBias = -cc.Restitution * vRel;
                            }
                        }

                        // If we have two points, then prepare the block solver.
                        if (cc.PointCount == 2)
                        {
                            ContactConstraintPoint* ccp1 = &ccPointsPtr[0];
                            ContactConstraintPoint* ccp2 = &ccPointsPtr[1];

                            Fix64 invMassA = bodyA._invMass;
                            Fix64 invIA = bodyA._invI;
                            Fix64 invMassB = bodyB._invMass;
                            Fix64 invIB = bodyB._invI;

                            Fix64 rn1A = Vec2.Cross(ccp1->RA, cc.Normal);
                            Fix64 rn1B = Vec2.Cross(ccp1->RB, cc.Normal);
                            Fix64 rn2A = Vec2.Cross(ccp2->RA, cc.Normal);
                            Fix64 rn2B = Vec2.Cross(ccp2->RB, cc.Normal);

                            Fix64 k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
                            Fix64 k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
                            Fix64 k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;

                            // Ensure a reasonable condition number.
                            if (k11 * k11 < MAX_CONDITION_NUMBER * (k11 * k22 - k12 * k12))
                            {
                                // K is safe to invert.
                                cc.K.Col1.Set(k11, k12);
                                cc.K.Col2.Set(k12, k22);
                                cc.NormalMass = cc.K.GetInverse();
                            }
                            else
                            {
                                // The constraints are redundant, just use one.
                                // TODO_ERIN use deepest?
                                cc.PointCount = 1;
                            }
                        }
                    }
                }
            }
        }

        public void Dispose()
        {
            _constraints = null;
        }

        public void InitVelocityConstraints(TimeStep step)
        {
            unsafe
            {
                // Warm start.
                for (int i = 0; i < _constraintCount; ++i)
                {
                    ContactConstraint c = _constraints[i];

                    Body bodyA = c.BodyA;
                    Body bodyB = c.BodyB;
                    Fix64 invMassA = bodyA._invMass;
                    Fix64 invIA = bodyA._invI;
                    Fix64 invMassB = bodyB._invMass;
                    Fix64 invIB = bodyB._invI;
                    Vec2 normal = c.Normal;
                    Vec2 tangent = Vec2.Cross(normal, Fix64.One);

                    fixed (ContactConstraintPoint* pointsPtr = c.Points)
                    {
                        if (step.WarmStarting)
                        {
                            for (int j = 0; j < c.PointCount; ++j)
                            {
                                ContactConstraintPoint* ccp = &pointsPtr[j];
                                ccp->NormalImpulse *= step.DtRatio;
                                ccp->TangentImpulse *= step.DtRatio;
                                Vec2 P = ccp->NormalImpulse * normal + ccp->TangentImpulse * tangent;
                                bodyA._angularVelocity -= invIA * Vec2.Cross(ccp->RA, P);
                                bodyA._linearVelocity -= invMassA * P;
                                bodyB._angularVelocity += invIB * Vec2.Cross(ccp->RB, P);
                                bodyB._linearVelocity += invMassB * P;
                            }
                        }
                        else
                        {
                            for (int j = 0; j < c.PointCount; ++j)
                            {
                                ContactConstraintPoint* ccp = &pointsPtr[j];
                                ccp->NormalImpulse = Fix64.Zero;
                                ccp->TangentImpulse = Fix64.Zero;
                            }
                        }
                    }
                }
            }
        }

        private static readonly Fix64 ERROR_TOL = Fix64.From("0.001");

        public void SolveVelocityConstraints()
        {
            for (int i = 0; i < _constraintCount; ++i)
            {
                ContactConstraint c = _constraints[i];
                Body bodyA = c.BodyA;
                Body bodyB = c.BodyB;
                Fix64 wA = bodyA._angularVelocity;
                Fix64 wB = bodyB._angularVelocity;
                Vec2 vA = bodyA._linearVelocity;
                Vec2 vB = bodyB._linearVelocity;
                Fix64 invMassA = bodyA._invMass;
                Fix64 invIA = bodyA._invI;
                Fix64 invMassB = bodyB._invMass;
                Fix64 invIB = bodyB._invI;
                Vec2 normal = c.Normal;
                Vec2 tangent = Vec2.Cross(normal, Fix64.One);
                Fix64 friction = c.Friction;

                Box2DXDebug.Assert(c.PointCount == 1 || c.PointCount == 2);

                unsafe
                {
                    fixed (ContactConstraintPoint* pointsPtr = c.Points)
                    {
                        // Solve tangent constraints
                        for (int j = 0; j < c.PointCount; ++j)
                        {
                            ContactConstraintPoint* ccp = &pointsPtr[j];

                            // Relative velocity at contact
                            Vec2 dv = vB + Vec2.Cross(wB, ccp->RB) - vA - Vec2.Cross(wA, ccp->RA);

                            // Compute tangent force
                            Fix64 vt = Vec2.Dot(dv, tangent);
                            Fix64 lambda = ccp->TangentMass * (-vt);

                            // b2Clamp the accumulated force
                            Fix64 maxFriction = friction * ccp->NormalImpulse;
                            Fix64 newImpulse = Common.Math.Clamp(ccp->TangentImpulse + lambda, -maxFriction, maxFriction);
                            lambda = newImpulse - ccp->TangentImpulse;

                            // Apply contact impulse
                            Vec2 P = lambda * tangent;

                            vA -= invMassA * P;
                            wA -= invIA * Vec2.Cross(ccp->RA, P);

                            vB += invMassB * P;
                            wB += invIB * Vec2.Cross(ccp->RB, P);

                            ccp->TangentImpulse = newImpulse;
                        }

                        // Solve normal constraints
                        if (c.PointCount == 1)
                        {
                            ref ContactConstraintPoint ccp = ref c.Points[0];

                            // Relative velocity at contact
                            Vec2 dv = vB + Vec2.Cross(wB, ccp.RB) - vA - Vec2.Cross(wA, ccp.RA);

                            // Compute normal impulse
                            Fix64 vn = Vec2.Dot(dv, normal);
                            Fix64 lambda = -ccp.NormalMass * (vn - ccp.VelocityBias);

                            // Clamp the accumulated impulse
                            Fix64 newImpulse = Common.Math.Max(ccp.NormalImpulse + lambda, Fix64.Zero);
                            lambda = newImpulse - ccp.NormalImpulse;

                            // Apply contact impulse
                            Vec2 P = lambda * normal;
                            vA -= invMassA * P;
                            wA -= invIA * Vec2.Cross(ccp.RA, P);

                            vB += invMassB * P;
                            wB += invIB * Vec2.Cross(ccp.RB, P);
                            ccp.NormalImpulse = newImpulse;
                        }
                        else
                        {
                            // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
                            // Build the mini LCP for this contact patch
                            //
                            // vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
                            //
                            // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
                            // b = vn_0 - velocityBias
                            //
                            // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
                            // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
                            // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
                            // solution that satisfies the problem is chosen.
                            // 
                            // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
                            // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
                            //
                            // Substitute:
                            // 
                            // x = x' - a
                            // 
                            // Plug into above equation:
                            //
                            // vn = A * x + b
                            //    = A * (x' - a) + b
                            //    = A * x' + b - A * a
                            //    = A * x' + b'
                            // b' = b - A * a;

                            ContactConstraintPoint* cp1 = &pointsPtr[0];
                            ContactConstraintPoint* cp2 = &pointsPtr[1];

                            Vec2 a = new Vec2(cp1->NormalImpulse, cp2->NormalImpulse);
                            Box2DXDebug.Assert(a.X >= Fix64.Zero && a.Y >= Fix64.Zero);

                            // Relative velocity at contact
                            Vec2 dv1 = vB + Vec2.Cross(wB, cp1->RB) - vA - Vec2.Cross(wA, cp1->RA);
                            Vec2 dv2 = vB + Vec2.Cross(wB, cp2->RB) - vA - Vec2.Cross(wA, cp2->RA);

                            // Compute normal velocity
                            Fix64 vn1 = Vec2.Dot(dv1, normal);
                            Fix64 vn2 = Vec2.Dot(dv2, normal);

                            Vec2 b;
                            b.X = vn1 - cp1->VelocityBias;
                            b.Y = vn2 - cp2->VelocityBias;
                            b -= Common.Math.Mul(c.K, a);

                            //B2_NOT_USED(k_errorTol);

                            for (; ; )
                            {
                                //
                                // Case 1: vn = 0
                                //
                                // 0 = A * x' + b'
                                //
                                // Solve for x':
                                //
                                // x' = - inv(A) * b'
                                //
                                Vec2 x = -Common.Math.Mul(c.NormalMass, b);

                                if (x.X >= Fix64.Zero && x.Y >= Fix64.Zero)
                                {
                                    // Resubstitute for the incremental impulse
                                    Vec2 d = x - a;

                                    // Apply incremental impulse
                                    Vec2 P1 = d.X * normal;
                                    Vec2 P2 = d.Y * normal;
                                    vA -= invMassA * (P1 + P2);
                                    wA -= invIA * (Vec2.Cross(cp1->RA, P1) + Vec2.Cross(cp2->RA, P2));

                                    vB += invMassB * (P1 + P2);
                                    wB += invIB * (Vec2.Cross(cp1->RB, P1) + Vec2.Cross(cp2->RB, P2));

                                    // Accumulate
                                    cp1->NormalImpulse = x.X;
                                    cp2->NormalImpulse = x.Y;

#if DEBUG_SOLVER
									// Postconditions
									dv1 = vB + Vec2.Cross(wB, cp1->RB) - vA - Vec2.Cross(wA, cp1->RA);
									dv2 = vB + Vec2.Cross(wB, cp2->RB) - vA - Vec2.Cross(wA, cp2->RA);

									// Compute normal velocity
									vn1 = Vec2.Dot(dv1, normal);
									vn2 = Vec2.Dot(dv2, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
									Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
                                    break;
                                }

                                //
                                // Case 2: vn1 = 0 and x2 = 0
                                //
                                //   0 = a11 * x1' + a12 * 0 + b1' 
                                // vn2 = a21 * x1' + a22 * 0 + b2'
                                //
                                x.X = -cp1->NormalMass * b.X;
                                x.Y = Fix64.Zero;
                                vn1 = Fix64.Zero;
                                vn2 = c.K.Col1.Y * x.X + b.Y;

                                if (x.X >= Fix64.Zero && vn2 >= Fix64.Zero)
                                {
                                    // Resubstitute for the incremental impulse
                                    Vec2 d = x - a;

                                    // Apply incremental impulse
                                    Vec2 P1 = d.X * normal;
                                    Vec2 P2 = d.Y * normal;
                                    vA -= invMassA * (P1 + P2);
                                    wA -= invIA * (Vec2.Cross(cp1->RA, P1) + Vec2.Cross(cp2->RA, P2));

                                    vB += invMassB * (P1 + P2);
                                    wB += invIB * (Vec2.Cross(cp1->RB, P1) + Vec2.Cross(cp2->RB, P2));

                                    // Accumulate
                                    cp1->NormalImpulse = x.X;
                                    cp2->NormalImpulse = x.Y;

#if DEBUG_SOLVER
									// Postconditions
									dv1 = vB + Vec2.Cross(wB, cp1->RB) - vA - Vec2.Cross(wA, cp1->RA);

									// Compute normal velocity
									vn1 = Vec2.Dot(dv1, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn1 - cp1.VelocityBias) < k_errorTol);
#endif
                                    break;
                                }


                                //
                                // Case 3: w2 = 0 and x1 = 0
                                //
                                // vn1 = a11 * 0 + a12 * x2' + b1' 
                                //   0 = a21 * 0 + a22 * x2' + b2'
                                //
                                x.X = Fix64.Zero;
                                x.Y = -cp2->NormalMass * b.Y;
                                vn1 = c.K.Col2.X * x.Y + b.X;
                                vn2 = Fix64.Zero;

                                if (x.Y >= Fix64.Zero && vn1 >= Fix64.Zero)
                                {
                                    // Resubstitute for the incremental impulse
                                    Vec2 d = x - a;

                                    // Apply incremental impulse
                                    Vec2 P1 = d.X * normal;
                                    Vec2 P2 = d.Y * normal;
                                    vA -= invMassA * (P1 + P2);
                                    wA -= invIA * (Vec2.Cross(cp1->RA, P1) + Vec2.Cross(cp2->RA, P2));

                                    vB += invMassB * (P1 + P2);
                                    wB += invIB * (Vec2.Cross(cp1->RB, P1) + Vec2.Cross(cp2->RB, P2));

                                    // Accumulate
                                    cp1->NormalImpulse = x.X;
                                    cp2->NormalImpulse = x.Y;

#if DEBUG_SOLVER
									// Postconditions
									dv2 = vB + Vec2.Cross(wB, cp2->RB) - vA - Vec2.Cross(wA, cp2->RA);

									// Compute normal velocity
									vn2 = Vec2.Dot(dv2, normal);

									Box2DXDebug.Assert(Common.Math.Abs(vn2 - cp2.VelocityBias) < k_errorTol);
#endif
                                    break;
                                }

                                //
                                // Case 4: x1 = 0 and x2 = 0
                                // 
                                // vn1 = b1
                                // vn2 = b2;
                                x.X = Fix64.Zero;
                                x.Y = Fix64.Zero;
                                vn1 = b.X;
                                vn2 = b.Y;

                                if (vn1 >= Fix64.Zero && vn2 >= Fix64.Zero)
                                {
                                    // Resubstitute for the incremental impulse
                                    Vec2 d = x - a;

                                    // Apply incremental impulse
                                    Vec2 P1 = d.X * normal;
                                    Vec2 P2 = d.Y * normal;
                                    vA -= invMassA * (P1 + P2);
                                    wA -= invIA * (Vec2.Cross(cp1->RA, P1) + Vec2.Cross(cp2->RA, P2));

                                    vB += invMassB * (P1 + P2);
                                    wB += invIB * (Vec2.Cross(cp1->RB, P1) + Vec2.Cross(cp2->RB, P2));

                                    // Accumulate
                                    cp1->NormalImpulse = x.X;
                                    cp2->NormalImpulse = x.Y;

                                    break;
                                }

                                // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
                                break;
                            }
                        }

                        bodyA._linearVelocity = vA;
                        bodyA._angularVelocity = wA;
                        bodyB._linearVelocity = vB;
                        bodyB._angularVelocity = wB;
                    }
                }
            }
        }

        public void FinalizeVelocityConstraints()
        {
            for (int i = 0; i < _constraintCount; ++i)
            {
                ContactConstraint c = _constraints[i];
                Manifold m = c.Manifold;

                for (int j = 0; j < c.PointCount; ++j)
                {
                    m.Points[j].NormalImpulse = c.Points[j].NormalImpulse;
                    m.Points[j].TangentImpulse = c.Points[j].TangentImpulse;
                }
            }
        }

        internal class PositionSolverManifold
        {
            internal Vec2 Normal;
            internal Vec2[] Points = new Vec2[Settings.MaxManifoldPoints];
            internal Fix64[] Separations = new Fix64[Settings.MaxManifoldPoints];

            internal void Initialize(ContactConstraint cc)
            {
                Box2DXDebug.Assert(cc.PointCount > 0);

                switch (cc.Type)
                {
                    case ManifoldType.Circles:
                        {
                            Vec2 pointA = cc.BodyA.GetWorldPoint(cc.LocalPoint);
                            Vec2 pointB = cc.BodyB.GetWorldPoint(cc.Points[0].LocalPoint);
                            if (Vec2.DistanceSquared(pointA, pointB) > Settings.FLT_EPSILON_SQUARED)
                            {
                                Normal = pointB - pointA;
                                Normal.Normalize();
                            }
                            else
                            {
                                Normal.Set(Fix64.One, Fix64.Zero);
                            }

                            Points[0] = Fix64.Half * (pointA + pointB);
                            Separations[0] = Vec2.Dot(pointB - pointA, Normal) - cc.Radius;
                        }
                        break;

                    case ManifoldType.FaceA:
                        {
                            Normal = cc.BodyA.GetWorldVector(cc.LocalPlaneNormal);
                            Vec2 planePoint = cc.BodyA.GetWorldPoint(cc.LocalPoint);

                            for (int i = 0; i < cc.PointCount; ++i)
                            {
                                Vec2 clipPoint = cc.BodyB.GetWorldPoint(cc.Points[i].LocalPoint);
                                Separations[i] = Vec2.Dot(clipPoint - planePoint, Normal) - cc.Radius;
                                Points[i] = clipPoint;
                            }
                        }
                        break;

                    case ManifoldType.FaceB:
                        {
                            Normal = cc.BodyB.GetWorldVector(cc.LocalPlaneNormal);
                            Vec2 planePoint = cc.BodyB.GetWorldPoint(cc.LocalPoint);

                            for (int i = 0; i < cc.PointCount; ++i)
                            {
                                Vec2 clipPoint = cc.BodyA.GetWorldPoint(cc.Points[i].LocalPoint);
                                Separations[i] = Vec2.Dot(clipPoint - planePoint, Normal) - cc.Radius;
                                Points[i] = clipPoint;
                            }

                            // Ensure normal points from A to B
                            Normal = -Normal;
                        }
                        break;
                }
            }
        }

        private static PositionSolverManifold s_PositionSolverManifold = new PositionSolverManifold();

        public bool SolvePositionConstraints(Fix64 baumgarte)
        {
            Fix64 minSeparation = Fix64.Zero;

            for (int i = 0; i < _constraintCount; ++i)
            {
                ContactConstraint c = _constraints[i];
                Body bodyA = c.BodyA;
                Body bodyB = c.BodyB;

                Fix64 invMassA = bodyA._mass * bodyA._invMass;
                Fix64 invIA = bodyA._mass * bodyA._invI;
                Fix64 invMassB = bodyB._mass * bodyB._invMass;
                Fix64 invIB = bodyB._mass * bodyB._invI;

                s_PositionSolverManifold.Initialize(c);
                Vec2 normal = s_PositionSolverManifold.Normal;

                // Solver normal constraints
                for (int j = 0; j < c.PointCount; ++j)
                {
                    Vec2 point = s_PositionSolverManifold.Points[j];
                    Fix64 separation = s_PositionSolverManifold.Separations[j];

                    Vec2 rA = point - bodyA._sweep.C;
                    Vec2 rB = point - bodyB._sweep.C;

                    // Track max constraint error.
                    minSeparation = Common.Math.Min(minSeparation, separation);

                    // Prevent large corrections and allow slop.
                    Fix64 C = baumgarte * Common.Math.Clamp(separation + Settings.LinearSlop, -Settings.MaxLinearCorrection, Fix64.Zero);

                    // Compute normal impulse
                    Fix64 impulse = -c.Points[j].EqualizedMass * C;

                    Vec2 P = impulse * normal;

                    bodyA._sweep.C -= invMassA * P;
                    bodyA._sweep.A -= invIA * Vec2.Cross(rA, P);
                    bodyA.SynchronizeTransform();

                    bodyB._sweep.C += invMassB * P;
                    bodyB._sweep.A += invIB * Vec2.Cross(rB, P);
                    bodyB.SynchronizeTransform();
                }
            }

            // We can't expect minSpeparation >= -Settings.LinearSlop because we don't
            // push the separation above -Settings.LinearSlop.
            return minSeparation >= -Fix64.OneAndHalf * Settings.LinearSlop;
        }
    }
}
