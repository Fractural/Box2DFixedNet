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

using Box2DX.Common;
using FixMath.NET;

namespace Box2DX.Collision
{
    /// <summary>
    /// Inpute parameters for TimeOfImpact
    /// </summary>
    public struct TOIInput
    {
        public Sweep SweepA;
        public Sweep SweepB;
        public Fix64 SweepRadiusA;
        public Fix64 SweepRadiusB;
        public Fix64 Tolerance;
    }

    internal struct SeparationFunction
    {
        internal enum Type
        {
            Points,
            FaceA,
            FaceB
        };

        internal unsafe void Initialize(SimplexCache* cache,
            Shape shapeA, XForm transformA,
            Shape shapeB, XForm transformB)
        {
            ShapeA = shapeA;
            ShapeB = shapeB;
            int count = cache->Count;
            Box2DXDebug.Assert(0 < count && count < 3);

            if (count == 1)
            {
                FaceType = Type.Points;
                Vec2 localPointA = ShapeA.GetVertex(cache->IndexA[0]);
                Vec2 localPointB = ShapeB.GetVertex(cache->IndexB[0]);
                Vec2 pointA = Common.Math.Mul(transformA, localPointA);
                Vec2 pointB = Common.Math.Mul(transformB, localPointB);
                Axis = pointB - pointA;
                Axis.Normalize();
            }
            else if (cache->IndexB[0] == cache->IndexB[1])
            {
                // Two points on A and one on B
                FaceType = Type.FaceA;
                Vec2 localPointA1 = ShapeA.GetVertex(cache->IndexA[0]);
                Vec2 localPointA2 = ShapeA.GetVertex(cache->IndexA[1]);
                Vec2 localPointB = ShapeB.GetVertex(cache->IndexB[0]);
                LocalPoint = Fix64.Half * (localPointA1 + localPointA2);
                Axis = Vec2.Cross(localPointA2 - localPointA1, Fix64.One);
                Axis.Normalize();

                Vec2 normal = Common.Math.Mul(transformA.R, Axis);
                Vec2 pointA = Common.Math.Mul(transformA, LocalPoint);
                Vec2 pointB = Common.Math.Mul(transformB, localPointB);

                Fix64 s = Vec2.Dot(pointB - pointA, normal);
                if (s < Fix64.Zero)
                {
                    Axis = -Axis;
                }
            }
            else
            {
                // Two points on B and one or two points on A.
                // We ignore the second point on A.
                FaceType = Type.FaceB;
                Vec2 localPointA = shapeA.GetVertex(cache->IndexA[0]);
                Vec2 localPointB1 = shapeB.GetVertex(cache->IndexB[0]);
                Vec2 localPointB2 = shapeB.GetVertex(cache->IndexB[1]);
                LocalPoint = Fix64.Half * (localPointB1 + localPointB2);
                Axis = Vec2.Cross(localPointB2 - localPointB1, Fix64.One);
                Axis.Normalize();

                Vec2 normal = Common.Math.Mul(transformB.R, Axis);
                Vec2 pointB = Common.Math.Mul(transformB, LocalPoint);
                Vec2 pointA = Common.Math.Mul(transformA, localPointA);

                Fix64 s = Vec2.Dot(pointA - pointB, normal);
                if (s < Fix64.Zero)
                {
                    Axis = -Axis;
                }
            }
        }

        internal Fix64 Evaluate(XForm transformA, XForm transformB)
        {
            switch (FaceType)
            {
                case Type.Points:
                    {
                        Vec2 axisA = Common.Math.MulT(transformA.R, Axis);
                        Vec2 axisB = Common.Math.MulT(transformB.R, -Axis);
                        Vec2 localPointA = ShapeA.GetSupportVertex(axisA);
                        Vec2 localPointB = ShapeB.GetSupportVertex(axisB);
                        Vec2 pointA = Common.Math.Mul(transformA, localPointA);
                        Vec2 pointB = Common.Math.Mul(transformB, localPointB);
                        Fix64 separation = Vec2.Dot(pointB - pointA, Axis);
                        return separation;
                    }

                case Type.FaceA:
                    {
                        Vec2 normal = Common.Math.Mul(transformA.R, Axis);
                        Vec2 pointA = Common.Math.Mul(transformA, LocalPoint);

                        Vec2 axisB = Common.Math.MulT(transformB.R, -normal);

                        Vec2 localPointB = ShapeB.GetSupportVertex(axisB);
                        Vec2 pointB = Common.Math.Mul(transformB, localPointB);

                        Fix64 separation = Vec2.Dot(pointB - pointA, normal);
                        return separation;
                    }

                case Type.FaceB:
                    {
                        Vec2 normal = Common.Math.Mul(transformB.R, Axis);
                        Vec2 pointB = Common.Math.Mul(transformB, LocalPoint);

                        Vec2 axisA = Common.Math.MulT(transformA.R, -normal);

                        Vec2 localPointA = ShapeA.GetSupportVertex(axisA);
                        Vec2 pointA = Common.Math.Mul(transformA, localPointA);

                        Fix64 separation = Vec2.Dot(pointA - pointB, normal);
                        return separation;
                    }

                default:
                    Box2DXDebug.Assert(false);
                    return Fix64.Zero;
            }
        }

        internal Shape ShapeA;
        internal Shape ShapeB;
        internal Type FaceType;
        internal Vec2 LocalPoint;
        internal Vec2 Axis;
    }

    public partial class Collision
    {
        public static int MaxToiIters;
        public static int MaxToiRootIters;

        private static readonly Fix64 FIX_100 = Fix64.From(100);
        private static readonly Fix64 FIX_0_025 = Fix64.From("0.025");
        private static readonly Fix64 FIX_0_75 = Fix64.From("0.75");
        private static readonly Fix64 FIX_0_02 = Fix64.From("0.02");

        // CCD via the secant method.
        /// <summary>
        /// Compute the time when two shapes begin to touch or touch at a closer distance.
        /// TOI considers the shape radii. It attempts to have the radii overlap by the tolerance.
        /// Iterations terminate with the overlap is within 0.5 * tolerance. The tolerance should be
        /// smaller than sum of the shape radii.
        /// Warning the sweeps must have the same time interval.
        /// </summary>
        /// <returns>
        /// The fraction between [0,1] in which the shapes first touch.
        /// fraction=0 means the shapes begin touching/overlapped, and fraction=1 means the shapes don't touch.
        /// </returns>
        public static Fix64 TimeOfImpact(TOIInput input, Shape shapeA, Shape shapeB)
        {
            Sweep sweepA = input.SweepA;
            Sweep sweepB = input.SweepB;

            Box2DXDebug.Assert(sweepA.T0 == sweepB.T0);
            Box2DXDebug.Assert(Fix64.One - sweepA.T0 > Common.Settings.FLT_EPSILON);

            Fix64 radius = shapeA._radius + shapeB._radius;
            Fix64 tolerance = input.Tolerance;

            Fix64 alpha = Fix64.Zero;

            const int k_maxIterations = 1000;   // TODO_ERIN b2Settings
            int iter = 0;
            Fix64 target = Fix64.Zero;

            // Prepare input for distance query.
            SimplexCache cache = new SimplexCache();
            cache.Count = 0;
            DistanceInput distanceInput;
            distanceInput.UseRadii = false;

            for (; ; )
            {
                XForm xfA, xfB;
                sweepA.GetTransform(out xfA, alpha);
                sweepB.GetTransform(out xfB, alpha);

                // Get the distance between shapes.
                distanceInput.TransformA = xfA;
                distanceInput.TransformB = xfB;
                DistanceOutput distanceOutput;
                Distance(out distanceOutput, ref cache, ref distanceInput, shapeA, shapeB);

                if (distanceOutput.Distance <= Fix64.Zero)
                {
                    alpha = Fix64.One;
                    break;
                }

                SeparationFunction fcn = new SeparationFunction();
                unsafe
                {
                    fcn.Initialize(&cache, shapeA, xfA, shapeB, xfB);
                }

                Fix64 separation = fcn.Evaluate(xfA, xfB);
                if (separation <= Fix64.Zero)
                {
                    alpha = Fix64.One;
                    break;
                }

                if (iter == 0)
                {
                    // Compute a reasonable target distance to give some breathing room
                    // for conservative advancement. We take advantage of the shape radii
                    // to create additional clearance.
                    if (separation > radius)
                    {
                        target = Common.Math.Max(radius - tolerance, FIX_0_75 * radius);
                    }
                    else
                    {
                        target = Common.Math.Max(separation - tolerance, FIX_0_02 * radius);
                    }
                }

                if (separation - target < Fix64.Half * tolerance)
                {
                    if (iter == 0)
                    {
                        alpha = Fix64.One;
                        break;
                    }

                    break;
                }

#if _FALSE
				// Dump the curve seen by the root finder
				{
					const int32 N = 100;
					float32 dx = Fix64.One / N;
					float32 xs[N+1];
					float32 fs[N+1];

					float32 x = Fix64.Zero;

					for (int32 i = 0; i <= N; ++i)
					{
						sweepA.GetTransform(&xfA, x);
						sweepB.GetTransform(&xfB, x);
						float32 f = fcn.Evaluate(xfA, xfB) - target;

						printf("%g %g\n", x, f);

						xs[i] = x;
						fs[i] = f;

						x += dx;
					}
				}
#endif

                // Compute 1D root of: f(x) - target = 0
                Fix64 newAlpha = alpha;
                {
                    Fix64 x1 = alpha, x2 = Fix64.One;

                    Fix64 f1 = separation;

                    sweepA.GetTransform(out xfA, x2);
                    sweepB.GetTransform(out xfB, x2);
                    Fix64 f2 = fcn.Evaluate(xfA, xfB);

                    // If intervals don't overlap at t2, then we are done.
                    if (f2 >= target)
                    {
                        alpha = Fix64.One;
                        break;
                    }

                    // Determine when intervals intersect.
                    int rootIterCount = 0;
                    for (; ; )
                    {
                        // Use a mix of the secant rule and bisection.
                        Fix64 x;
                        if ((rootIterCount & 1) != 0)
                        {
                            // Secant rule to improve convergence.
                            x = x1 + (target - f1) * (x2 - x1) / (f2 - f1);
                        }
                        else
                        {
                            // Bisection to guarantee progress.
                            x = Fix64.Half * (x1 + x2);
                        }

                        sweepA.GetTransform(out xfA, x);
                        sweepB.GetTransform(out xfB, x);

                        Fix64 f = fcn.Evaluate(xfA, xfB);

                        if (Common.Math.Abs(f - target) < FIX_0_025 * tolerance)
                        {
                            newAlpha = x;
                            break;
                        }

                        // Ensure we continue to bracket the root.
                        if (f > target)
                        {
                            x1 = x;
                            f1 = f;
                        }
                        else
                        {
                            x2 = x;
                            f2 = f;
                        }

                        ++rootIterCount;

                        Box2DXDebug.Assert(rootIterCount < 50);
                    }

                    MaxToiRootIters = Common.Math.Max(MaxToiRootIters, rootIterCount);
                }

                // Ensure significant advancement.
                if (newAlpha < (Fix64.One + FIX_100 * Common.Settings.FLT_EPSILON) * alpha)
                {
                    break;
                }

                alpha = newAlpha;

                ++iter;

                if (iter == k_maxIterations)
                {
                    break;
                }
            }

            MaxToiIters = Common.Math.Max(MaxToiIters, iter);

            return alpha;
        }
    }
}