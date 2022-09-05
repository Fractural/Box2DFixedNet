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
    /// A circle shape.
    /// </summary>
    public class CircleShape : Shape
    {
        // Position
        internal Vec2 _position;

        public CircleShape()
        {
            _type = ShapeType.CircleShape;
        }

        public override bool TestPoint(XForm transform, Vec2 p)
        {
            Vec2 center = transform.Position + Common.Math.Mul(transform.R, _position);
            Vec2 d = p - center;
            return Vec2.Dot(d, d) <= _radius * _radius;
        }

        // Collision Detection in Interactive 3D Environments by Gino van den Bergen
        // From Section 3.1.2
        // x = s + a * r
        // norm(x) = radius
        public override SegmentCollide TestSegment(XForm transform, out Fix64 lambda, out Vec2 normal, Segment segment, Fix64 maxLambda)
        {
            lambda = Fix64.Zero;
            normal = Vec2.Zero;

            Vec2 position = transform.Position + Common.Math.Mul(transform.R, _position);
            Vec2 s = segment.P1 - position;
            Fix64 b = Vec2.Dot(s, s) - _radius * _radius;

            // Does the segment start inside the circle?
            if (b < Fix64.Zero)
            {
                lambda = Fix64.Zero;
                return SegmentCollide.StartInsideCollide;
            }

            // Solve quadratic equation.
            Vec2 r = segment.P2 - segment.P1;
            Fix64 c = Vec2.Dot(s, r);
            Fix64 rr = Vec2.Dot(r, r);
            Fix64 sigma = c * c - rr * b;

            // Check for negative discriminant and short segment.
            if (sigma < Fix64.Zero || rr < Common.Settings.FLT_EPSILON)
            {
                return SegmentCollide.MissCollide;
            }

            // Find the point of intersection of the line with the circle.
            Fix64 a = -(c + Common.Math.Sqrt(sigma));

            // Is the intersection point on the segment?
            if (Fix64.Zero <= a && a <= maxLambda * rr)
            {
                a /= rr;
                lambda = a;
                normal = s + a * r;
                normal.Normalize();
                return SegmentCollide.HitCollide;
            }

            return SegmentCollide.MissCollide;
        }

        public override void ComputeAABB(out AABB aabb, XForm transform)
        {
            aabb = new AABB();

            Vec2 p = transform.Position + Common.Math.Mul(transform.R, _position);
            aabb.LowerBound.Set(p.X - _radius, p.Y - _radius);
            aabb.UpperBound.Set(p.X + _radius, p.Y + _radius);
        }

        public override void ComputeMass(out MassData massData, Fix64 density)
        {
            massData = new MassData();

            massData.Mass = density * Settings.Pi * _radius * _radius;
            massData.Center = _position;

            // inertia about the local origin
            massData.I = massData.Mass * (Fix64.Half * _radius * _radius + Vec2.Dot(_position, _position));
        }

        public override Fix64 ComputeSubmergedArea(Vec2 normal, Fix64 offset, XForm xf, out Vec2 c)
        {
            Vec2 p = Box2DX.Common.Math.Mul(xf, _position);
            Fix64 l = -(Vec2.Dot(normal, p) - offset);
            if (l < -_radius + Box2DX.Common.Settings.FLT_EPSILON)
            {
                //Completely dry
                c = new Vec2();
                return Fix64.Zero;
            }
            if (l > _radius)
            {
                //Completely wet
                c = p;
                return Box2DX.Common.Settings.Pi * _radius * _radius;
            }

            //Magic
            Fix64 r2 = _radius * _radius;
            Fix64 l2 = l * l;
            Fix64 area = r2 * Fix64.Acos(Fix64.Asin(l / _radius) + Box2DX.Common.Settings.Pi / Fix64.Two) +
                l * Box2DX.Common.Math.Sqrt(r2 - l2);
            Fix64 com = -Fix64.Two / Fix64.Three * Fix64.Pow(r2 - l2, Fix64.OneAndHalf) / area;

            c.X = p.X + normal.X * com;
            c.Y = p.Y + normal.Y * com;

            return area;
        }

        /// <summary>
        /// Get the supporting vertex index in the given direction.
        /// </summary>
        public override int GetSupport(Vec2 d)
        {
            return 0;
        }

        /// <summary>
        /// Get the supporting vertex in the given direction.
        /// </summary>
        public override Vec2 GetSupportVertex(Vec2 d)
        {
            return _position;
        }

        /// <summary>
        /// Get a vertex by index. Used by Distance.
        /// </summary>
        public override Vec2 GetVertex(int index)
        {
            Box2DXDebug.Assert(index == 0);
            return _position;
        }

        public override Fix64 ComputeSweepRadius(Vec2 pivot)
        {
            return Vec2.Distance(_position, pivot);
        }

        /// <summary>
        /// Get the vertex count.
        /// </summary>
        public int VertexCount { get { return 1; } }
    }
}