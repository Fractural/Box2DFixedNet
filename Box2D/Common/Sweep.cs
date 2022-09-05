/*
  Box2DX Copyright (c) 2008 Ihar Kalasouski http://code.google.com/p/box2dx
  Box2D original C++ version Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com

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

using FixMath.NET;

namespace Box2DX.Common
{
    public struct Sweep
    {
        public Vec2 LocalCenter;    //local center of mass position
        public Vec2 C0, C; //local center of mass position
        public Fix64 A0, A; //world angles
        public Fix64 T0; //time interval = [T0,1], where T0 is in [0,1]

        /// <summary>
        /// Get the interpolated transform at a specific time.
        /// </summary>
        /// <param name="alpha">Alpha is a factor in [0,1], where 0 indicates t0.</param>
        public void GetTransform(out XForm xf, Fix64 alpha)
        {
            xf = new XForm();
            xf.Position = (Fix64.One - alpha) * C0 + alpha * C;
            Fix64 angle = (Fix64.One - alpha) * A0 + alpha * A;
            xf.R.Set(angle);

            // Shift to origin
            xf.Position -= Common.Math.Mul(xf.R, LocalCenter);
        }

        /// <summary>
        /// Advance the sweep forward, yielding a new initial state.
        /// </summary>
        /// <param name="t">The new initial time.</param>
        public void Advance(Fix64 t)
        {
            if (T0 < t && Fix64.One - T0 > Settings.FLT_EPSILON)
            {
                Fix64 alpha = (t - T0) / (Fix64.One - T0);
                C0 = (Fix64.One - alpha) * C0 + alpha * C;
                A0 = (Fix64.One - alpha) * A0 + alpha * A;
                T0 = t;
            }
        }
    }
}
