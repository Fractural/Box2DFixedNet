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
    public class Settings
    {
#if TARGET_FLOAT32_IS_FIXED
		public static readonly Fix64 FLT_EPSILON = FIXED_EPSILON;
		public static readonly Fix64 FLT_MAX = FIXED_MAX;
		public static Fix64	FORCE_SCALE2(x){ return x<<7;}
		public static Fix64 FORCE_INV_SCALE2(x)	{return x>>7;}
#else
        // TODO: Should we use Precision or Epsilon here?
        public static readonly Fix64 FLT_EPSILON = Fix64.FixPrecision;//smallest such that Fix64.One+FLT_EPSILON != Fix64.One
        public static readonly Fix64 FLT_EPSILON_SQUARED = FLT_EPSILON * FLT_EPSILON;//smallest such that Fix64.One+FLT_EPSILON != Fix64.One
        public static readonly Fix64 FLT_MAX = Fix64.MaxValue;
        public static Fix64 FORCE_SCALE(Fix64 x) { return x; }
        public static Fix64 FORCE_INV_SCALE(Fix64 x) { return x; }
#endif

        public static readonly Fix64 Pi = Fix64.Pi;

        // Global tuning constants based on meters-kilograms-seconds (MKS) units.

        // Collision
        public static readonly int MaxManifoldPoints = 2;
        public static readonly int MaxPolygonVertices = 8;
        public static readonly int MaxProxies = 512; // this must be a power of two
        public static readonly int MaxPairs = 8 * MaxProxies; // this must be a power of two

        // Dynamics

        /// <summary>
        /// A small length used as a collision and constraint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        /// </summary>
        public static readonly Fix64 LinearSlop = Fix64.From("0.005");   // 0.5 cm

        /// <summary>
        /// A small angle used as a collision and constraint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        /// </summary>
        public static readonly Fix64 AngularSlop = Fix64.Two / Fix64.From(180) * Pi; // 2 degrees

        /// <summary>
        /// The radius of the polygon/edge shape skin. This should not be modified. Making
        /// this smaller means polygons will have and insufficient for continuous collision.
        /// Making it larger may create artifacts for vertex collision.
        /// </summary>
        public static readonly Fix64 PolygonRadius = Fix64.Two * LinearSlop;

        /// <summary>
        /// Continuous collision detection (CCD) works with core, shrunken shapes. This is amount
        /// by which shapes are automatically shrunk to work with CCD. 
        /// This must be larger than LinearSlop.
        /// </summary>
        public static readonly Fix64 ToiSlop = Fix64.From(8) * LinearSlop;

        /// <summary>
        /// Maximum number of contacts to be handled to solve a TOI island.
        /// </summary>
        public static readonly int MaxTOIContactsPerIsland = 32;

        /// <summary>
        /// Maximum number of joints to be handled to solve a TOI island.
        /// </summary>
        public static readonly int MaxTOIJointsPerIsland = 32;

        /// <summary>
        /// A velocity threshold for elastic collisions. Any collision with a relative linear
        /// velocity below this threshold will be treated as inelastic.
        /// </summary>
        public static readonly Fix64 VelocityThreshold = Fix64.One; // 1 m/s

        /// <summary>
        /// The maximum linear position correction used when solving constraints.
        /// This helps to prevent overshoot.
        /// </summary>
        public static readonly Fix64 MaxLinearCorrection = Fix64.From("0.2"); // 20 cm

        /// <summary>
        /// The maximum angular position correction used when solving constraints.
        /// This helps to prevent overshoot.
        /// </summary>
        public static readonly Fix64 MaxAngularCorrection = Fix64.From(8) / Fix64.From(180) * Pi; // 8 degrees

        /// <summary>
        /// The maximum linear velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
#if TARGET_FLOAT32_IS_FIXED
		public static readonly Fix64 MaxLinearVelocity = 100.Fix64.Zero;
#else
        public static readonly Fix64 MaxLinearVelocity = Fix64.From(200);
        public static readonly Fix64 MaxLinearVelocitySquared = MaxLinearVelocity * MaxLinearVelocity;
#endif
        /// <summary>
        /// The maximum angular velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public static readonly Fix64 MaxAngularVelocity = Fix64.From(250);
#if !TARGET_FLOAT32_IS_FIXED
        public static readonly Fix64 MaxAngularVelocitySquared = MaxAngularVelocity * MaxAngularVelocity;
#endif

        /// <summary>
        /// The maximum linear velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public static readonly Fix64 MaxTranslation = Fix64.Two;
        public static readonly Fix64 MaxTranslationSquared = (MaxTranslation * MaxTranslation);

        /// <summary>
        /// The maximum angular velocity of a body. This limit is very large and is used
        /// to prevent numerical problems. You shouldn't need to adjust this.
        /// </summary>
        public static readonly Fix64 MaxRotation = (Fix64.Half * Pi);
        public static readonly Fix64 MaxRotationSquared = (MaxRotation * MaxRotation);

        /// <summary>
        /// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
        /// that overlap is removed in one time step. However using values close to 1 often lead to overshoot.
        /// </summary>
        public static readonly Fix64 ContactBaumgarte = Fix64.From("0.2");

        // Sleep

        /// <summary>
        /// The time that a body must be still before it will go to sleep.
        /// </summary>
        public static readonly Fix64 TimeToSleep = Fix64.Half; // half a second

        /// <summary>
        /// A body cannot sleep if its linear velocity is above this tolerance.
        /// </summary>
        public static readonly Fix64 LinearSleepTolerance = Fix64.From("0.01"); // 1 cm/s

        /// <summary>
        /// A body cannot sleep if its angular velocity is above this tolerance.
        /// </summary>
        public static readonly Fix64 AngularSleepTolerance = Fix64.Two / Fix64.From(180); // 2 degrees/s

        /// <summary>
        /// Friction mixing law. Feel free to customize this.
        /// </summary>
        public static Fix64 MixFriction(Fix64 friction1, Fix64 friction2)
        {
            return Fix64.Sqrt(friction1 * friction2);
        }

        /// <summary>
        /// Restitution mixing law. Feel free to customize this.
        /// </summary>
        public static Fix64 MixRestitution(Fix64 restitution1, Fix64 restitution2)
        {
            return restitution1 > restitution2 ? restitution1 : restitution2;
        }
    }
}
