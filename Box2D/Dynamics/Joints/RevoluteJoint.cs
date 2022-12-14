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

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Motor constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

using Box2DX.Common;
using FixMath.NET;

namespace Box2DX.Dynamics
{
    using Box2DXMath = Box2DX.Common.Math;

    /// <summary>
    /// Revolute joint definition. This requires defining an
    /// anchor point where the bodies are joined. The definition
    /// uses local anchor points so that the initial configuration
    /// can violate the constraint slightly. You also need to
    /// specify the initial relative angle for joint limits. This
    /// helps when saving and loading a game.
    /// The local anchor points are measured from the body's origin
    /// rather than the center of mass because:
    /// 1. you might not know where the center of mass will be.
    /// 2. if you add/remove shapes from a body and recompute the mass,
    ///    the joints will be broken.
    /// </summary>
    public class RevoluteJointDef : JointDef
    {
        public RevoluteJointDef()
        {
            Type = JointType.RevoluteJoint;
            LocalAnchor1.Set(Fix64.Zero, Fix64.Zero);
            LocalAnchor2.Set(Fix64.Zero, Fix64.Zero);
            ReferenceAngle = Fix64.Zero;
            LowerAngle = Fix64.Zero;
            UpperAngle = Fix64.Zero;
            MaxMotorTorque = Fix64.Zero;
            MotorSpeed = Fix64.Zero;
            EnableLimit = false;
            EnableMotor = false;
        }

        /// <summary>
        /// Initialize the bodies, anchors, and reference angle using the world
        /// anchor.
        /// </summary>
        public void Initialize(Body body1, Body body2, Vec2 anchor)
        {
            Body1 = body1;
            Body2 = body2;
            LocalAnchor1 = body1.GetLocalPoint(anchor);
            LocalAnchor2 = body2.GetLocalPoint(anchor);
            ReferenceAngle = body2.GetAngle() - body1.GetAngle();
        }

        /// <summary>
        /// The local anchor point relative to body1's origin.
        /// </summary>
        public Vec2 LocalAnchor1;

        /// <summary>
        /// The local anchor point relative to body2's origin.
        /// </summary>
        public Vec2 LocalAnchor2;

        /// <summary>
        /// The body2 angle minus body1 angle in the reference state (radians).
        /// </summary>
        public Fix64 ReferenceAngle;

        /// <summary>
        /// A flag to enable joint limits.
        /// </summary>
        public bool EnableLimit;

        /// <summary>
        /// The lower angle for the joint limit (radians).
        /// </summary>
        public Fix64 LowerAngle;

        /// <summary>
        /// The upper angle for the joint limit (radians).
        /// </summary>
        public Fix64 UpperAngle;

        /// <summary>
        /// A flag to enable the joint motor.
        /// </summary>
        public bool EnableMotor;

        /// <summary>
        /// The desired motor speed. Usually in radians per second.
        /// </summary>
        public Fix64 MotorSpeed;

        /// <summary>
        /// The maximum motor torque used to achieve the desired motor speed.
        /// Usually in N-m.
        /// </summary>
        public Fix64 MaxMotorTorque;
    }

    /// <summary>
    /// A revolute joint constrains to bodies to share a common point while they
    /// are free to rotate about the point. The relative rotation about the shared
    /// point is the joint angle. You can limit the relative rotation with
    /// a joint limit that specifies a lower and upper angle. You can use a motor
    /// to drive the relative rotation about the shared point. A maximum motor torque
    /// is provided so that infinite forces are not generated.
    /// </summary>
    public class RevoluteJoint : Joint
    {
        public Vec2 _localAnchor1;  // relative
        public Vec2 _localAnchor2;
        public Vec3 _impulse;
        public Fix64 _motorImpulse;
        public Mat33 _mass; //effective mass for p2p constraint.
        public Fix64 _motorMass;    // effective mass for motor/limit angular constraint.

        public bool _enableMotor;
        public Fix64 _maxMotorTorque;
        public Fix64 _motorSpeed;

        public bool _enableLimit;
        public Fix64 _referenceAngle;
        public Fix64 _lowerAngle;
        public Fix64 _upperAngle;
        public LimitState _limitState;

        public override Vec2 Anchor1
        {
            get { return _body1.GetWorldPoint(_localAnchor1); }
        }

        public override Vec2 Anchor2
        {
            get { return _body2.GetWorldPoint(_localAnchor2); }
        }

        public override Vec2 GetReactionForce(Fix64 inv_dt)
        {
            Vec2 P = new Vec2(_impulse.X, _impulse.Y);
            return inv_dt * P;
        }

        public override Fix64 GetReactionTorque(Fix64 inv_dt)
        {
            return inv_dt * _impulse.Z;
        }

        /// <summary>
        /// Get the current joint angle in radians.
        /// </summary>
        public Fix64 JointAngle
        {
            get
            {
                Body b1 = _body1;
                Body b2 = _body2;
                return b2._sweep.A - b1._sweep.A - _referenceAngle;
            }
        }


        /// <summary>
        /// Get the current joint angle speed in radians per second.
        /// </summary>
        public Fix64 JointSpeed
        {
            get
            {
                Body b1 = _body1;
                Body b2 = _body2;
                return b2._angularVelocity - b1._angularVelocity;
            }
        }

        /// <summary>
        /// Is the joint limit enabled?
        /// </summary>
        public bool IsLimitEnabled
        {
            get { return _enableLimit; }
        }

        /// <summary>
        /// Enable/disable the joint limit.
        /// </summary>
        public void EnableLimit(bool flag)
        {
            _body1.WakeUp();
            _body2.WakeUp();
            _enableLimit = flag;
        }

        /// <summary>
        /// Get the lower joint limit in radians.
        /// </summary>
        public Fix64 LowerLimit
        {
            get { return _lowerAngle; }
        }

        /// <summary>
        /// Get the upper joint limit in radians.
        /// </summary>
        public Fix64 UpperLimit
        {
            get { return _upperAngle; }
        }

        /// <summary>
        /// Set the joint limits in radians.
        /// </summary>
        public void SetLimits(Fix64 lower, Fix64 upper)
        {
            Box2DXDebug.Assert(lower <= upper);
            _body1.WakeUp();
            _body2.WakeUp();
            _lowerAngle = lower;
            _upperAngle = upper;
        }

        /// <summary>
        /// Is the joint motor enabled?
        /// </summary>
        public bool IsMotorEnabled
        {
            get { return _enableMotor; }
        }

        /// <summary>
        /// Enable/disable the joint motor.
        /// </summary>
        public void EnableMotor(bool flag)
        {
            _body1.WakeUp();
            _body2.WakeUp();
            _enableMotor = flag;
        }

        /// <summary>
        /// Get\Set the motor speed in radians per second.
        /// </summary>
        public Fix64 MotorSpeed
        {
            get { return _motorSpeed; }
            set
            {
                _body1.WakeUp();
                _body2.WakeUp();
                _motorSpeed = value;
            }
        }

        /// <summary>
        /// Set the maximum motor torque, usually in N-m.
        /// </summary>
        public void SetMaxMotorTorque(Fix64 torque)
        {
            _body1.WakeUp();
            _body2.WakeUp();
            _maxMotorTorque = torque;
        }

        /// <summary>
        /// Get the current motor torque, usually in N-m.
        /// </summary>
        public Fix64 MotorTorque
        {
            get { return _motorImpulse; }
        }

        public RevoluteJoint(RevoluteJointDef def)
            : base(def)
        {
            _localAnchor1 = def.LocalAnchor1;
            _localAnchor2 = def.LocalAnchor2;
            _referenceAngle = def.ReferenceAngle;

            _impulse = new Vec3();
            _motorImpulse = Fix64.Zero;

            _lowerAngle = def.LowerAngle;
            _upperAngle = def.UpperAngle;
            _maxMotorTorque = def.MaxMotorTorque;
            _motorSpeed = def.MotorSpeed;
            _enableLimit = def.EnableLimit;
            _enableMotor = def.EnableMotor;
            _limitState = LimitState.InactiveLimit;
        }

        internal override void InitVelocityConstraints(TimeStep step)
        {
            Body b1 = _body1;
            Body b2 = _body2;

            if (_enableMotor || _enableLimit)
            {
                // You cannot create a rotation limit between bodies that
                // both have fixed rotation.
                Box2DXDebug.Assert(b1._invI > Fix64.Zero || b2._invI > Fix64.Zero);
            }

            // Compute the effective mass matrix.
            Vec2 r1 = Box2DXMath.Mul(b1.GetXForm().R, _localAnchor1 - b1.GetLocalCenter());
            Vec2 r2 = Box2DXMath.Mul(b2.GetXForm().R, _localAnchor2 - b2.GetLocalCenter());

            // J = [-I -r1_skew I r2_skew]
            //     [ 0       -1 0       1]
            // r_skew = [-ry; rx]

            // Matlab
            // K = [ m1+r1y^2*i1+m2+r2y^2*i2,  -r1y*i1*r1x-r2y*i2*r2x,          -r1y*i1-r2y*i2]
            //     [  -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2,           r1x*i1+r2x*i2]
            //     [          -r1y*i1-r2y*i2,           r1x*i1+r2x*i2,                   i1+i2]

            Fix64 m1 = b1._invMass, m2 = b2._invMass;
            Fix64 i1 = b1._invI, i2 = b2._invI;

            _mass.Col1.X = m1 + m2 + r1.Y * r1.Y * i1 + r2.Y * r2.Y * i2;
            _mass.Col2.X = -r1.Y * r1.X * i1 - r2.Y * r2.X * i2;
            _mass.Col3.X = -r1.Y * i1 - r2.Y * i2;
            _mass.Col1.Y = _mass.Col2.X;
            _mass.Col2.Y = m1 + m2 + r1.X * r1.X * i1 + r2.X * r2.X * i2;
            _mass.Col3.Y = r1.X * i1 + r2.X * i2;
            _mass.Col1.Z = _mass.Col3.X;
            _mass.Col2.Z = _mass.Col3.Y;
            _mass.Col3.Z = i1 + i2;

            _motorMass = Fix64.One / (i1 + i2);

            if (_enableMotor == false)
            {
                _motorImpulse = Fix64.Zero;
            }

            if (_enableLimit)
            {
                Fix64 jointAngle = b2._sweep.A - b1._sweep.A - _referenceAngle;
                if (Box2DXMath.Abs(_upperAngle - _lowerAngle) < Fix64.Two * Settings.AngularSlop)
                {
                    _limitState = LimitState.EqualLimits;
                }
                else if (jointAngle <= _lowerAngle)
                {
                    if (_limitState != LimitState.AtLowerLimit)
                    {
                        _impulse.Z = Fix64.Zero;
                    }
                    _limitState = LimitState.AtLowerLimit;
                }
                else if (jointAngle >= _upperAngle)
                {
                    if (_limitState != LimitState.AtUpperLimit)
                    {
                        _impulse.Z = Fix64.Zero;
                    }
                    _limitState = LimitState.AtUpperLimit;
                }
                else
                {
                    _limitState = LimitState.InactiveLimit;
                    _impulse.Z = Fix64.Zero;
                }
            }
            else
            {
                _limitState = LimitState.InactiveLimit;
            }

            if (step.WarmStarting)
            {
                // Scale impulses to support a variable time step.
                _impulse *= step.DtRatio;
                _motorImpulse *= step.DtRatio;

                Vec2 P = new Vec2(_impulse.X, _impulse.Y);

                b1._linearVelocity -= m1 * P;
                b1._angularVelocity -= i1 * (Vec2.Cross(r1, P) + _motorImpulse + _impulse.Z);

                b2._linearVelocity += m2 * P;
                b2._angularVelocity += i2 * (Vec2.Cross(r2, P) + _motorImpulse + _impulse.Z);
            }
            else
            {
                _impulse.SetZero();
                _motorImpulse = Fix64.Zero;
            }
        }

        internal override void SolveVelocityConstraints(TimeStep step)
        {
            Body b1 = _body1;
            Body b2 = _body2;

            Vec2 v1 = b1._linearVelocity;
            Fix64 w1 = b1._angularVelocity;
            Vec2 v2 = b2._linearVelocity;
            Fix64 w2 = b2._angularVelocity;

            Fix64 m1 = b1._invMass, m2 = b2._invMass;
            Fix64 i1 = b1._invI, i2 = b2._invI;

            //Solve motor constraint.
            if (_enableMotor && _limitState != LimitState.EqualLimits)
            {
                Fix64 Cdot = w2 - w1 - _motorSpeed;
                Fix64 impulse = _motorMass * (-Cdot);
                Fix64 oldImpulse = _motorImpulse;
                Fix64 maxImpulse = step.Dt * _maxMotorTorque;
                _motorImpulse = Box2DXMath.Clamp(_motorImpulse + impulse, -maxImpulse, maxImpulse);
                impulse = _motorImpulse - oldImpulse;

                w1 -= i1 * impulse;
                w2 += i2 * impulse;
            }

            //Solve limit constraint.
            if (_enableLimit && _limitState != LimitState.InactiveLimit)
            {
                Vec2 r1 = Box2DXMath.Mul(b1.GetXForm().R, _localAnchor1 - b1.GetLocalCenter());
                Vec2 r2 = Box2DXMath.Mul(b2.GetXForm().R, _localAnchor2 - b2.GetLocalCenter());

                // Solve point-to-point constraint
                Vec2 Cdot1 = v2 + Vec2.Cross(w2, r2) - v1 - Vec2.Cross(w1, r1);
                Fix64 Cdot2 = w2 - w1;
                Vec3 Cdot = new Vec3(Cdot1.X, Cdot1.Y, Cdot2);

                Vec3 impulse = _mass.Solve33(-Cdot);

                if (_limitState == LimitState.EqualLimits)
                {
                    _impulse += impulse;
                }
                else if (_limitState == LimitState.AtLowerLimit)
                {
                    Fix64 newImpulse = _impulse.Z + impulse.Z;
                    if (newImpulse < Fix64.Zero)
                    {
                        Vec2 reduced = _mass.Solve22(-Cdot1);
                        impulse.X = reduced.X;
                        impulse.Y = reduced.Y;
                        impulse.Z = -_impulse.Z;
                        _impulse.X += reduced.X;
                        _impulse.Y += reduced.Y;
                        _impulse.Z = Fix64.Zero;
                    }
                }
                else if (_limitState == LimitState.AtUpperLimit)
                {
                    Fix64 newImpulse = _impulse.Z + impulse.Z;
                    if (newImpulse > Fix64.Zero)
                    {
                        Vec2 reduced = _mass.Solve22(-Cdot1);
                        impulse.X = reduced.X;
                        impulse.Y = reduced.Y;
                        impulse.Z = -_impulse.Z;
                        _impulse.X += reduced.X;
                        _impulse.Y += reduced.Y;
                        _impulse.Z = Fix64.Zero;
                    }
                }

                Vec2 P = new Vec2(impulse.X, impulse.Y);

                v1 -= m1 * P;
                w1 -= i1 * (Vec2.Cross(r1, P) + impulse.Z);

                v2 += m2 * P;
                w2 += i2 * (Vec2.Cross(r2, P) + impulse.Z);
            }
            else
            {
                Vec2 r1 = Box2DXMath.Mul(b1.GetXForm().R, _localAnchor1 - b1.GetLocalCenter());
                Vec2 r2 = Box2DXMath.Mul(b2.GetXForm().R, _localAnchor2 - b2.GetLocalCenter());

                // Solve point-to-point constraint
                Vec2 Cdot = v2 + Vec2.Cross(w2, r2) - v1 - Vec2.Cross(w1, r1);
                Vec2 impulse = _mass.Solve22(-Cdot);

                _impulse.X += impulse.X;
                _impulse.Y += impulse.Y;

                v1 -= m1 * impulse;
                w1 -= i1 * Vec2.Cross(r1, impulse);

                v2 += m2 * impulse;
                w2 += i2 * Vec2.Cross(r2, impulse);
            }

            b1._linearVelocity = v1;
            b1._angularVelocity = w1;
            b2._linearVelocity = v2;
            b2._angularVelocity = w2;
        }

        internal override bool SolvePositionConstraints(Fix64 baumgarte)
        {
            // TODO_ERIN block solve with limit.

            Body b1 = _body1;
            Body b2 = _body2;

            Fix64 angularError = Fix64.Zero;
            Fix64 positionError = Fix64.Zero;

            // Solve angular limit constraint.
            if (_enableLimit && _limitState != LimitState.InactiveLimit)
            {
                Fix64 angle = b2._sweep.A - b1._sweep.A - _referenceAngle;
                Fix64 limitImpulse = Fix64.Zero;

                if (_limitState == LimitState.EqualLimits)
                {
                    // Prevent large angular corrections
                    Fix64 C = Box2DXMath.Clamp(angle, -Settings.MaxAngularCorrection, Settings.MaxAngularCorrection);
                    limitImpulse = -_motorMass * C;
                    angularError = Box2DXMath.Abs(C);
                }
                else if (_limitState == LimitState.AtLowerLimit)
                {
                    Fix64 C = angle - _lowerAngle;
                    angularError = -C;

                    // Prevent large angular corrections and allow some slop.
                    C = Box2DXMath.Clamp(C + Settings.AngularSlop, -Settings.MaxAngularCorrection, Fix64.Zero);
                    limitImpulse = -_motorMass * C;
                }
                else if (_limitState == LimitState.AtUpperLimit)
                {
                    Fix64 C = angle - _upperAngle;
                    angularError = C;

                    // Prevent large angular corrections and allow some slop.
                    C = Box2DXMath.Clamp(C - Settings.AngularSlop, Fix64.Zero, Settings.MaxAngularCorrection);
                    limitImpulse = -_motorMass * C;
                }

                b1._sweep.A -= b1._invI * limitImpulse;
                b2._sweep.A += b2._invI * limitImpulse;

                b1.SynchronizeTransform();
                b2.SynchronizeTransform();
            }

            // Solve point-to-point constraint.
            {
                Vec2 r1 = Box2DXMath.Mul(b1.GetXForm().R, _localAnchor1 - b1.GetLocalCenter());
                Vec2 r2 = Box2DXMath.Mul(b2.GetXForm().R, _localAnchor2 - b2.GetLocalCenter());

                Vec2 C = b2._sweep.C + r2 - b1._sweep.C - r1;
                positionError = C.Length();

                Fix64 invMass1 = b1._invMass, invMass2 = b2._invMass;
                Fix64 invI1 = b1._invI, invI2 = b2._invI;

                // Handle large detachment.
                Fix64 k_allowedStretch = Fix64.From(10) * Settings.LinearSlop;
                if (C.LengthSquared() > k_allowedStretch * k_allowedStretch)
                {
                    // Use a particle solution (no rotation).
                    Vec2 u = C; u.Normalize();
                    Fix64 k = invMass1 + invMass2;
                    Box2DXDebug.Assert(k > Settings.FLT_EPSILON);
                    Fix64 m = Fix64.One / k;
                    Vec2 impulse = m * (-C);
                    Fix64 k_beta = Fix64.Half;
                    b1._sweep.C -= k_beta * invMass1 * impulse;
                    b2._sweep.C += k_beta * invMass2 * impulse;

                    C = b2._sweep.C + r2 - b1._sweep.C - r1;
                }

                Mat22 K1 = new Mat22();
                K1.Col1.X = invMass1 + invMass2; K1.Col2.X = Fix64.Zero;
                K1.Col1.Y = Fix64.Zero; K1.Col2.Y = invMass1 + invMass2;

                Mat22 K2 = new Mat22();
                K2.Col1.X = invI1 * r1.Y * r1.Y; K2.Col2.X = -invI1 * r1.X * r1.Y;
                K2.Col1.Y = -invI1 * r1.X * r1.Y; K2.Col2.Y = invI1 * r1.X * r1.X;

                Mat22 K3 = new Mat22();
                K3.Col1.X = invI2 * r2.Y * r2.Y; K3.Col2.X = -invI2 * r2.X * r2.Y;
                K3.Col1.Y = -invI2 * r2.X * r2.Y; K3.Col2.Y = invI2 * r2.X * r2.X;

                Mat22 K = K1 + K2 + K3;
                Vec2 impulse_ = K.Solve(-C);

                b1._sweep.C -= b1._invMass * impulse_;
                b1._sweep.A -= b1._invI * Vec2.Cross(r1, impulse_);

                b2._sweep.C += b2._invMass * impulse_;
                b2._sweep.A += b2._invI * Vec2.Cross(r2, impulse_);

                b1.SynchronizeTransform();
                b2.SynchronizeTransform();
            }

            return positionError <= Settings.LinearSlop && angularError <= Settings.AngularSlop;
        }
    }
}
