﻿/*
 * Window Simulation Copyright © Ben Ukhanov 2021
 */

using OpenTK;
using OpenTK.Graphics.OpenGL;

namespace Box2D.Window
{
    public class CameraView : IView
    {
        public Vector2 Position { get; set; }

        public float Zoom { get; set; }

        public CameraView(Vector2 position, float zoom)
        {
            Position = position;
            Zoom = zoom;
        }

        public void Update()
        {
            GL.LoadIdentity();

            var transform = Matrix4.Identity;
            var translation = Matrix4.CreateTranslation(-Position.X, -Position.Y, 0);
            var scale = Matrix4.CreateScale(Zoom, Zoom, 1.0f);

            transform = Matrix4.Mult(transform, translation);
            transform = Matrix4.Mult(transform, scale);

            GL.MultMatrix(ref transform);
        }
    }
}