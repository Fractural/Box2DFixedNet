![Box2D Logo](https://box2d.org/images/logo.svg)

# Box2D Fixed.NET 

Fixed-point Box2D library in C#. It combines the [FixedMath.Net](https://github.com/asik/FixedMath.Net) repo with codingben (Ben Ukhanov)'s [box2D-netstandard](https://github.com/codingben/box2d-netstandard).

## Source

- [Ported C# Box2DX - Ihar Kalasouski](https://code.google.com/archive/p/box2dx/)
- [C++ Box2D Original - Erin Catto](https://github.com/erincatto/box2d)

## Contributing

Anyone who wants to contribute to this repository:
- The changes will be in the new branch (feature/new or feature/bug-fix).
- The new pull request will be started.
- The new version will be published.

## Features

### Collision

- Continuous collision detection.
- Contact callbacks: add, persist, remove.
- Convex polygons and circles.
- Multiple shapes per body.
- One-shot contact manifolds.
- Incremental sweep-and-prune broadphase.
- Efficient pair management.
- Fast broadphase AABB queries.
- Collision groups and categories.

### Physics

- Continuous physics with time of impact solver.
- Persistent body-joint-contact graph.
- Island solution and sleep management.
- Contact, friction, and restitution.
- Stable stacking with a linear-time solver.
- Revolute, prismatic, distance, pulley, gear, and mouse joints.
- Joint limits, motors, and friction.
- Momentum decoupled position correction.
- Fairly accurate reaction forces/impulses.

### System

- Centralized tuning parameters.
- Pure .NET Standard 2.0+ library.
- Please [See .NET Implementation Support](https://docs.microsoft.com/en-us/dotnet/standard/net-standard).

## Documentation

- [Manual](https://box2d.org/documentation/)
- [Reddit](https://www.reddit.com/r/box2d/)
- [Discord](https://discord.gg/NKYgCBP)

## License

Original C++ Box2D is developed by Erin Catto, under the [MIT license](https://en.wikipedia.org/wiki/MIT_License).
