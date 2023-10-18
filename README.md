# Simulation Library

The simulation library is a library of simulations used to simulate a physical model of a robot system. While these simulations may be inacurrate in terms of the PID values they generate, they are good at testing control logic.

Currently, the library supports the following simulations:

- [Single jointed arm](www.google.com)
- [Double jointed arm](www.google.com)
- [Telescope on a single jointed arm](www.google.com)
- [Elevator](www.google.com)
- [Flywheel](www.google.com)
- [Swerve module](www.google.com)
- [Swerve drive](www.google.com)
- [Differential drive](www.google.com)

Many of these simulations are natively supported by WPILib and require very little setup. Others require a bit more knowledge, such as the double jointed arm or swerve drive.

The code is fairly well documented (compared to our old code at least), and should be easy to import into another project. The code cannot just be directly copy and pasted in one go, but it provides a basis for how to organise simulation segements of the project.

For users that use [Advantage Scope](www.google.com) to visualise their code, drag and drop the NT4 object under SmartDashboard into the respective tab's field.