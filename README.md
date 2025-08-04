The goal of this project is to optimize usage of a traditional command-based framework. We will implement best-practices for structuring commands and attempt to streamline the development process. We also desire the creation of a robust drive-to-point implementation to replace path planning when possible.

Currently, implementation is planned of the following:
https://therekrab.github.io/autopilot/

This is a new solution for alignment and drive-to-point that is allegedly more robust than simple PID-to-pose commands. However, this project will also investigate an efficient and proper implementation of drive-to-pose with a holonomic drive controller, such as FRC Team 695's reef-repulsion system https://github.com/FRCTeam695/Goldfish

Implementation of DogLog will be done in the main branch:
https://doglog.dev/

AdvantageKit, while offering robust simulation and replay capabilities, offers complexity that we will likely not take advantage of in the following season. This intent of this project is to reduce development time while also understanding how to write extremely efficient code, and DogLog will allow us to do just that. One MAJOR advantage of DogLog is being able to quick-access logs through a USB stick. This is extremely appealing as we will not have to remain plugged in to the Robot and rely on the RIO for file storage. 

DogLog does not natively support logging Poses, so we will utilize the AdvantageKit fallback:

# Goals
1. Write a much more refined version of competition Marvin 18 code
2. Implement Autopilot from therekrab (3414)
3. Implement DogLog

```
Pose2d poseA = new Pose2d();
Pose2d poseB = new Pose2d();

StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("MyPose", Pose2d.struct).publish();
StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

periodic() {
  publisher.set(poseA);
  arrayPublisher.set(new Pose2d[] {poseA, poseB});
}
```

DogLog also features a working live-tuning (for PIDs and whatnot) interface so we don't have to redploy every time.

