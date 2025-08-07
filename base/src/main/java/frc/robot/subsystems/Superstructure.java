package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Superstructure extends SubsystemBase {
    CommandSwerveDrivetrain dt;
    AlgaeSubsystem algae;
    CoralSubsystem coral;
    Magic magic;
    ElevatorSubsystem elevator;

    Pose2d currentTarget = Pose2d.kZero;
    boolean closeEnoughToRaiseElevator = false;

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Poses/DesiredAPTarget", Pose2d.struct).publish();

    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("Poses/APTargetArray", Pose2d.struct).publish();

    private SwerveRequest.FieldCentricFacingAngle headingAwareRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    .withDriveRequestType(DriveRequestType.Velocity)
    .withHeadingPID(4, 0, 0); /* change theese values for your robot */


    // Intialize superstructure
    public Superstructure(CommandSwerveDrivetrain dt, AlgaeSubsystem alg, CoralSubsystem cor, Magic mag, ElevatorSubsystem elv) {
        this.dt = dt;
        this.algae = alg;
        this.coral = cor;
        this.magic = mag;
        this.elevator = elv;
    }

    // Builder Methods

    public Command autopilotToCoralTarget(boolean side) {
        APTarget alignmentTarget = magic.getAPTarget(side, dt.getState().Pose);
        return this.run(() -> {
            currentTarget = alignmentTarget.getReference();
            Translation2d velocities = getDriveVelocitiesAsTranslation2d();
            Pose2d currentPose = dt.getState().Pose;

            APResult output = Constants.AutopilotConstants.kAutopilot.calculate(currentPose, velocities, alignmentTarget);

            dt.setControl(headingAwareRequest
            .withVelocityX(output.vx())
            .withVelocityY(output.vy())
            .withTargetDirection(output.targetAngle()));
        }).until(() -> 
            Constants.AutopilotConstants.kAutopilot.atTarget(dt.getState().Pose, alignmentTarget))
        .finallyDo(
            () -> dt.stop()
        );
    }

    public Command handleAutoScore() {
        int lev = magic.getLevel();
        if (magic.getAutomation() == 1) {
            return Commands.none();
        } else {
            if (lev == 4) {
                return coral.timedScore();
            } else {
                return coral.timedLowScore();
            }
        }
    }

    public Command autopilotToTarget(APTarget target) {
        return this.run(() -> {
            Translation2d velocities = getDriveVelocitiesAsTranslation2d();
            Pose2d currentPose = dt.getState().Pose;

            APResult output = Constants.AutopilotConstants.kAutopilot.calculate(currentPose, velocities, target);

            dt.setControl(headingAwareRequest
            .withVelocityX(output.vx())
            .withVelocityY(output.vy())
            .withTargetDirection(output.targetAngle()));
        }).until(() -> 
            Constants.AutopilotConstants.kAutopilot.atTarget(dt.getState().Pose, target))
        .finallyDo(
            () -> dt.stop()
        );
    }

    // Teleop methods


    // A fully integrated method that for L2 and L3 scoring that will align, raise the elevator, score, and zero. Otherwise,
    // it will align, score, and rely on operator to zero.
    public Command AlignAndScore(boolean side) {
        int lev = magic.getLevel();
        Command setpoint = Commands.none();

        switch (lev) {
            case 3 -> setpoint = elevator.goToSetpointL3();
            case 2 -> setpoint = elevator.goToSetpointL2();
        }

        if (lev == 3 || lev == 2) {
             return Commands.parallel(
                autopilotToCoralTarget(side),
                setpoint
            ).andThen(
                handleAutoScore()
            ).andThen(elevator.zero());
        } else if (lev == 4) {
            return Commands.parallel(
                autopilotToCoralTarget(side),
                elevator.goToSetpointL4()
            ).andThen(
                handleAutoScore()
            );
        } else {
            DogLog.log("Failed to align. Level not set or invalid.", AlertType.kWarning);
            return Commands.none();
        }
    }

    public Command home() {
        return Commands.parallel(
            elevator.zero(),
            coral.passive(),
            algae.stop().withTimeout(0.1) // We only need to command the algae motor to stop momentarily
        );
    }

    // A utility command for intaking algae, and once a control is false, the elevator will raise slightly to allow the algae to be released.
    public Command intakeAlgaeAndRaiseElevatorOnFalse() {
        return runEnd(() -> {
            algae.intake();
        }, () -> {
            elevator.bumpUp(.25);
            algae.hold();
        });
    }


    // Utilities

    private Translation2d getDriveVelocitiesAsTranslation2d() {
        double vx = dt.getState().Speeds.vxMetersPerSecond;
        double vy = dt.getState().Speeds.vyMetersPerSecond;
        Rotation2d theta = dt.getState().Pose.getRotation();

        return new Translation2d(vx, vy).rotateBy(theta);
    }

    @Override
    public void periodic() {
        publisher.set(currentTarget);
        arrayPublisher.set(new Pose2d[] {currentTarget});
    }
}
