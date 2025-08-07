package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.reflect.Field;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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


    public Command AlignAndScore(boolean side) {
        int lev = magic.getLevel();
        Command setpoint = Commands.none();

        if (lev == 4) {
            setpoint = elevator.goToSetpointL4();
        }
        else if (lev == 3) {
            setpoint = elevator.goToSetpointL3();
        }
        else if (lev == 2) {
            setpoint = elevator.goToSetpointL2();
        }

        return Commands.parallel(
            autopilotToCoralTarget(side),
            setpoint
        ).andThen(
            handleAutoScore()
        );
    }

    // Utilities

    public boolean closeEnoughCheck(Pose2d target, Pose2d current) {
        if (target.getTranslation().getDistance(current.getTranslation()) < 1) {
            return true;
        } else {
            return false;
        }
    }

    private Translation2d getDriveVelocitiesAsTranslation2d() {
        double vx = dt.getState().Speeds.vxMetersPerSecond;
        double vy = dt.getState().Speeds.vyMetersPerSecond;
        Rotation2d theta = dt.getState().Pose.getRotation();

        return new Translation2d(vx, vy).rotateBy(theta);
    }


}
