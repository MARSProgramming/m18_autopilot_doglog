package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.reflect.Field;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Superstructure extends SubsystemBase {
    CommandSwerveDrivetrain dt;
    AlgaeSubsystem algae;
    CoralSubsystem coral;
    FieldmapUtilities fieldmap;
    ElevatorSubsystem elevator;


    private SwerveRequest.FieldCentricFacingAngle headingAwareRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
    .withDriveRequestType(DriveRequestType.Velocity)
    .withHeadingPID(4, 0, 0); /* change theese values for your robot */

    // Intialize superstructure
    public Superstructure(CommandSwerveDrivetrain dt, AlgaeSubsystem alg, CoralSubsystem cor, FieldmapUtilities fld, ElevatorSubsystem elv) {
        this.dt = dt;
        this.algae = alg;
        this.coral = cor;
        this.fieldmap = fld;
        this.elevator = elv;


    }

    public Command AutopilotNearestReefFace(boolean side) {
        APTarget alignmentTarget = fieldmap.getAPTarget(side, dt.getState().Pose);
    }
}
