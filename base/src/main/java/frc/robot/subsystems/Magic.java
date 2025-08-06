package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.therekrab.autopilot.APTarget;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.TunableTransforms;

// A utility class for handling the level of automation as well as  
public class FieldmapUtilities extends SubsystemBase {

    int level;
    private Alliance alliance;
    private final BooleanSubscriber enableAutoScore = DogLog.tunable("System/LC/Autoscore", true);
    static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private List<Pose2d> reefPoses;

    public FieldmapUtilities(Alliance dsAlliance) {
        if (dsAlliance != null) {
            alliance = dsAlliance;
            DogLog.log("System/LC/AllianceGenerated", true);
        }

        reefPoses = generateList();
        DogLog.log("System/LC/ListGenerated", true);
    }

    public int getLevel() {
        return level;
    }

    public void setLevel(int lev) {
        level = lev;
    }

    public static List<Pose2d> createAprilTagPose2dList(int[] tagIntegers) {
        List<Pose2d> tags = new ArrayList<>();
        for (int i = 0; i < tagIntegers.length; i++) {
        int tagInteger = tagIntegers[i];
        Pose2d tagPose = field.getTagPose(tagInteger).get().toPose2d();
        tags.add(tagPose);
        }
    return tags;
    }

    public List<Pose2d> generateList() {
            return (
                ((alliance == Alliance.Blue) ? createAprilTagPose2dList(Constants.VisionFiducials.BLUE_CORAL_TAGS) : createAprilTagPose2dList(Constants.VisionFiducials.RED_CORAL_TAGS))
            );
    }

    public APTarget getAPTarget(boolean left, Pose2d currentDrivetrainPose) {
        Pose2d poseToTarget = Pose2d.kZero;
        APTarget AutopilotCompatibleTarget = new APTarget(poseToTarget);

        if (reefPoses.isEmpty()) {
            DogLog.logFault("System: Pose list not generated!", AlertType.kWarning);
            return new APTarget(currentDrivetrainPose);
        } else {
            poseToTarget = currentDrivetrainPose.nearest(reefPoses);

            if (poseToTarget.getTranslation().getDistance(currentDrivetrainPose.getTranslation()) > 3) {
                DogLog.logFault("Attempted to align from too far of a location.", AlertType.kWarning);
                return new APTarget(currentDrivetrainPose);
            }
        }

        if (level == 4) {
            if (left) {
                AutopilotCompatibleTarget = new APTarget(poseToTarget.transformBy(TunableTransforms.Transforms.LEFT_L4_TRANSFORM));
            } else {
                AutopilotCompatibleTarget = new APTarget(poseToTarget.transformBy(TunableTransforms.Transforms.RIGHT_L4_TRANSFORM));
            }
        } else {
            if (left) {
                AutopilotCompatibleTarget = new APTarget(poseToTarget.transformBy(TunableTransforms.Transforms.LEFT_LOW_TRANSFORM));
            } else {
                AutopilotCompatibleTarget = new APTarget(poseToTarget.transformBy(TunableTransforms.Transforms.RIGHT_LOW_TRANSFORM));
            }        
        }

        return AutopilotCompatibleTarget;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("System/LC/Level", level);
        SmartDashboard.putBoolean("System/LC/RedundantAutomationLogger", enableAutoScore.get());
    }


}