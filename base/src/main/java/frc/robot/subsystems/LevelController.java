package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// A utility class for handling the level of automation as well as  
public class LevelController extends SubsystemBase {

    int level;
    private final BooleanSubscriber enableAutoScore = DogLog.tunable("System/LC/Autoscore", true);


    public LevelController() {
        level = 1;
    }

    public int getLevel() {
        return level;
    }

    public void setLevel(int lev) {
        level = lev;
    }


    public Command setLevelCommand(int lev) {
        return runOnce(
            () -> level = lev
        );
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("System/LC/Level", level);
        SmartDashboard.putBoolean("System/LC/RedundantAutomationLogger", enableAutoScore.get());
    }


}