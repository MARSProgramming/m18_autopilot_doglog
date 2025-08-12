package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class CANdleSubsystem extends SubsystemBase {
    CANdle ledController;

    public CANdleSubsystem() {
        ledController = new CANdle(Constants.CAN_IDS.LED);  // Create CANdle object with the provided ID
        CANdleConfiguration config = new CANdleConfiguration();  // Configuration object for CANdle
        config.stripType = CANdle.LEDStripType.RGBW;  // Set the LED strip type to RGB
        ledController.configAllSettings(config);  // Apply configuration to CANdle
        }

        public enum COLOR {
        RED(255, 0, 0, 0),
        GREEN(0, 255, 0, 0),
        BLUE(0, 0, 255, 0),
        WHITE(0, 0, 0, 255),
        PURPLE(128, 0, 128, 0),
        ORANGE(255, 165, 0, 0);

        private final int r;
        private final int g;
        private final int b;
        private final int w;

        private COLOR(int r, int g, int b, int w) {
            this.r = r;
            this.g = g;
            this.b = b;
            this.w = w;
        }

        public int getR() {
            return r;
        }

        public int getG() {
            return g;
        }

        public int getB() {
            return b;
        }

        public int getW() {
            return w;
        }
        }


    // Set the color of the CANdle LEDs, non-terminating. If termination is desired, apply a timeout.
    public Command setColor(COLOR color) {
        return runOnce(
            () -> ledController.setLEDs(color.getR(), color.getG(), color.getB())
            );
    }

    // Define the default behavior, which is to have RED LEDs.

    public Command defaultcmd() {
        return runOnce(
            () -> ledController.setLEDs(255, 0, 0)
        );
    }

    public Command rainbowAnimation() {
        return runOnce(
            () -> setRainbowAnimation()
        );
    }

    public Command strobeAnimation(COLOR color) {
        return runOnce(
            () -> setStrobeAnimation(color)
        );
    }

    // methods

        // Method to set a rainbow animation across the LED strip
    private void setRainbowAnimation() {
        Animation rainbow = new RainbowAnimation();  // Create rainbow animation
        ledController.animate(rainbow);  // Apply the rainbow animation to the LED strip
    }

    // Method to set a strobe animation with the given color and speed
    private void setStrobeAnimation(COLOR color) {
        Animation strobe = new StrobeAnimation(color.getR(), color.getG(), color.getB());  // Create strobe animation
        ledController.animate(strobe);  // Apply the strobe animation to the LED strip
    }

    

    @Override
    public void periodic() {
        if (ledController.getTemperature() > 60.0) {
            DogLog.logFault("HOT CAUTION - CANdle", AlertType.kWarning);
        }
    }
}
