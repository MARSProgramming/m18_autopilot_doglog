package frc.robot.constants;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;

public class TunableTransforms {
    public static class AlignTransforms {
        private static final DoubleSubscriber RightXLower = DogLog.tunable("Transforms/Right/XLow", .47);
        private static final DoubleSubscriber RightYLower = DogLog.tunable("Transforms/Right/YLow", .42);
        private static final DoubleSubscriber RightXL4 = DogLog.tunable("Transforms/Right/XL4", .455);
        private static final DoubleSubscriber RightYL4 = DogLog.tunable("Transforms/Right/YL4", .43);

        private static final DoubleSubscriber RightRot = DogLog.tunable("Transforms/Right/Rot", 90.0);

        private static final DoubleSubscriber LeftXLower = DogLog.tunable("Transforms/Left/XLow", .47);
        private static final DoubleSubscriber LeftYLower = DogLog.tunable("Transforms/Left/YLow", .03);
        private static final DoubleSubscriber LeftXL4 = DogLog.tunable("Transforms/Left/XL4", .44);
        private static final DoubleSubscriber LeftYL4 = DogLog.tunable("Transforms/Left/YL4", .05);

        private static final DoubleSubscriber LeftRot = DogLog.tunable("Transforms/Left/Rot", 90.0);

        private static final BooleanSubscriber tuningEnabled = DogLog.tunable("Transforms/TuningEnabled", false);

    }

        public static class Transforms {
        public static Transform2d RIGHT_LOW_TRANSFORM = new Transform2d(AlignTransforms.RightXLower.get(), AlignTransforms.RightYLower.get(), new Rotation2d(Math.toRadians(AlignTransforms.RightRot.get())));
        public static Transform2d RIGHT_L4_TRANSFORM = new Transform2d(AlignTransforms.RightXL4.get(), AlignTransforms.RightYL4.get(), new Rotation2d(Math.toRadians(AlignTransforms.RightRot.get())));
        public static Transform2d LEFT_LOW_TRANSFORM = new Transform2d(AlignTransforms.LeftXLower.get(), AlignTransforms.LeftYLower.get(), new Rotation2d(Math.toRadians(AlignTransforms.LeftRot.get())));
        public static Transform2d LEFT_L4_TRANSFORM = new Transform2d(AlignTransforms.LeftXL4.get(), AlignTransforms.LeftYL4.get(), new Rotation2d(Math.toRadians(AlignTransforms.LeftRot.get())));
    }

    // During testing, this is what will allow us to constantly change the transforms.
    public static void periodic() {

        if (AlignTransforms.tuningEnabled.get()) {
             Transforms.RIGHT_LOW_TRANSFORM = new Transform2d(AlignTransforms.RightXLower.get(), AlignTransforms.RightYLower.get(), new Rotation2d(Math.toRadians(AlignTransforms.RightRot.get())));
             Transforms.RIGHT_L4_TRANSFORM = new Transform2d(AlignTransforms.RightXL4.get(), AlignTransforms.RightYL4.get(), new Rotation2d(Math.toRadians(AlignTransforms.RightRot.get())));
             Transforms.LEFT_LOW_TRANSFORM = new Transform2d(AlignTransforms.LeftXLower.get(), AlignTransforms.LeftYLower.get(), new Rotation2d(Math.toRadians(AlignTransforms.LeftRot.get())));
             Transforms.LEFT_L4_TRANSFORM = new Transform2d(AlignTransforms.LeftXL4.get(), AlignTransforms.LeftYL4.get(), new Rotation2d(Math.toRadians(AlignTransforms.LeftRot.get())));
        }

    }


}
