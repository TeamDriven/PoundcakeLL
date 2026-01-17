package frc.robot;

import frc.robot.generated.TunerConstants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Constants {
    public class DrivetrainConst {
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    // Intake

    // Pivot
    public static final double PIVOT_REST_POS = 0; // TODO find the actual value
    public static final double pivotInternalTicksPerDegree = 1; // TODO find the actual value
    public static final double pivotTicksPerDegree = 1; // TODO find the actual value
    public static final double pivotOffset = 1; // TODO find the actual value

    // Shooter

    // AngleController
    public static final double ANGLE_CONTROLLER_REST_POS = 0;
    public static final double GEAR_RATIO = 1;
}
