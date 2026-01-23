package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;

public class Controls {
    public static final boolean isRightStickDrive = true;

    // public Command driveCommand() {
    //     // return drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getRightY() * DrivetrainConst.MaxSpeed) // Drive forward with
    //     //                                                                                            // negative Y
    //     //                                                                                            // (forward)
    //     //                 .withVelocityY(joystick.getRightX() * DrivetrainConst.MaxSpeed) // Drive left with negative X (left)
    //     //                 .withRotationalRate(-joystick.getLeftX() * DrivetrainConst.MaxAngularRate) // Drive counterclockwise with
    //     //                                                                             // negative X (left)
    //     //         );
    // }
    // if(m_controller.getAButton())
    // {
    //     final var rot_limelight = limelight_aim_proportional();
    //     rot = rot_limelight;

    //     final var forward_limelight = limelight_range_proportional();
    //     xSpeed = forward_limelight;

    //     //while using Limelight, turn off field-relative driving.
    //     fieldRelative = false;
    // }
}
