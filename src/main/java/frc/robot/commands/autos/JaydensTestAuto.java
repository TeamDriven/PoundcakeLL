package frc.robot.commands.autos;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Controls;
import frc.robot.RobotContainer;

public class JaydensTestAuto {
    private final AutoFactory m_factory;

    public JaydensTestAuto(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine jaydensTestAuto() {
        final AutoRoutine routine = m_factory.newRoutine("Jaydens Test");
       // final AutoTrajectory jaydensTestAuto = ChoreoTraj.Jaydens_test.asAutoTraj(routine);
        final AutoTrajectory jaydensTestAuto = routine.trajectory("Jaydens_test", 0);

        routine.active().onTrue(
                jaydensTestAuto.resetOdometry().andThen(
                        Commands.print("Auto start works"))
                        .andThen(Commands.runOnce(() -> SmartDashboard.putBoolean("test1", true)))
                        .andThen(jaydensTestAuto.cmd())
                        .andThen(Commands.runOnce(() -> SmartDashboard.putBoolean("test2", true))));

        jaydensTestAuto.done().onTrue(Commands.sequence(
                // RobotContainer.drivetrain.applyRequest(Controls.driveRequest()),
                Commands.runOnce(() -> SmartDashboard.putBoolean("test3", true))));

        return routine;
    }
}