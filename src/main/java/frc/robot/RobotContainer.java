package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIODoubleSparkFlex;

public class RobotContainer {
    public final Flywheel flywheel;

    public final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                flywheel = new Flywheel(new FlywheelIODoubleSparkFlex(IDs.SHOOTER_LEADER, IDs.SHOOTER_FOLLLOWER));
                break;
            case REPLAY:
                flywheel = new Flywheel(new FlywheelIO() {});
                break;
            default:
                flywheel = new Flywheel(new FlywheelIO() {});
                break;
        }

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        controller.a().whileTrue(flywheel.getSysIDFactory().getRoutine().quasistatic(Direction.kForward));
        controller.b().whileTrue(flywheel.getSysIDFactory().getRoutine().quasistatic(Direction.kReverse));
        controller.x().whileTrue(flywheel.getSysIDFactory().getRoutine().dynamic(Direction.kForward));
        controller.y().whileTrue(flywheel.getSysIDFactory().getRoutine().dynamic(Direction.kReverse));
    }
}
