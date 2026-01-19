package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Commands;
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
        flywheel =
            new Flywheel(new FlywheelIODoubleSparkFlex(IDs.SHOOTER_LEADER, IDs.SHOOTER_FOLLLOWER));
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
    controller
        .a()
        .whileTrue(flywheel.getSysIDFactory().getRoutine().quasistatic(Direction.kForward));
    controller
        .b()
        .whileTrue(flywheel.getSysIDFactory().getRoutine().quasistatic(Direction.kReverse));
    controller.x().whileTrue(flywheel.getSysIDFactory().getRoutine().dynamic(Direction.kForward));
    controller.y().whileTrue(flywheel.getSysIDFactory().getRoutine().dynamic(Direction.kReverse));

    controller
        .leftBumper()
        .whileTrue(
            Commands.runEnd(
                () -> {
                  System.out.println("Command Set Voltage");
                  flywheel.setVoltage(Volts.of(5));
                },
                () -> {
                  flywheel.setVoltage(Volts.zero());
                },
                flywheel));

    controller.rightBumper().whileTrue(flywheel.runVelocity());
  }
}
