package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IDs;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIODoubleSparkFlex;
import frc.robot.utils.State2;
import frc.robot.utils.StateMachine2;

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
    // controller
    //     .a()
    //     .whileTrue(flywheel.getSysIDFactory().getRoutine().quasistatic(Direction.kForward));
    // controller
    //     .b()
    //     .whileTrue(flywheel.getSysIDFactory().getRoutine().quasistatic(Direction.kReverse));
    // controller.x().whileTrue(flywheel.getSysIDFactory().getRoutine().dynamic(Direction.kForward));
    // controller.y().whileTrue(flywheel.getSysIDFactory().getRoutine().dynamic(Direction.kReverse));

    StateMachine2 flywheelStates =
        new StateMachine2(flywheel).restartOnTeleop().publishToNT("FlywheelStates");
    State2 RUNNINGREV = flywheelStates.state("RUNNINGREV").whileRunning(flywheel.runVelocity());
    // State2 RUNNINGWPI =
    // flywheelStates.state("RUNNINGWPI").whileRunning(flywheel.runVelocityWPI());
    State2 OFF =
        flywheelStates
            .defaultState("OFF")
            .whileRunning(flywheel.runVelocity(RotationsPerSecond.of(400 / 60.0)))
            .onTrigger(controller.rightBumper(), () -> RUNNINGREV);
    RUNNINGREV.onTrigger(controller.rightBumper(), () -> OFF);
    // RUNNINGWPI.onTrigger(controller.rightBumper(), () -> OFF);
  }
}
