package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class SetFlywheelCommand extends Command {
  private final Flywheel flywheel;

  private final Supplier<AngularVelocity> flywheelRPM;

  public SetFlywheelCommand(Flywheel flywheel, Supplier<AngularVelocity> flywheelRPM) {
    this.flywheel = flywheel;
    addRequirements(flywheel);

    this.flywheelRPM = flywheelRPM;
  }

  @Override
  public void execute() {
    flywheel.setVelocity(flywheelRPM.get());
  }
}
