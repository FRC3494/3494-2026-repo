package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class SetHoodCommand extends Command {
  private final Hood hood;

  private final Supplier<Rotation2d> hoodAngle;

  public SetHoodCommand(Hood hood, Supplier<Rotation2d> hoodAngle) {
    this.hood = hood;
    addRequirements(hood);

    this.hoodAngle = hoodAngle;
  }

  @Override
  public void execute() {
    hood.setPosition(hoodAngle.get());
  }
}
