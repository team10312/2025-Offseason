package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Leds.AnimationType;

public class AnimateLed extends Command {

  private final AnimationType animationType;
  private final Integer r;
  private final Integer g;
  private final Integer b;

  private final Leds m_leds = Leds.getInstance();

  public AnimateLed(AnimationType animationType) {
    this.animationType = animationType;
    r = null;
    g = null;
    b = null;

    addRequirements(m_leds);
  }

  public AnimateLed(AnimationType animationType, int r, int g, int b) {
    this.animationType = animationType;
    this.r = r;
    this.g = g;
    this.b = b;

    addRequirements(m_leds);
  }

  @Override
  public void initialize() {
    if (r == null) {
      m_leds.setAnimation(animationType);
    } else {
      m_leds.setAnimation(animationType, r, g, b);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
