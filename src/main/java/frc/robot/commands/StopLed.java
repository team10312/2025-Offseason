package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Leds.AnimationType;

public class StopLed extends Command {
  private final Leds m_leds = Leds.getInstance();

  public StopLed() {
    addRequirements(m_leds);
  }



  @Override
  public void initialize() {
    m_leds.off();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
