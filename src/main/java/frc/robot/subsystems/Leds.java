// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leds extends SubsystemBase {
    private static Leds mInstance;

    private static Leds getInstance(){
        if(mInstance == null){
            mInstance = new Leds();
        }
        return mInstance;
    }

    private final CANdle m_leds;
    private final int ledCount = 20;

    public Leds() {

    m_leds = new CANdle(0, "rio");

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.statusLedOffWhenActive = true;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = 0.1;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    m_leds.configAllSettings(configAll, 100);
  }

  public void setColor(int r, int g, int b){
    m_leds.setLEDs(r, g, b, 0, 0, ledCount);
  }

  @Override
  public void periodic() {

  }


}
