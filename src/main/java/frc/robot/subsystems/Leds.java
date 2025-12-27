// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leds extends SubsystemBase {
    private static Leds mInstance;

    public static Leds getInstance(){
        if(mInstance == null){
            mInstance = new Leds();
        }
        return mInstance;
    }

    private final CANdle m_candle;
    private final int ledCount = 65;

    public Leds() {

        m_candle = new CANdle(44, "rio");
        var cfg = new CANdleConfiguration();
        /* set the LED strip type and brightness */
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        cfg.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
        cfg.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
        /* disable status LED when being controlled */
        cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        m_candle.getConfigurator().apply(cfg);
  }

  public void setColor(int r, int g, int b){
    m_candle.setControl(new SolidColor(0, ledCount).withColor(new RGBWColor(r, g, b)));
  }



  
  @Override
  public void periodic() {

  }


}
