// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import java.lang.reflect.Member;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Leds extends SubsystemBase {
    private static Leds mInstance;

    private static Leds getInstance(){
        if(mInstance == null){
            mInstance = new Leds();
        }
        return mInstance;
    }

    private final CANdle m_candle;
    private final int ledCount;

    public Leds() {

    m_candle = new CANdle(0, "rio");
    ledCount = 20;

  }

  public void setLeds(int r, int g, int b){
    m_candle.setLEDs(r, g, b);
  }



  @Override
  public void periodic() {

  }


}
