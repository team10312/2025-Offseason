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

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import java.lang.reflect.Member;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase {
    private static Elevator mInstance;

    private static Elevator getInstance(){
        if(mInstance == null){
            mInstance = new Elevator();
        }
        return mInstance;
    }

    private final TalonFX mElevator;
    final MotionMagicVoltage mMotionMagicRequest = new MotionMagicVoltage(0);

    public Elevator() {

    mElevator = new TalonFX(29, "rio");

    var talonFXConfigs = new TalonFXConfiguration();
    var talonFXConfigurator = mElevator.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = 80;
    limitConfigs.SupplyCurrentLimit = 40;

    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);

    var Slot0Configs = talonFXConfigs.Slot0;
    Slot0Configs.kS = 0.35;
    Slot0Configs.kV = 0.12;
    Slot0Configs.kA = 0.01;
    Slot0Configs.kP = 2.5;
    Slot0Configs.kI = 0;
    Slot0Configs.kD = 0.1;
    Slot0Configs.kG = 0.35; 

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30;
    motionMagicConfigs.MotionMagicAcceleration = 300;
    motionMagicConfigs.MotionMagicJerk = 3000;

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    mElevator.getConfigurator().apply(talonFXConfigs);
  }

public double rotationsToInches(double rotations){
    //should add home pos
    return (4 + (rotations * Math.PI * 1.729) / 20);
}

public double inchesToRotations(double inches){
    return (inches - 4) / ((Math.PI * 1.729) / 20);
}

public double getInches(){
    return rotationsToInches(mElevator.getPosition().getValueAsDouble());
}

public void setInches(double inches){
    mElevator.setControl(mMotionMagicRequest.withPosition(inchesToRotations(inches)));
    // mElevator.setPosition(inchesToRotations(inches));
}

public void setPercentOutput(double percent){
    mElevator.set(percent);
  }

public void resetHomePosition(){
    mElevator.setPosition(0);
}

public double getCurrent(){
    return mElevator.getStatorCurrent().getValueAsDouble();
}

public boolean currentSpiked(){
    if(getCurrent() > 30){
        return true;
    }
    else{
        return false;
    }
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Inches", getInches());
    SmartDashboard.putNumber("Elevator Current", getCurrent());
  }


}
