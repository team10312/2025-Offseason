// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.Constants;
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


  }

public double rotationsToInches(double rotations){
    //should add home pos
    return ((rotations * Math.PI * 1.729) / 20);
}

public double inchesToRotations(double inches){
    return ((inches) * 20) / (Math.PI * 1.729);}

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


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Inches", getInches());
  }


}