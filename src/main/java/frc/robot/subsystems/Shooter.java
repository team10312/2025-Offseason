// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance;

    private static Shooter getInstance(){
        if(mInstance == null){
            mInstance = new Shooter();
        }
        return mInstance;
    }

    private final TalonFX mLeftShooter, mRightShooter;
    final MotionMagicVoltage mMotionMagicRequest = new MotionMagicVoltage(0);

    public Shooter() {

    mLeftShooter = new TalonFX(0, "rio");
    mRightShooter = new  TalonFX(0, "rio");


  }


public void setSpeed(double leftSpeed, double rightSpeed){
    mLeftShooter.set(leftSpeed);
    mRightShooter.set(rightSpeed);
}


  @Override
  public void periodic() {

  }


}