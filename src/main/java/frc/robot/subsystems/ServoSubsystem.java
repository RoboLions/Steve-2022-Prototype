// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ServoSubsystem extends SubsystemBase {

  private static final Servo servo = RobotMap.servo;

  public ServoSubsystem() {
    servo.setBounds(2, 1.8, 1.5, 1.2, 1);
  }

  public void moveFServo() {
    servo.setSpeed(1);
  }

  public void moveBServo() {
    servo.setSpeed(-1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
