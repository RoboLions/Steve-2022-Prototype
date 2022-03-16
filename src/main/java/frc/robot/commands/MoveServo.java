// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.ServoSubsystem;

public class MoveServo extends CommandBase {

  private final ServoSubsystem servoSubsystem;
  private final XboxController manipulatorController = RobotContainer.manipulatorController;

  public MoveServo(ServoSubsystem servo) {
    servoSubsystem = servo;
    addRequirements(servoSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (manipulatorController.getYButton()) {
      servoSubsystem.moveFServo();
    } 
    
    
    if (manipulatorController.getBButton()) {
      servoSubsystem.moveBServo();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
