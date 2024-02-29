// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootNoteCommand extends Command {
  private ShooterSubsystem m_shooter;
  private int ticks;
  double getLeftTriggerAxis;
  /** Creates a new ShootNote. */
  public ShootNoteCommand(ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    getLeftTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    m_shooter.setShooterSpeed(-getLeftTriggerAxis);
    ticks = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getLeftTriggerAxis=RobotContainer.m_armController.getRightTriggerAxis(); 
    ticks +=1;
    if (ticks == 25) {
      m_shooter.setFeederSpeed(-getLeftTriggerAxis);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterSpeed(0);
    m_shooter.setFeederSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
