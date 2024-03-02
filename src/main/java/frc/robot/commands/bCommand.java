// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class bCommand extends Command {
  /** Creates a new ShootNote. */
  private int ticks;
  private DriveSubsystem m_robodrive;
  public bCommand(DriveSubsystem robodrive) {
    addRequirements(robodrive);
    m_robodrive = robodrive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticks=0;
    m_robodrive.driveMode=decrementSpeed(m_robodrive.driveMode);
    m_robodrive.speedModifier=button(m_robodrive.speedModifier, m_robodrive.driveMode);
    SmartDashboard.putBoolean("bpressed", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
    ticks++;
    if (ticks%10==0){
      m_robodrive.driveMode=decrementSpeed(m_robodrive.driveMode);
      m_robodrive.speedModifier=button(m_robodrive.speedModifier, m_robodrive.driveMode);
    }
  }
  public double button(double speedModifier, int mod){
    
      if (mod==0){
        speedModifier=0.3;
      }
      else if (mod==1){
        speedModifier=0.475;
      }
      else if (mod==2){
        speedModifier=0.65;
      }
      else if (mod==3){
        speedModifier=.815;
      }
      else if (mod==4){
        speedModifier=1.0;
      }
    return speedModifier;
  }
  public int decrementSpeed(int speedMode){
    if (speedMode>=0){
      return speedMode--;
    }
    else{
      return speedMode;
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("bpressed", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
