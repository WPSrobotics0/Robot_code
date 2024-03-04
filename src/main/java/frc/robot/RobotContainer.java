// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.ClimbExtendCommand;
import frc.robot.commands.ClimbRetractCommand;
//import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.ShootNoteCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.aCommand;
import frc.robot.commands.bCommand;
import frc.robot.commands.asCommand;
import frc.robot.commands.bsCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final IntakeNoteCommand m_IntakeNoteCommand = new IntakeNoteCommand(m_ShooterSubsystem);
  private final ShootNoteCommand m_ShootNoteCommand = new ShootNoteCommand(m_ShooterSubsystem);
  private final ClimbExtendCommand m_ClimbExtendCommand = new ClimbExtendCommand(m_ClimbSubsystem);
  private final ClimbRetractCommand m_ClimbRetractCommand = new ClimbRetractCommand(m_ClimbSubsystem);
  private final aCommand m_ACommand = new aCommand(m_robotDrive);
  private final bCommand m_BCommand = new bCommand(m_robotDrive);
  private final asCommand m_ASCommand = new asCommand(m_ShooterSubsystem);
  private final bsCommand m_BSCommand = new bsCommand(m_ShooterSubsystem);
    private final CANSparkMax m_leftShooter = new CANSparkMax(SubsystemConstants.kLeftShooterCanId, MotorType.kBrushed);
  private final CANSparkMax m_leftFeeder = new CANSparkMax(SubsystemConstants.kLeftFeederCanId, MotorType.kBrushed);
  private final CANSparkMax m_rightShooter = new CANSparkMax(SubsystemConstants.kRightShooterCanId, MotorType.kBrushed);
  private final CANSparkMax m_rightFeeder = new CANSparkMax(SubsystemConstants.kRightFeederCanId, MotorType.kBrushed);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OIConstants.kDriverControllerPort);
  public final static CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort0);

  public final static CommandXboxController m_armController = new CommandXboxController(
      OIConstants.kDriverControllerPort1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldRelative, false),
            m_robotDrive));
  }
  boolean fieldRelative = true;

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_armController.rightTrigger(0.15).whileTrue(m_ShootNoteCommand);
    m_armController.leftTrigger(0.15).whileTrue(m_IntakeNoteCommand);
    
    m_driverController.y().onTrue(new InstantCommand(()->fieldRelative = false));
    m_driverController.x().onTrue(new InstantCommand(()->fieldRelative = true));

    //m_driverController.start().onTrue(new InstantCommand(()->m_robotDrive.zeroHeading()));
    // m_armController.a().whileTrue(m_ASCommand);
    // m_armController.b().whileTrue(m_BSCommand);

    m_armController.rightBumper().whileTrue(m_ClimbExtendCommand);
    m_armController.leftBumper().whileTrue(m_ClimbRetractCommand);

    // m_driverController.a().whileTrue(m_ACommand);
    // m_driverController.b().whileTrue(m_BCommand);

     chooser = new SendableChooser<Command>();
     chooser.addOption("Time", time());
     SmartDashboard.putData(chooser);
     chooser.addOption("Time2", time2());
     SmartDashboard.putData(chooser);
  }

   SendableChooser<Command> chooser;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    Command selected = chooser.getSelected();
    if (selected != null) {
       return selected;
     } else {
       return time();
     }

    //return time();
  }

  public Command time() {
    // An example command will be run in autonomous
    return new StartEndCommand(() -> {
      m_robotDrive.drive(0.25, 0, 0, false, false);
    }, () -> {
      m_robotDrive.drive(0, 0, 0, false, false);
    }, m_robotDrive).withTimeout(5);
  }

    public Command time2() {
    // An example command will be run in autonomous
    return new StartEndCommand(() -> {
      m_leftShooter.set(1);
      m_leftFeeder.set(1);
      m_rightShooter.follow(m_leftShooter,true);
      m_rightFeeder.follow(m_leftFeeder,true);
      
    }, () -> {
      m_leftShooter.set(0);
      m_leftFeeder.set(0);
      m_rightShooter.follow(m_leftShooter,true);
      m_rightFeeder.follow(m_leftFeeder,true);
    }, m_robotDrive).withTimeout(5).andThen(()->{
      m_robotDrive.drive(-0.25, 0, 0, false, false);
    }).withTimeout(5).andThen(()->{m_robotDrive.drive(0, 0, 0, false, false);});
  }
}
