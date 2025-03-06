// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // Subsystem delclarations
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/545"));                                                                                
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
  private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();
  // private final ClimberSubsystem climber = new ClimberSubsystem();

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * 1,
                                                                () -> driverXbox.getLeftX() * 1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveFieldOrientedAnglularVelocitySim = drivebase.driveFieldOriented(driveAngularVelocitySim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);


  public RobotContainer()
  { 
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("Hello World!"));

    // Register all named commands for auto and teleop
    NamedCommands.registerCommand("elevatorPos0", new ElevatorPositionCommand(elevator, 0));
    NamedCommands.registerCommand("elevatorPos1", new ElevatorPositionCommand(elevator, ElevatorConstants.POS_ONE));
    NamedCommands.registerCommand("elevatorPos2", new ElevatorPositionCommand(elevator, ElevatorConstants.POS_TWO));
    NamedCommands.registerCommand("elevatorPos3", new ElevatorPositionCommand(elevator, ElevatorConstants.POS_THREE));
    NamedCommands.registerCommand("elevatorPos4", new ElevatorPositionCommand(elevator, ElevatorConstants.POS_FOUR));

    NamedCommands.registerCommand("armPosUp", new ArmPositionCommand(arm, ArmConstants.POS_UP));
    NamedCommands.registerCommand("armPosMid", new ArmPositionCommand(arm, ArmConstants.POS_MID));
    NamedCommands.registerCommand("armPosDown", new ArmPositionCommand(arm, ArmConstants.POS_DOWN));

    NamedCommands.registerCommand("coralIntakeWithLimit", new CoralIntakeCommand(coralIntake).andThen(coralIntake.lock()));
    NamedCommands.registerCommand("coralIntakeForward", coralIntake.forward());
    NamedCommands.registerCommand("coralIntakeReverse", coralIntake.reverse());
    NamedCommands.registerCommand("coralIntakeLock", coralIntake.lock());

    NamedCommands.registerCommand("algaeIntakeForward", algaeIntake.forward());
    NamedCommands.registerCommand("algaeIntakeReverse", algaeIntake.reverse());
    NamedCommands.registerCommand("algaeIntakeLock", algaeIntake.lock());

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedAnglularVelocity :
                                driveFieldOrientedAnglularVelocitySim);

    // climber.setDefaultCommand(climber.freezeClimber());

    driverXbox.rightTrigger().onTrue(NamedCommands.getCommand("coralIntakeForward")).onFalse(NamedCommands.getCommand("coralIntakeLock"));
    driverXbox.rightBumper().onTrue(NamedCommands.getCommand("coralIntakeReverse")).onFalse(NamedCommands.getCommand("coralIntakeLock"));

    driverXbox.leftTrigger().onTrue(NamedCommands.getCommand("algaeIntakeForward")).onFalse(NamedCommands.getCommand("algaeIntakeLock"));
    driverXbox.leftBumper().onTrue(NamedCommands.getCommand("algaeIntakeReverse")).onFalse(NamedCommands.getCommand("algaeIntakeLock"));

    driverXbox.pov(0).onTrue(NamedCommands.getCommand("elevatorPos1"));
    driverXbox.pov(90).onTrue(NamedCommands.getCommand("elevatorPos2"));
    driverXbox.pov(180).onTrue(NamedCommands.getCommand("elevatorPos3"));
    driverXbox.pov(270).onTrue(NamedCommands.getCommand("elevatorPos4"));

    driverXbox.a().onTrue(NamedCommands.getCommand("armPosUp"));
    driverXbox.b().onTrue(NamedCommands.getCommand("armPosDown"));

    // driverXbox.a().onTrue(NamedCommands.getCommand("coralIntakeWithLimit"));

    driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("testing");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public ElevatorSubsystem getElevator(){
    return elevator;
  }

  // public ClimberSubsystem getClimber(){
  //   return climber;
  // }

  public ArmSubsystem getArm(){
    return arm;
  }

  public CoralIntakeSubsystem getCoralIntake(){
    return coralIntake;
  }

  public AlgaeIntakeSubsystem getAlgaeIntake(){
    return algaeIntake;
  }
}