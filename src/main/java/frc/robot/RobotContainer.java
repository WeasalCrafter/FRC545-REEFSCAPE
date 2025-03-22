// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.sensors.laserCan;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer
{
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser;

  // Subsystem delclarations
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/545"));                                                                                
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final CoralIntakeSubsystem coralIntake = new CoralIntakeSubsystem();
  private final laserCan laserCan = new laserCan();

  //private final AlgaeIntakeSubsystem algaeIntake = new AlgaeIntakeSubsystem();

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
                                                                () -> -driverXbox.getLeftY(),
                                                                () -> -driverXbox.getLeftX())
                                                            .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                            // .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

    SwerveInputStream driveRobotRelativeAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> -driverXbox.getLeftY(),
                                                            () -> -driverXbox.getLeftX())
                                                          .withControllerRotationAxis(() -> -driverXbox.getRightX())
                                                          // .withControllerRotationAxis(driverXbox::getRightX)
                                                          .deadband(OperatorConstants.DEADBAND)
                                                          .scaleTranslation(0.2)
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
                                                                   () -> driverXbox.getLeftY(),
                                                                   () -> driverXbox.getLeftX())
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

  SequentialCommandGroup positionOne = new SequentialCommandGroup(
    // new ArmPositionCommand(arm, ArmConstants.POS_UP)
    new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_ONE)
  );
  SequentialCommandGroup positionTwo = new SequentialCommandGroup(
    new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_TWO)
    //new ArmPositionCommand(arm, ArmConstants.POS_MID)
  );
  SequentialCommandGroup positionThree = new SequentialCommandGroup(
    new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_THREE)
    //new ArmPositionCommand(arm, ArmConstants.POS_DOWN)
  );
  SequentialCommandGroup positionFour = new SequentialCommandGroup(
    new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_FOUR)
    //new ArmPositionCommand(arm, ArmConstants.POS_DOWN)
  );

  SequentialCommandGroup fullIntake = new SequentialCommandGroup(
    new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_ONE),
    new CoralIntakeCommand(coralIntake, this).andThen(coralIntake.lock())
  );

  SequentialCommandGroup fullOuttake = new SequentialCommandGroup(
    new CoralOuttakeCommand(coralIntake, this).andThen(coralIntake.lock()),
    new WaitCommand(0.5),
    new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_ONE)
  );

  public RobotContainer()
  { 
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("Hello World!"));
    NamedCommands.registerCommand("distance", Commands.print("distance: "+getLaserCan().getDistance()));

    // Register all named commands for auto and teleop
    NamedCommands.registerCommand("elevatorPos1", new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_ONE));
    NamedCommands.registerCommand("elevatorPos2", new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_TWO));
    NamedCommands.registerCommand("elevatorPos3", new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_THREE));
    NamedCommands.registerCommand("elevatorPos4", new ElevatorPositionCommand(elevator, this, ElevatorConstants.POS_FOUR));

    NamedCommands.registerCommand("armPosUp", new ArmPositionCommand(arm, ArmConstants.POS_UP));
    NamedCommands.registerCommand("armPosDown", new ArmPositionCommand(arm, ArmConstants.POS_DOWN));

    NamedCommands.registerCommand("coralOuttakeWithLimit", new CoralOuttakeCommand(coralIntake, this).andThen(coralIntake.lock()));
    NamedCommands.registerCommand("coralIntakeWithLimit", new CoralIntakeCommand(coralIntake, this).andThen(coralIntake.lock()));
    NamedCommands.registerCommand("coralIntakeForward", coralIntake.forward());
    NamedCommands.registerCommand("coralIntakeReverse", coralIntake.reverse());
    NamedCommands.registerCommand("coralIntakeLock", coralIntake.lock());

    //NamedCommands.registerCommand("algaeIntakeForward", algaeIntake.forward());
    //NamedCommands.registerCommand("algaeIntakeReverse", algaeIntake.reverse());
    //NamedCommands.registerCommand("algaeIntakeLock", algaeIntake.lock());

    // Configure the trigger bindings and auto chooser
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureDriverAndOperator();
    // configureBindingsTest();
  }

  private void configureBindingsTest()
  {
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
      driveFieldOrientedAnglularVelocity :
      driveFieldOrientedAnglularVelocitySim);

    driverXbox.leftTrigger().onTrue(NamedCommands.getCommand("algaeIntakeForward")).onFalse(NamedCommands.getCommand("algaeIntakeLock"));
    driverXbox.leftBumper().onTrue(NamedCommands.getCommand("algaeIntakeReverse")).onFalse(NamedCommands.getCommand("algaeIntakeLock"));
    driverXbox.rightTrigger().onTrue(NamedCommands.getCommand("coralIntakeForward")).onFalse(NamedCommands.getCommand("coralIntakeLock"));
    driverXbox.rightBumper().onTrue(NamedCommands.getCommand("coralIntakeReverse")).onFalse(NamedCommands.getCommand("coralIntakeLock"));

    driverXbox.pov(0).onTrue(positionOne);
    driverXbox.pov(90).onTrue(positionTwo);
    driverXbox.pov(180).onTrue(positionThree);
    driverXbox.pov(270).onTrue(positionFour);

    driverXbox.a().and(driverXbox.leftBumper()).onTrue(drivebase.autoAlignReef(false));
    driverXbox.a().and(driverXbox.rightBumper()).onTrue(drivebase.autoAlignReef(true));
    driverXbox.b().onTrue(NamedCommands.getCommand("coralOuttakeWithLimit")); 
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));
  }

  private void configureDriverAndOperator(){

    // ---------------------------------------------------------------------------------------------------------------------------
    // ----------------------------------------------------- Driver Controls -----------------------------------------------------
    // ---------------------------------------------------------------------------------------------------------------------------

    driverXbox.a().and(driverXbox.leftBumper()).onTrue(drivebase.autoAlignReef(false));
    driverXbox.a().and(driverXbox.rightBumper()).onTrue(drivebase.autoAlignReef(true));
    driverXbox.b().onTrue(Commands.none());
    driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
      driveFieldOrientedAnglularVelocity :
      driveFieldOrientedAnglularVelocitySim);

    // ---------------------------------------------------------------------------------------------------------------------------
    // ---------------------------------------------------- Operator Controls ----------------------------------------------------
    // ---------------------------------------------------------------------------------------------------------------------------
    
    operatorXbox.a().onTrue(fullIntake);
    operatorXbox.b().onTrue(fullOuttake);
    // operatorXbox.a().onTrue(NamedCommands.getCommand("coralIntakeWithLimit")); 
    // operatorXbox.b().onTrue(NamedCommands.getCommand("coralOuttakeWithLimit")); 
    // operatorXbox.x().onTrue(NamedCommands.getCommand("armPosDown"));
    // operatorXbox.y().onTrue(NamedCommands.getCommand("armPosUp"));
    operatorXbox.y().onTrue(NamedCommands.getCommand("distance"));


    operatorXbox.pov(0).onTrue(positionOne);
    operatorXbox.pov(90).onTrue(positionTwo);
    operatorXbox.pov(180).onTrue(positionThree);
    operatorXbox.pov(270).onTrue(positionFour);

    //operatorXbox.leftTrigger().onTrue(NamedCommands.getCommand("algaeIntakeForward")).onFalse(NamedCommands.getCommand("algaeIntakeLock"));
    //operatorXbox.leftBumper().onTrue(NamedCommands.getCommand("algaeIntakeReverse")).onFalse(NamedCommands.getCommand("algaeIntakeLock"));
    operatorXbox.rightBumper().onTrue(NamedCommands.getCommand("coralIntakeReverse")).onFalse(NamedCommands.getCommand("coralIntakeLock"));
    operatorXbox.rightTrigger().onTrue(NamedCommands.getCommand("coralIntakeForward")).onFalse(NamedCommands.getCommand("coralIntakeLock"));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // return drivebase.driveToDistanceCommand(2,2);
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("test");
    return autoChooser.getSelected();
  }

  public laserCan getLaserCan(){
    return laserCan;
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}