// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
import static frc.robot.Constants.Swerve.TRACK_WIDTH;
import static frc.robot.Constants.Swerve.WHEEL_BASE;

import frc.robot.subsystems.*;

import java.sql.Driver;
import java.util.Optional;
import java.util.concurrent.locks.Condition;

/*
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
*/

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems:
  // private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); 
  private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_visionSubsystem);


  //set to color specific constants later on 
  private Pose2d m_speakerPose; 
  private Pose2d m_ampPose; 
  //private Pose2d m_sourcePose; 
  // private Pose2d m_stagePose; 
  private double m_directionInvert; 
  private double m_angleOffset; 


  private SendableChooser<Command> m_autoChooser; //for autons
  
  // Xbox Controllers (Replace with CommandPS4Controller or CommandJoystick if needed)
  private final CommandXboxController m_driverController =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
    setAllianceSpecific(); //sets constants to be either blue or red 
    configureAutoBuilder(); // Configure PathPlanner AutonBuilder 
    setAllianceSpecific();
    setDefaultCommands();  // Set/Bind the default commands for subsystems (i.e. commands that will run if the SS isn't actively running a command)
    configureDriverBindings();  // Configure driver game controller bindings and Triggers
    configureOperatorBindings();  //Configure operator game controller bindings and Triggers
    */
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */





  public void initializeTeleop()
  {
    
    // m_drivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
    //m_armSubsystem.goToAngle(Constants.ArmConstants.ARM_RESTING_POSITION_ANGLE);
    //m_armSubsystem.resetEncoderCommand(); 
  }


}


 

