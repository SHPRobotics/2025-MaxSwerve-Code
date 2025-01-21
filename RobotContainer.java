// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // Coral Intake subsystem
  private final CoralSubsystem m_coral = new CoralSubsystem();
  // Algae Intake subsystem
  private final AlgaeSubsystem m_algae = new AlgaeSubsystem();


  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  //XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    m_coral.setDefaultCommand(
                new RunCommand(
                    () -> m_coral.CoralStop(),
                    m_coral));
    m_algae.setDefaultCommand(
                new RunCommand(
                () -> m_algae.AlgaeStop(),
                        m_algae));
    
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
 
// -------------------------------- CHASSIS --------------------------------------------
// using CommandXboxController 
 m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

 // press left bumper for strafe true left
    m_driverController.leftBumper()
        .whileTrue(new RunCommand(
        ()-> m_robotDrive.drive(0, 0.5, 0, false), 
        m_robotDrive));

    // press right bumper for strafe true right
    m_driverController.rightBumper()
        .whileTrue(new RunCommand(
        ()-> m_robotDrive.drive(0, -0.5, 0, false), 
        m_robotDrive));

    

    /*new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

// press left bumper to strafe left
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0,.5,0,false),
            m_robotDrive));

// press right bumper to strafe right
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.drive(0,-.5,0,false),
            m_robotDrive));
*/
// -------------------------------- CORAL -----------------------------------------------
// using CommandXboxController
  m_operatorController.x()
       .whileTrue(new RunCommand(
            () -> m_coral.FeedCoralIn(),
            m_coral));

m_operatorController.b()
      .whileTrue(new RunCommand(
            () -> m_coral.ShootCoralOut(),
            m_coral));
m_operatorController.x()
    .whileTrue(new RunCommand(() -> {}, m_coral));
/*// press x to feed coral in
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_coral.FeedCoralIn(),
            m_coral));

// press b to shoot coral out
    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(
            () -> m_coral.ShootCoralOut(),
            m_coral));
    
*/
// -------------------------------- ALGAE -----------------------------------------------
// using CommandXboxController
  m_operatorController.y()
       .whileTrue(new RunCommand(
            () -> m_algae.FeedAlgaeIn(),
            m_algae));

m_operatorController.a()
       .whileTrue(new RunCommand(
            () -> m_algae.ShootAlgaeOut(),
            m_algae));
 
/*// press y to feed algae in
    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(
            () -> m_algae.FeedAlgaeIn(),
            m_algae));

// press a to shoot algae out
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(
            () -> m_algae.ShootAlgaeOut(),
            m_algae));
    
*/

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
