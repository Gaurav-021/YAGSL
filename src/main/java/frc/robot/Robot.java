// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.sensors.Pigeon2;
// https://github.com/Mechanical-Advantage/AdvantageKit/releases/download/v2.2.4/AdvantageKit.json
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.NodeSelector;

import java.io.File;
import java.io.IOException;

import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{
  XboxController driver_Controller = new XboxController(0);
  XboxController operator_controller = new XboxController(1);

  boolean operator_controller_A_button;
  boolean operator_controller_Y_button;
  boolean operator_controller_X_button;
  boolean operator_controller_B_button;
  
  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  Vision m_vision = new Vision();
  Arm m_arm = new Arm();
  NodeSelector m_nodeSelector = new NodeSelector();

  boolean[] operator_buttons = {false, false, false, false, false, false, false, false};
  double[] operator_triggers = new double[2];

  public Pigeon2 gyro;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    PathPlannerServer.startServer(5811);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();

    gyro = new Pigeon2(16);
    gyro.configFactoryDefault();
    gyro.setYaw(0);

    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (m_vision.getEstimatedRobotPose() != null){
      m_robotContainer.addVisionMeasurement(m_vision.getEstimatedRobotPose());
      SmartDashboard.putNumber("Vision Pose X", m_vision.getEstimatedRobotPose().getX());
      SmartDashboard.putNumber("Vision Pose Y", m_vision.getEstimatedRobotPose().getY());
    }
    
    m_vision.gamePiecePeriodic();
    m_vision.aprilTagPeriodic();
    m_vision.reflectiveTapePeriodic();
    m_nodeSelector.updateSelectedNode(driver_Controller.getPOV());

    SmartDashboard.putNumber("Main Arm Position", m_arm.get_main_arm_position());
    SmartDashboard.putNumber("Intake Arm Position", m_arm.get_intake_arm_position_selected());
    SmartDashboard.putNumber("Main Arm Position Throughbore", m_arm.get_main_arm_position_throughbore());
    SmartDashboard.putNumber("Swerve Pose X", m_robotContainer.getPose().getX());
    SmartDashboard.putNumber("Swerve Pose Y", m_robotContainer.getPose().getY());
    SmartDashboard.putString("node", m_nodeSelector.getCurrentNode());

    getControllerStates();

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.Drivebase.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
    
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setDriveMode();
    m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    // m_arm.armPeriodic(operator_buttons, operator_triggers, operator_controller.getLeftY(), operator_controller.getRightY());

    if (driver_Controller.getPOV() == 0){
      m_robotContainer.drive(new Translation2d(0, 2), 0, false);
    }
    else if (driver_Controller.getPOV() == 90){
      m_robotContainer.drive(new Translation2d(2, 0), 0, false);

    }
    else if (driver_Controller.getPOV() == 270){
      m_robotContainer.drive(new Translation2d(-2, 0), 0, false);

    }
    else if (driver_Controller.getPOV() == 180){
      m_robotContainer.drive(new Translation2d(0, -2), 0, false);

    }
    else if (driver_Controller.getRawButton(6)) {
      m_robotContainer.drive(new Translation2d(0, 0), -m_vision.getGamePieceYaw()/10, false);
      // m_robotContainer.updateSwerveParameters(new Translation2d(0, 0), -m_vision.getGamePieceYaw() * (0.1 / Math.sqrt(Math.abs(m_vision.getGamePieceYaw()))), false);
    }
    else if (driver_Controller.getRawButton(5)) {
      Constants.speedScale = 0.5;
    }
    else {
      Constants.speedScale = 1.0;
    }

  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    try
    {
      new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
    } catch (IOException e)
    {
      throw new RuntimeException(e);
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }


  public void getControllerStates() {
    operator_controller_A_button = operator_controller.getAButton();
    operator_controller_B_button = operator_controller.getBButton();
    operator_controller_X_button = operator_controller.getXButton();
    operator_controller_Y_button = operator_controller.getYButton();
    operator_buttons[0] = operator_controller_X_button;
    operator_buttons[1] = operator_controller_Y_button;
    operator_buttons[2] = operator_controller_A_button;
    operator_buttons[3] = operator_controller_B_button;
    operator_buttons[4] = operator_controller.getStartButton();
    operator_buttons[5] = operator_controller.getBackButton();
    operator_buttons[6] = operator_controller.getLeftBumperReleased();
    operator_buttons[7] = operator_controller.getRightBumperReleased();
    operator_triggers[0] = operator_controller.getLeftTriggerAxis();
    operator_triggers[1] = operator_controller.getRightTriggerAxis();

  }
}
