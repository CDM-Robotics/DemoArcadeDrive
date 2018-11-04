/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.       
/* mod to check git    mod 2 and now mod 3                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 * 
 * DE branch  18:43
 */
public class Robot extends IterativeRobot {

  private DifferentialDrive mRoboDrive;
  private Joystick m_leftStick;

  private WPI_TalonSRX mLeft_Master;
  private WPI_TalonSRX mLeft_Slave0;
  private WPI_TalonSRX mRight_Master;
  private WPI_TalonSRX mRight_Slave0;

  ArrayList<TalonSRX> mMasterTalons = new ArrayList<TalonSRX>();

  private int m_teleopCtr = 0;


  @Override
  public void robotInit() {

    m_leftStick = new Joystick(0);

    mLeft_Master = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_MASTER);
    mLeft_Master.setSafetyEnabled(false);
    mLeft_Master.configOpenloopRamp(0.1 , 10);
    mLeft_Master.setNeutralMode(NeutralMode.Brake);
    mLeft_Master.setSensorPhase(true);
    mLeft_Master.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);

    mLeft_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_SLAVE0);
    mLeft_Slave0.set(ControlMode.Follower, RobotConfig.DRIVE_LEFT_MASTER);
    mLeft_Slave0.setInverted(false);

    mRight_Master = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_MASTER);
    mRight_Master.setSafetyEnabled(false);
    mRight_Master.configOpenloopRamp(0.1, 10);
    mRight_Master.setNeutralMode(NeutralMode.Brake);
    mRight_Master.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);

    mRight_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_SLAVE0);
    mRight_Slave0.set(ControlMode.Follower, RobotConfig.DRIVE_RIGHT_MASTER);
    mRight_Slave0.setInverted(false);

    mMasterTalons.add(mRight_Master);
    mMasterTalons.add(mLeft_Master);

    mRoboDrive = new DifferentialDrive(mLeft_Master, mRight_Master);

  }



  /**
   * See here for an explanation of tank drive vs arcade drive:
   *    https://wpilib.screenstepslive.com/s/currentCS/m/java/l/914148-driving-a-robot-using-differential-drive
   * Tank Drive - This mode uses one value each to control the individual sides of the drivetrain.
   *    Each side of the robot is controlled independently with two sticks
   * Arcade Drive - This mode uses one value to control the throttle (speed along the X-axis)
   *    of the drivetrain and one for the rate of rotation - single stick
   * 
   * Go here to see doco on DifferentialDrive class and arcadeDrive() method
   *    http://first.wpi.edu/FRC/roborio/release/docs/java/
   */
  @Override
  public void teleopPeriodic() {
    //mRoboDrive.tankDrive(m_leftStick.getY(), m_rightStick.getY());

    double mag, yaw;
    mag = m_leftStick.getY();                   // how fast
    yaw = m_leftStick.getX();                   // turn left or right
    yaw = yaw * 0.8;                            // reduce sensitivity on turn
    mRoboDrive.arcadeDrive(-mag, yaw, true);    // last param is whether to square the inputs - modifies response characteristics

    m_teleopCtr++;
    if (m_teleopCtr % 50 == 0)  {
      int lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
      int lPW = mLeft_Master.getSensorCollection().getPulseWidthPosition();
      int lQuadVel = mLeft_Master.getSensorCollection().getQuadratureVelocity();
      int lPWVel = mLeft_Master.getSensorCollection().getPulseWidthVelocity();
      System.out.printf("teleopPeriodic:    lQuad: %6d   lPW: %6d   lQuadVel: %6d   lPWVel: %6d", lQuad, lPW, lQuadVel, lPWVel);
    }

  }


}
