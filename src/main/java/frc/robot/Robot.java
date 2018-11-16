/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.       
/* mod to check git    mod 2 and now mod 3                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation; 
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;


// setup for NavX 

import com.kauailabs.navx.frc.AHRS; 
import edu.wpi.first.wpilibj.SPI; 
import edu.wpi.first.wpilibj.I2C; 
import edu.wpi.first.wpilibj.SerialPort; 

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 * Branch ChristopherRP
 * ~~Kahn~~
 * My Name is Kahn!(emphasis on all words)
 * ~~Commander Spock~~
 * Kaaaaaaahhhhhhhhnnnnnnnn!!?!(anger in voice)
 */
public class Robot extends IterativeRobot {

  private DifferentialDrive mRoboDrive;
  private Joystick m_leftStick;

  private WPI_TalonSRX mLeft_Master;
  private WPI_TalonSRX mLeft_Slave0;
  private WPI_TalonSRX mRight_Master;
  private WPI_TalonSRX mRight_Slave0;

  ArrayList<TalonSRX> mMasterTalons = new ArrayList<TalonSRX>();

  private AHRS mAhrs;       // this is the NavX control library 

  private int m_MilliCount = 0;        //second counter for the debug

  private int m_PeriodicCount = 0;

  private int m_teleopCtr = 0;
 
  private int m_lPW = 0;

  private double mag = 0;

 

  int lPW;

  @Override
  public void robotInit() {

    m_leftStick = new Joystick(0);

    mLeft_Master = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_MASTER);
    mLeft_Master.setSafetyEnabled(false);
    mLeft_Master.configOpenloopRamp(0.1 , 10);
    mLeft_Master.setNeutralMode(NeutralMode.Brake);
    mLeft_Master.setSensorPhase(true);
    mLeft_Master.getSensorCollection().setQuadraturePosition(0, 10);

    mLeft_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_LEFT_SLAVE0);
    mLeft_Slave0.set(ControlMode.Follower, RobotConfig.DRIVE_LEFT_MASTER);
    mLeft_Slave0.setInverted(false);

    mRight_Master = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_MASTER);
    mRight_Master.setSafetyEnabled(false);
    mRight_Master.configOpenloopRamp(0.1, 10);
    mRight_Master.setNeutralMode(NeutralMode.Brake);

    mRight_Slave0 = new WPI_TalonSRX(RobotConfig.DRIVE_RIGHT_SLAVE0);
    mRight_Slave0.set(ControlMode.Follower, RobotConfig.DRIVE_RIGHT_MASTER);
    mRight_Slave0.setInverted(false);

    mMasterTalons.add(mRight_Master);
    mMasterTalons.add(mLeft_Master);

    mRoboDrive = new DifferentialDrive(mLeft_Master, mRight_Master);

    try { 
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */ 
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */ 
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */ 
      mAhrs = new AHRS(SPI.Port.kMXP);  

      mAhrs.zeroYaw(); 

  } catch (RuntimeException ex ) { 

      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true); 

  } 

    System.out.println("**************************************");
    System.out.println("      CHRIS    CODE                   ");
    System.out.println("**************************************");


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
    
    m_MilliCount = m_MilliCount + 1;   
    m_PeriodicCount = m_PeriodicCount + 1;                                      // adds 1 every 20 miliseconds 

    if (m_MilliCount == 100){                                             // 100 groups of 20 miliseconds is 20 seconds
      System.out.println("====================================");
      System.out.println("Debug--Complete for the " + m_PeriodicCount + " time");
      System.out.println("====================================");
      m_MilliCount = 0;
    }

    m_teleopCtr++;
    if (m_teleopCtr % 50 == 0)  {

      int lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();

      lPW = mLeft_Master.getSensorCollection().getPulseWidthPosition();// ticks

      int lQuadVel = mLeft_Master.getSensorCollection().getQuadratureVelocity();

      int lPWVel = mLeft_Master.getSensorCollection().getPulseWidthVelocity();

      System.out.printf("teleopPeriodic:    lQuad: %6d   lPW: %6d   lQuadVel: %6d   lPWVel: %6d\n", lQuad, lPW, lQuadVel, lPWVel);
      
      m_lPW = lPW;

      DumpNavX();
    }

    /** 
   * Dump various values from the NavX 
   */ 
  
    
  // 0.0046 inches per tick(rounded from 0.00460205 inches)
    

      double yaw;

      mag = m_leftStick.getY();                   // how fast

      yaw = m_leftStick.getX();                   // turn left or right

      yaw = yaw * 0.8;                            // reduce sensitivity on turn

      mRoboDrive.arcadeDrive(-mag, yaw, true);  // last param is whether to square the inputs - modifies response characteristics

  }


  @Override
  public void autonomousPeriodic() {

    // Test program to drive 2 feet
    int lPW = mLeft_Master.getSensorCollection().getPulseWidthPosition();// ticks
    m_teleopCtr++;
    if (m_teleopCtr % 50 == 0) {
      System.out.println(lPW);
    }

    if (lPW >= 5216) {// checks distance
      mag = 0;
      
    } else {
      mag = 0.5;
    }
    mRoboDrive.arcadeDrive(-mag, 0, true);

  }



private void DumpNavX() { 
 
  /* Display 6-axis Processed Angle Data                                      */ 

  // System.out.printf(  "IMU_Connected: " +      mAhrs.isConnected()); 
  // System.out.printf(  "IMU_IsCalibrating" +  mAhrs.isCalibrating()); 

  System.out.printf(   "IMU_Yaw:\t\t\t%5f \n",              mAhrs.getYaw()); 
  
  // System.out.printf(   "IMU_Pitch:\t\t\t%5f\n",            mAhrs.getPitch()); 
  // System.out.printf(   "IMU_Roll:\t\t\t%5f\n",             mAhrs.getRoll()); 

  /* Display tilt-corrected, Magnetometer-based heading (requires             */ 
  /* magnetometer calibration to be useful)                                   */   

  System.out.printf(   "IMU_CompassHeading:\t\t%5f\n",   mAhrs.getCompassHeading()); 

  /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */ 

 // System.out.printf(   "IMU_FusedHeading:\t\t%5f\n",     mAhrs.getFusedHeading()); 

  /* These functions are compatible w/the WPI Gyro Class, providing a simple  */ 
  /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */ 

  // System.out.printf(   "IMU_TotalYaw:\t\t\t%5f\n",         mAhrs.getAngle()); 
  // System.out.printf(   "IMU_YawRateDPS:\t\t\t%5f\n",       mAhrs.getRate()); 

  /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */ 

  // System.out.printf(   "IMU_Accel_X:\t\t\t%5f\n",          mAhrs.getWorldLinearAccelX()); 
  // System.out.printf(   "IMU_Accel_Y:\t\t\t%5f\n",          mAhrs.getWorldLinearAccelY()); 
  // System.out.printf(  "IMU_IsMoving:\t\t\t" +       mAhrs.isMoving()); 
  // System.out.printf(  "IMU_IsRotating:\t\t\t" +     mAhrs.isRotating()); 

  /* Display estimates of velocity/displacement.  Note that these values are  */ 
  /* not expected to be accurate enough for estimating robot position on a    */ 
  /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */ 
  /* of these errors due to single (velocity) integration and especially      */ 
  /* double (displacement) integration.   */ 
  // System.out.printf(   "Velocity_X:\t\t\t%5f\n",           mAhrs.getVelocityX()); 
  // System.out.printf(   "Velocity_Y:\t\t\t%5f\n",           mAhrs.getVelocityY()); 
  // System.out.printf(   "Displacement_X:\t\t\t%5f\n",       mAhrs.getDisplacementX()); 
  // System.out.printf(   "Displacement_Y:\t\t\t%5f\n",       mAhrs.getDisplacementY()); 

  } 
}
