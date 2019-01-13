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
import com.ctre.phoenix.motorcontrol.SensorCollection;
// setup for NavX
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;


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

  private AHRS mAhrs;       // this is the NavX control library

  //teleop variables
  private int count = 0;

  //autonomous variables
  private int m_autonomousCtr = 0;
  private double mag = 0;


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

    try {
        /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
        /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
        /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
        mAhrs = new AHRS(SPI.Port.kMXP); 
        mAhrs.zeroYaw();
    } catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

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
    mRoboDrive.arcadeDrive(mag, yaw, true);    // last param is whether to square the inputs - modifies response characteristics

    count++;
    if (count == 100)  {
      int lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
      int lPW = mLeft_Master.getSensorCollection().getPulseWidthPosition();
      int lQuadVel = mLeft_Master.getSensorCollection().getQuadratureVelocity();
      int lPWVel = mLeft_Master.getSensorCollection().getPulseWidthVelocity();
      System.out.println("_____________________________________________________________________");
      System.out.printf("teleopPeriodic:    lQuad: %6d   lPW: %6d   lQuadVel: %6d   lPWVel: %6d", lQuad, lPW, lQuadVel, lPWVel);
      System.out.println();
      DumpNavX();
    }

  }


  /**
   * Dump various values from the NavX
   */
  private void DumpNavX() {
          /* Display 6-axis Processed Angle Data                                      */
          System.out.printf(  "IMU_Connected: " +      mAhrs.isConnected());System.out.println();
          System.out.printf(  "IMU_IsCalibrating" +  mAhrs.isCalibrating());System.out.println();
          System.out.printf(   "IMU_Yaw:\t\t\t%5f",              mAhrs.getYaw());System.out.println();
          System.out.printf(   "IMU_Pitch:\t\t\t%5f",            mAhrs.getPitch());System.out.println();
          System.out.printf(   "IMU_Roll:\t\t\t%5f",             mAhrs.getRoll());System.out.println();
          
          /* Display tilt-corrected, Magnetometer-based heading (requires             */
          /* magnetometer calibration to be useful)                                   */  
          System.out.printf(   "IMU_CompassHeading:\t\t%5f",   mAhrs.getCompassHeading());System.out.println();
          
          /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
          System.out.printf(   "IMU_FusedHeading:\t\t%5f",     mAhrs.getFusedHeading());System.out.println();

          /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
          /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
          
          System.out.printf(   "IMU_TotalYaw:\t\t\t%5f",         mAhrs.getAngle());System.out.println();
          System.out.printf(   "IMU_YawRateDPS:\t\t\t%5f",       mAhrs.getRate());System.out.println();

          /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
          
          System.out.printf(   "IMU_Accel_X:\t\t\t%5f",          mAhrs.getWorldLinearAccelX());System.out.println();
          System.out.printf(   "IMU_Accel_Y:\t\t\t%5f",          mAhrs.getWorldLinearAccelY());System.out.println();
          System.out.printf(  "IMU_IsMoving:\t\t\t" +       mAhrs.isMoving());System.out.println();
          System.out.printf(  "IMU_IsRotating:\t\t\t" +     mAhrs.isRotating());System.out.println();

          /* Display estimates of velocity/displacement.  Note that these values are  */
          /* not expected to be accurate enough for estimating robot position on a    */
          /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
          /* of these errors due to single (velocity) integration and especially      */
          /* double (displacement) integration.                                       */
          
          System.out.printf(   "Velocity_X:\t\t\t%5f",           mAhrs.getVelocityX());System.out.println();
          System.out.printf(   "Velocity_Y:\t\t\t%5f",           mAhrs.getVelocityY());System.out.println();
          System.out.printf(   "Displacement_X:\t\t\t%5f",       mAhrs.getDisplacementX());System.out.println();
          System.out.printf(   "Displacement_Y:\t\t\t%5f",       mAhrs.getDisplacementY());System.out.println();

  }


  private boolean start = false;
  private int iterationsCounter = 0;
  private double forwardBackDrive(int iterations, int startPoint, int endPoint)
  {
    /**
     *  makes the robot go forward and then backward a certian amount of iterations
     * NOTE: this function is specific for Gussetts and is specific to his drive train
     *  - this is because as Gussetts goes forward, its Quadtrature counter goes negative
     *  - Also the number 5216 is specific to Gussetts' wheel size
     *  - if you would like to reuse this code for a different robot you will need to change these settings
     *      * Quadtrature number per foot
     *      * drive train specifics
     */
    int lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
    if(start == false && lQuad >= startPoint)
    {
      start = true;
      mag = .5;
      iterationsCounter++;
    }
    if(start == true && lQuad <= endPoint)
    {
      mag = -.5;
      start = false;
    }
    if(iterationsCounter > iterations)
    {
      mag = 0;
    }
    return mag;
    
  }



  private double correctYaw(double correctAngle)
  {
    /**
     * fixing the yaw to be consistantly straight
     * use in conjuction with forwardDrive
     * takes correct angle and keeps the robots angle within 0.4 degrees of that angle
     */
    double driveYaw;
    double navYaw = mAhrs.getYaw();
    navYaw = navYaw - correctAngle;
    if(navYaw < 2 && navYaw > -2)
    {
      driveYaw = (-navYaw / 180) * 15;
    }
    else if(navYaw < 18 && navYaw > -18)
    {
      driveYaw = (-navYaw / 180) * 10;
    }
    else
    {
      driveYaw = (-navYaw / 180);
    }
    return driveYaw;
  }


  private double abs(double num)
  {
    if(num < 0)
    {
      num = num * -1;
    }
    return num;
  }


  private int abs(int num)
  {
    if(num < 0)
    {
      num = num * -1;
    }
    return num;
  }


  private void forwardDrive(int distance, double speed)
  {
    /**
     * distance is in feet
     * NOTE: this function is specific for Gussetts and is specific to his drive train
     *  - this is because as Gussetts goes forward, its Quadtrature counter goes negative
     *  - Also the number 5216 is specific to Gussetts' wheel size
     *  - if you would like to reuse this code for a different robot you will need to change these settings
     *      * Quadtrature number per foot
     *      * drive train specifics
     */
    int lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
    int lQuadVel = mLeft_Master.getSensorCollection().getQuadratureVelocity();
    System.out.printf("autonomousPeriodic:    lQuad: %6d   lQuadVel: %6d   ", lQuad, lQuadVel);
    System.out.println();

    int totalDistance = distance;
    mag = speed;
    
    double initialYaw = mAhrs.getYaw();
    int initialPosition = mLeft_Master.getSensorCollection().getQuadraturePosition();
    
    int i = 0;
    boolean straight = true;
    if(lQuad < totalDistance + initialPosition)
    {
      while(lQuad > totalDistance + initialPosition && straight == true)
      {
        lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
        lQuadVel = mLeft_Master.getSensorCollection().getQuadratureVelocity();
        double driveYaw = correctYaw(initialYaw);
        double navYaw = mAhrs.getYaw();
        if(i % 600 == 0)
        {
          System.out.printf("autonomousPeriodic:    lQuad: %6d   lQuadVel: %6d   ", lQuad, lQuadVel);
          System.out.println();
          System.out.printf("driveYaw : %5f   navYaw : %5f", driveYaw, navYaw);
          System.out.println();
        }
  
        // if(abs(navYaw - initialYaw) > 5)
        // {
        //   System.out.println("ERROR: SOMETHING NOT RIGHT, CHECK IF BROKEN!");
        //   straight = false;
        // }
        
        mRoboDrive.arcadeDrive(-mag, driveYaw, true); 
        i++;
      }
    }
    else if(lQuad > totalDistance + initialPosition)
    {
      while(lQuad < totalDistance + initialPosition && straight == true)
      {
        lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
        lQuadVel = mLeft_Master.getSensorCollection().getQuadratureVelocity();
        double driveYaw = correctYaw(initialYaw);
        double navYaw = mAhrs.getYaw();
        if(i % 600 == 0)
        {
          System.out.printf("autonomousPeriodic:    lQuad: %6d   lQuadVel: %6d   ", lQuad, lQuadVel);
          System.out.println();
          System.out.printf("driveYaw : %5f   navYaw : %5f", driveYaw, navYaw);
          System.out.println();
        }
  
        // if(abs(navYaw - initialYaw) > 5)
        // {
        //   System.out.println("ERROR: SOMETHING NOT RIGHT, CHECK IF BROKEN!");
        //   straight = false;
        // }
        
        mRoboDrive.arcadeDrive(-mag, driveYaw, true); 
        i++;
      }
    }
  }

  private void turnDrive(double turnAngle)
  {
    /**
     * turnAngle is from 180 to -180
     * negative is right
     * positive is left
     * NOTE: these specifics are in particular to Gussetts' drive train and 
     *      motor configuration
     * - if you want to reuse this code you should figure out which direction it turns when the 
     *      turn angle is positive and negative
     */

    mag = 0;
    double initialNavYaw = mAhrs.getYaw();
    double endYaw = initialNavYaw + turnAngle;

    double navYaw = mAhrs.getYaw();
    System.out.println();
    System.out.printf("initial NavYaw : %5f   endYaw : %5f   navYaw :   %5f", initialNavYaw, endYaw, navYaw);
    System.out.println();

    int i = 0;
    double driveYaw = 0;
    double difference = (endYaw - navYaw);
    double initialDiff = difference;
    
    System.out.printf("driveYaw : %5f   navYaw : %5f    difference : %5f", driveYaw, navYaw, difference);
    System.out.println();

    while((int)(difference) != 0)    {

      navYaw = mAhrs.getYaw();
      difference = (endYaw - navYaw);
      if((int)(difference) > 0)
      {
        driveYaw = 1;
      }
      else if((int)(difference) < 0)
      {
        driveYaw = -1;
      }
      
      
      if(abs(difference) < abs((initialDiff / 6)))
      {
        driveYaw = driveYaw * .3;
      }
      else
      {
        driveYaw = driveYaw * .5;
      }

      
      if(i % 600 == 0)
      {
        System.out.printf("driveYaw : %5f   navYaw : %5f    difference : %5f", driveYaw, navYaw, difference);
        System.out.println();
      }

      i++;

      mRoboDrive.arcadeDrive(-mag, driveYaw, true);
    }
    System.out.printf("driveYaw : %5f   navYaw : %5f    difference : %5f", driveYaw, navYaw, difference);
    System.out.println();  
  }


  private void sleep(int amt)
  {
    long a = System.currentTimeMillis();
    long b = System.currentTimeMillis();
    while ((b - a) <= amt)
    {
      b = System.currentTimeMillis();
    }
  }


  private void driveAndCheck(int x, int y, double speed, double turnAngle, double endNavX)
  {
    int initialLQuad = abs(mLeft_Master.getSensorCollection().getQuadraturePosition());
    int initialRQuad = abs(mRight_Master.getSensorCollection().getQuadraturePosition());
    double initialNavX = mAhrs.getYaw();
    
    int lQuad = abs(mLeft_Master.getSensorCollection().getQuadraturePosition());
    int rQuad = abs(mRight_Master.getSensorCollection().getQuadraturePosition());

    int lTraveled = abs(lQuad - initialLQuad);
    int rTraveled = abs(rQuad - initialRQuad);
    int aTraveled = (int)((lTraveled + rTraveled) / 2);

    int xTraveled = (int)(abs(aTraveled * java.lang.Math.cos(turnAngle)));
    int yTraveled = (int)(abs(aTraveled * java.lang.Math.sin(turnAngle)));

    while(xTraveled < x && yTraveled < y)
    {
        //x and y has to be within a certian distance from 
        lQuad = abs(mLeft_Master.getSensorCollection().getQuadraturePosition());
        rQuad = abs(mRight_Master.getSensorCollection().getQuadraturePosition());

        lTraveled = abs(lQuad - initialLQuad);
        rTraveled = abs(rQuad - initialRQuad);
        aTraveled = (int)((lTraveled + rTraveled) / 2);
    
        xTraveled = (int)(abs(aTraveled * java.lang.Math.cos(turnAngle)));
        yTraveled = (int)(abs(aTraveled * java.lang.Math.sin(turnAngle)));

        double navX = mAhrs.getYaw();

        double driveYaw = 0;
        if(abs(navX - endNavX) != endNavX)
        {
          driveYaw = -navX / 180;
        }

        if(xTraveled + 1000 > x)
        {
          speed = .3;
        } 
        else if(xTraveled + 2000 > x)
        {
          speed = speed * .5;
        }

        mRoboDrive.arcadeDrive(speed, driveYaw);

    }

  }


  private void xyDrive(double xFeet, double yFeet, double speed)
  {
    final int quadPerFoot = -(5216 / 2);// change this for different robots
    int newX = (int)(xFeet * quadPerFoot);
    int newY = (int)(yFeet * quadPerFoot);
    
    double endNavX = mAhrs.getYaw();

    double turnAngle = java.lang.Math.atan(newX / newY);

    turnDrive(turnAngle);

    driveAndCheck(newX, newY, speed, turnAngle, endNavX);


  }



  private void myDrive()
  {
    mLeft_Master.getSensorCollection().setQuadraturePosition(0, 10);
    mRight_Master.getSensorCollection().setQuadraturePosition(0, 10);
    mAhrs.zeroYaw();

    sleep(1000);

    xyDrive(3, 4, .5);

    final int quadPerFoot = -(5216 / 2);// change this for different robots
    int x = 2 * quadPerFoot;
    double speed = .5;
    forwardDrive(x, speed);
    sleep(500);
    forwardDrive(-x, speed);
  }


  @Override
  public void autonomousPeriodic()
  {
    if(m_autonomousCtr == 0) 
    {
      myDrive();
    }

    //outputs all the data every second
    if (m_autonomousCtr % 50 == 0)  
    {
      int lQuad = mLeft_Master.getSensorCollection().getQuadraturePosition();
      int lQuadVel = mLeft_Master.getSensorCollection().getQuadratureVelocity();
      double navYaw = mAhrs.getYaw();
        
      System.out.printf("autonomousPeriodic:    lQuad: %6d   lQuadVel: %6d   ", lQuad, lQuadVel);
      System.out.println();

      double distance = (lQuad / 4096) * 6 * 3.1415;// one full spin of the Gussetts wheel is 4096 Quadtrature units
      System.out.println("Distance Traveled is : " + distance);
      System.out.printf("IMU_Yaw:\t\t\t%5f"   , navYaw);
      System.out.println("\n");
      //DumpNavX();

    }

    //increments the time holder by .02sec
    m_autonomousCtr++;
  }

}

