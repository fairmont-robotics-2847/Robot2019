/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.*;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.vision.VisionRunner;
import edu.wpi.first.wpilibj.vision.VisionThread;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.GripPipeline;


import java.util.ArrayList;

public class Robot extends IterativeRobot {
	// Driver control
	Joystick _joy = new Joystick(0);

	// Drive
	WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(0);
	WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(1);
	WPI_VictorSPX _leftSlave1 = new WPI_VictorSPX(0);
	WPI_VictorSPX _rightSlave1 = new WPI_VictorSPX(1);

	// Control ball intake/expel
	WPI_VictorSPX _ballIntake = new WPI_VictorSPX(6);

	// Pneumatics are used to place the hatch
	Compressor _compressor = new Compressor(0);
	DoubleSolenoid _extend = new DoubleSolenoid(0, 1);

	// Camera
	GripPipeline _Pipeline = new GripPipeline();
	ADXRS450_Gyro _gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); // SPI.Port.kOnboardCS0

	// TODO: might need to tune these values
	/*
	 * final double kMinLineLength = 30; final double kMaxLineLength = 1000; final
	 * double kAcceptableLineLengthDiff = 10; final double kAcceptableSlopeDiff =
	 * 1.0; final double kTargetAngle = 29.06; // degrees final double
	 * kAcceptableAngleError = 5; // degrees
	 * 
	 * private double getLineLength(Line line) { return Math.sqrt(Math.pow(line.x2 -
	 * line.x1, 2) + Math.pow(line.y2 - line.y1, 2)); }
	 * 
	 * private double getLineSlope(Line line) { return (line.y2 - line.y1) /
	 * (line.x2 - line.x1); }
	 * 
	 * private double dotProduct(Line line1, Line line2) { // We use abs because we
	 * want all the vectors in the upper right quadrant return Math.abs(line1.x2 -
	 * line1.x1) * Math.abs(line2.x2 - line2.x1) + Math.abs(line1.y2 - line1.y1) *
	 * Math.abs(line2.y2 - line2.y1); }
	 * 
	 * private double angleBetweenLines(Line line1, Line line2) { // From:
	 * http://www.wikihow.com/Find-the-Angle-Between-Two-Vectors // cos(T) = dot(v1,
	 * v2) / (len(v1) * len(v2)) // T is the angle between vector v1 and v2 double d
	 * = dotProduct(line1, line2); double l = getLineLength(line1) *
	 * getLineLength(line2); return Math.acos(d / l) * 180.0 / Math.PI; // degrees }
	 */
	boolean lastStepLowSpeed;
	boolean lowSpeed;

    public void robotInit() {


		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(256, 144);
		
		configureDrive();
		/*new Thread(() -> {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
			camera.setResolution(256, 144);
			
			// TODO: potential code to use vision to detect hatches
		 	CvSink cvSink = CameraServer.getInstance().getVideo();
		 	Mat source = new Mat();
		 	while (!Thread.interrupted()) {
		 		if (cvSink.grabFrame(source) == 0) {
		 			// Do nothing on errors
		 		} else {
		 			_Pipeline.process(source);
		 			CameraServer.getVideo(cvSink);
				 };
				}
			});	// Find parallel lines
		 			ArrayList<ArrayList<Line>> parallelLines = new ArrayList<ArrayList<Line>>();
		 			SmartDashboard.putNumber("Lines", lines.size());
		 			for (int i = 0; i < lines.size(); ++i) {
		 				Line line1 = lines.get(i);
		 				if (lengths[i] >= kMinLineLength && lengths[i] <= kMaxLineLength) {
		 					for (int j = 0; j < lines.size(); ++j) {
		 						if (i != j) {
		 							Line line2 = lines.get(j);
		 							if (Math.abs(lengths[j] - lengths[j]) <= kAcceptableLineLengthDiff) {
		 								if (Math.abs(slopes[j] - slopes[i]) <= kAcceptableSlopeDiff) {
		 									// TODO: do we need to make sure that we aren't looking at multiple segments of the same line? 
		 									ArrayList<Line> pl = new ArrayList<Line>();
		 									pl.add(line1);
		 									pl.add(line2);
		 									parallelLines.add(pl);
		 								}
		 							}
		 						}
		 					}
		 				}
		 			}
		 			// Now look for lines that converge at the expected angle
		 			for (int i = 0; i < parallelLines.size(); ++i) {
		 				ArrayList<Line> pLines1 = parallelLines.get(i);
		 				for (int j = 0; j < parallelLines.size(); ++j) {
		 					if (i != j) {
		 						ArrayList<Line> pLines2 = parallelLines.get(j);
		 						double angle = angleBetweenLines(pLines1.get(0), pLines2.get(0));
		 						SmartDashboard.putNumber(("Calculated Angle"), angle);
		 						if (Math.abs(kTargetAngle - angle) <= kAcceptableAngleError) {
									 
									 
		 						}
		 					}
		 				}
		 			}
				}
			 }
	
		});*/
	
		_gyro.calibrate();
		_gyro.reset();
	}

	public void autonomousPeriodic(){
		teleopPeriodic();
	}
	
	
	double kP = 5;
	double kI = 0;
	double kD = 0; 

    public void teleopPeriodic() {
		

		double scaling = 0.3;

		if(_joy.getRawButton(4)){
			scaling = .8;
		}


		// Get input
		double forward = -_joy.getY()*scaling;
		if (_joy.getY() <= 0.2 && _joy.getY() >= -0.2) {
			forward=0;
		}
		double turn = _joy.getZ() * 0.4;
		boolean ballIntake = _joy.getRawButton(7);
		boolean expelBall = _joy.getRawButton(2);
		boolean releaseHatch = _joy.getRawButton(8);
		boolean fastTurn = _joy.getRawButton(6);
		boolean pushback = _joy.getRawButton(10);
		boolean center = _joy.getRawButton(5);


		double angle = _gyro.getAngle();

		// Drive control
		/*if (Math.abs(forward) < 0.10) forward = 0; // within 10% joystick, make it zero
		if (Math.abs(turn) < 0.10) turn = 0; // within 10% joystick, make it zero*/

		if(forward > 0){
			forward = forward * forward;
		} else{
			forward = forward * forward * -1;
		}

		

		if(forward < 0){
			turn = turn * -1;
		}
	
	  
		  double angularPower;
		  boolean overPower;
	  
		  if (fastTurn) {
			overPower = true;
			angularPower = turn;
		  } else {
			overPower = false;
			angularPower = Math.abs(forward) * turn;
	
		  }
	  
		 double leftMotorOutput = forward  + angularPower ;
		 double rightMotorOutput = forward - angularPower;
	  
		  //If rotation is overpowered, reduce both outputs to within acceptable range
		  if (overPower) {
			if (leftMotorOutput > 1.0) {
			  rightMotorOutput -= leftMotorOutput - 1.0;
			  leftMotorOutput = 1.0;
			} else if (rightMotorOutput > 1.0) {
			  leftMotorOutput -= rightMotorOutput - 1.0;
			  rightMotorOutput = 1.0;
			} else if (leftMotorOutput < -1.0) {
			  rightMotorOutput -= leftMotorOutput + 1.0;
			  leftMotorOutput = -1.0;
			} else if (rightMotorOutput < -1.0) {
			  leftMotorOutput -= rightMotorOutput + 1.0;
			  rightMotorOutput = -1.0;
			}
		  }

		  
		 /* _frontRightMotor.set(ControlMode.Velocity, forward * kMaxVelocity, DemandType.AuxPID, turn);
		  _frontLeftMotor.follow(_frontRightMotor, FollowerType.AuxOutput1);
		  _leftSlave1.follow(_frontRightMotor, FollowerType.AuxOutput1);
		  _rightSlave1.follow(_frontRightMotor);*/
		 _frontRightMotor.set(ControlMode.Velocity, rightMotorOutput * kMaxVelocity);
		 _frontLeftMotor.set(ControlMode.Velocity, leftMotorOutput * kMaxVelocity);
		//  _leftSlave1.follow(_frontLeftMotor, FollowerType.PercentOutput);
		//  _rightSlave1.follow(_frontRightMotor, FollowerType.PercentOutput);
		
			//Centering button
			/*double currentAngle = _gryo.getAngle;
            if (center) {
                double adjustedAngle;

                //Left Adjustment
                if (THETHINGISLEFTOFCENTER) {
                    adjustedAngle = -25;
                } else {
                    adjustedAngle = 0;
                }

                //Right Adjustment
                if (THETHINGISRIGHTOFCENTER) {
                    adjustedAngle = 25;
                } else {
                    adjustedAngle = 0;
                }

                _frontRightMotor.set(ControlMode.Velocity, forward * kMaxVelocity, DemandType.AuxPID, adjustedAngle);
                

            }

			*/
		// Ball intake/expel control
		_ballIntake.set(ballIntake ? 0.6 : (expelBall ? -0.6 : 0.0));
		// Hatch placement control
		//  if(releaseHatch){
		//  	_extend.set(Value.kReverse);
		//  } else {
		//  	_extend.set(Value.kForward);
		//  }


		if(pushback){
			releaseHatch = true;
			ballIntake = true;
		} else{
			releaseHatch = false;
			ballIntake = false;
		}

		// SmartDashboard
		SmartDashboard.putNumber("Gyro Output", angle);
		SmartDashboard.putBoolean("Gyro Connected", _gyro.isConnected());
		SmartDashboard.putNumber("Y-Axis", _joy.getY());
		SmartDashboard.putNumber("Desired forward", forward);
		SmartDashboard.putNumber("Desired turn", turn);
		// SmartDashboard.putBoolean("Pressure", _compressor.getPressureSwitchValue());
		// SmartDashboard.putNumber("Compressor Current", _compressor.getCompressorCurrent());
		// SmartDashboard.putBoolean("Compressor CLC", _compressor.getClosedLoopControl());
	}


	private void configureDrive(){

		_frontLeftMotor.configFactoryDefault();
		_frontRightMotor.configFactoryDefault();


		/** Feedback Sensor Configuration */
		
		
		// Initialize the drive (take our extra talons and just have them
		//   follow the same input as the main Talons)
    	_leftSlave1.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		_frontLeftMotor.setInverted(false);
		_leftSlave1.setInverted(false);
		_frontRightMotor.setInverted(true);
		_rightSlave1.setInverted(true);

		_frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		_frontRightMotor.setSensorPhase(true);

		_frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		_frontLeftMotor.setSensorPhase(true);

		
		 _frontLeftMotor.config_kF(0, kFeedForwardVelocity);
		 _frontLeftMotor.config_kP(0,1.2);
		 _frontLeftMotor.config_kI(0,0.001);
		 _frontLeftMotor.config_kD(0,12);
		 _frontLeftMotor.config_IntegralZone(0, 50);
			_frontLeftMotor.configClosedloopRamp(.35);
		

		_frontRightMotor.config_kF(0, kFeedForwardVelocity);
		_frontRightMotor.config_kP(0,1.2);
		_frontRightMotor.config_kI(0,0.001);
		_frontRightMotor.config_kD(0,12);
		_frontRightMotor.config_IntegralZone(0, 50);
		_frontRightMotor.configClosedloopRamp(.35);
	}


	private void configureAngleDrive(){
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor to a Quad Encoder*/
		_frontLeftMotor.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Relative, 			// Local Feedback Source
													0, 30			// PID Slot for Source [0, 1]
													);					// Configuration Timeout

		/* Configure the Remote Talon's selected sensor as a remote sensor for the right Talon */
		_frontRightMotor.configRemoteFeedbackFilter(_frontLeftMotor.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												1						// Source number [0, 1]
												);						// Configuration Timeout
		
		/* Setup Sum signal to be used for Distance */
		_frontRightMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1);	// Feedback Device of Remote Talon
		_frontRightMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		_frontRightMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1);
		_frontRightMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		_frontRightMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													0, 30);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		_frontRightMotor.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														0		// PID Slot of Source 
														,30);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		_frontRightMotor.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													1
													, 30);
		
		/* Don't scale the Feedback Sensor (use 1 for 1:1 ratio) */
		_frontRightMotor.configSelectedFeedbackCoefficient(	1, 1, 30);
		/* FPID Gains for turn servo */
		_frontRightMotor.config_kP(1, 2);
		_frontRightMotor.config_kI(1, 0);
		_frontRightMotor.config_kD(1, 8);
		_frontRightMotor.config_kF(1, 0);
		_frontRightMotor.config_IntegralZone(1, 0);
		_frontRightMotor.configAllowableClosedloopError(1, 0);
		
		
		// Initialize the drive (take our extra talons and just have them
		//   follow the same input as the main Talons)
    	_leftSlave1.follow(_frontLeftMotor);
		_rightSlave1.follow(_frontRightMotor);
		_frontLeftMotor.setInverted(false);
		_leftSlave1.setInverted(false);
		_frontRightMotor.setInverted(true);
		_rightSlave1.setInverted(true);

		_frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		_frontRightMotor.setSensorPhase(true);

		_frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		_frontLeftMotor.setSensorPhase(true);

		
		 _frontLeftMotor.config_kF(0, kFeedForwardVelocity);
		 _frontLeftMotor.config_kP(0,1.2);
		 _frontLeftMotor.config_kI(0,0.001);
		 _frontLeftMotor.config_kD(0,12);
		 _frontLeftMotor.config_IntegralZone(0, 50);
		_frontLeftMotor.configClosedloopRamp(.35);
		

		_frontRightMotor.config_kF(0, kFeedForwardVelocity);
		_frontRightMotor.config_kP(0,1.2);
		_frontRightMotor.config_kI(0,0.001);
		_frontRightMotor.config_kD(0,12);
		_frontRightMotor.config_IntegralZone(0, 50);
		_frontRightMotor.configClosedloopRamp(.35);
	}



	private Double CalculateMaxSensorVelocity(){	
		int kSensorUnitsPerRotation = 4096;
		int kMaxMotorRPM = 5330;
		double kGearRatio = 10.75;
		return (kMaxMotorRPM / kGearRatio) * kSensorUnitsPerRotation / 600;
	}

	private double kMaxVelocity = 3000;
	private double kMaxTurn = 2086;
	private double kFeedForwardVelocity = 1023/kMaxVelocity * 1.2;
	// private double kFeedForwardTurn = 1023/kMaxTurn * 1.2;
			
}