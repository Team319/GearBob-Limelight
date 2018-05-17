package org.usfirst.frc.team319.robot.subsystems;

import org.usfirst.frc.team319.models.BobTalonSRX;
import org.usfirst.frc.team319.models.MotionParameters;
import org.usfirst.frc.team319.models.SRXGains;
import org.usfirst.frc.team319.robot.commands.wrist.JoystickWrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Wrist extends Subsystem {

	private int maxUpTravelPosition = 4144;
	private int maxDownTravelPosition = 319;
	private int homePosition = 319;
	private int safeposition = 150;
	private int switchPostion = 1319;
	private int exchangePosition = 4200;
	private int autoSwitchPostion = 3344;
	private int parallelPosition = 4144;
	private int collectPosition = 4500;

	public final static int WRIST_PROFILE_UP = 0;
	public final static int WRIST_PROFILE_DOWN = 1;

	private int upPositionLimit = homePosition;
	private int downPositionLimit = maxDownTravelPosition;

	private int targetPosition = homePosition;
	private final static int onTargetThreshold = 100;

	private SRXGains upGains = new SRXGains(WRIST_PROFILE_UP, 1.00, 0.005, 16.0, 0.799, 150);
	private SRXGains downGains = new SRXGains(WRIST_PROFILE_DOWN, 0.400, 0.005, 15.0, 0.799, 150);

	private MotionParameters upMotionParameters = new MotionParameters(5000, 1024, upGains);
	private MotionParameters downMotionParameters = new MotionParameters(0, 0, downGains);

	public final BobTalonSRX wristMotor = new BobTalonSRX(6);// just a guess -Josh and Kelly

	public Wrist() {
		this.wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

		this.wristMotor.setInverted(false);
		this.wristMotor.setSensorPhase(true);

		this.wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
		this.wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

		this.wristMotor.configForwardSoftLimitThreshold(downPositionLimit);
		this.wristMotor.configReverseSoftLimitThreshold(upPositionLimit);

		this.wristMotor.configForwardSoftLimitEnable(true);
		this.wristMotor.configReverseSoftLimitEnable(true);

		this.wristMotor.setNeutralMode(NeutralMode.Brake);

		wristMotor.configMotionParameters(upMotionParameters);
		wristMotor.configMotionParameters(downMotionParameters);

	}

	public void initDefaultCommand() {
		setDefaultCommand(new JoystickWrist());
	}
	
	public void wristMove(ControlMode controlMode, double targetPosition) {
		this.manageMotion(targetPosition);
		wristMotor.set(controlMode, targetPosition);
	}
	
	public void motionMagicControl() {
		manageMotion(targetPosition);
		wristMotor.set(ControlMode.MotionMagic, targetPosition);
	}

	public void motionMagicPositionControl(double positionScalar) {
		double encoderPosition = 0;

		if (positionScalar > 0) {
			encoderPosition = positionScalar * upPositionLimit;
		} else {
			encoderPosition = -positionScalar * downPositionLimit;
		}

		wristMotor.set(ControlMode.MotionMagic, encoderPosition);
	}

	public void manageMotion(double targetPosition) {
    	
    	double currentPosition = getCurrentPosition();
    	
    	
    	if (currentPosition > homePosition) {
    		if(currentPosition > targetPosition) {
    			wristMotor.selectMotionParameters(downMotionParameters);
    		} else {
    			wristMotor.selectMotionParameters(upMotionParameters);
    		}
    	} else {
			if (currentPosition > targetPosition) {
				wristMotor.selectMotionParameters(upMotionParameters);
			} else {
				wristMotor.selectMotionParameters(downMotionParameters);
			}
		this.wristMotor.configForwardSoftLimitThreshold(downPositionLimit);
		this.wristMotor.configReverseSoftLimitThreshold(upPositionLimit);
		
    	}
	}
	
	
	public int getTargetPosition() {
		return this.targetPosition;
	}
	
	public int getCurrentPosition() {
		return this.wristMotor.getSelectedSensorPosition();
	}
	public boolean isValidPosition(int position) {
		return (position >= upPositionLimit && position <= downPositionLimit);
	}

	public void incrementTargetPosition(int increment) {
		int currentTargetPosition = this.targetPosition;
		int newTargetPosition = currentTargetPosition + increment;
		if (isValidPosition(newTargetPosition)) {
			this.targetPosition = newTargetPosition;
		}		
	}
	
}
