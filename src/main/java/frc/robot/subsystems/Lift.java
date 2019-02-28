package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Lift extends Subsystem{

	double encoderCal = 0.0026929;
	double liftDeadzone = 1.25;
	double liftAutoSpeed = 0.8;

    //Init talonSRX
    TalonSRX TALONLIFT = new TalonSRX(RobotMap.TALONLIFT);

    final int kTimeoutMs = 30;
    final boolean kDiscontinuityPresent = true;
	final int kBookEnd_0 = 910;		/* 80 deg */
	final int kBookEnd_1 = 1137;	/* 100 deg */

	double range = 1.5; //range the lights will come on at

	String holes[] = {"psHatch","psBall","rHatch1","rHatch2","rHatch3","rBall1","rBall2","rBall3", "autoHatch"};
	double heights[] = {11.5, 34, 11.25, 38.75, 64, 18, 46, 72, 24};


    public Lift(){

        double rampRate = 0.2;

        TALONLIFT.configFactoryDefault();

        initQuadrature();

        TALONLIFT.configOpenloopRamp(rampRate);

        TALONLIFT.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kTimeoutMs);

    }

    //Function that actuate the lift
    public void MoveLift(double liftSpeed){

        TALONLIFT.set(ControlMode.PercentOutput, -liftSpeed);

    }

    public void resetEncoder(){
        TALONLIFT.setSelectedSensorPosition(0, 0, this.kTimeoutMs);
    }

    public double getEncoder(){

        double encoderMove = -TALONLIFT.getSelectedSensorPosition(0);
        encoderMove = encoderMove * encoderCal;

        return encoderMove;
	}

	public void liftIndicatiors(){

		double height = getEncoder();
		
		for(int i = 0; i < heights.length; i++){

			if(height >= heights[i] - range && heights[i] + range >= height){
				SmartDashboard.putBoolean(holes[i], true);
			} else {
				SmartDashboard.putBoolean(holes[i], false);
			}

		}	
	}

	public void gotoBall(){

		//1 for ball

		double height = getEncoder();

		if(height >= heights[1] - liftDeadzone && heights[1] + liftDeadzone >= height){

		} else if(height > heights[1]){
			MoveLift(liftAutoSpeed);
		} else if(height < heights[1]){
			MoveLift(-liftAutoSpeed);
		}

	}

	public void gotoHatch(){

		//0 for hatch

		double height = getEncoder();

		if(height >= heights[0] - liftDeadzone && heights[0] + liftDeadzone >= height){

		} else if(height > heights[0]){
			MoveLift(liftAutoSpeed);
		} else if(height < heights[0]){
			MoveLift(-liftAutoSpeed);
		}

	}

	public void gotorHatch2(){

		//0 for hatch

		double height = getEncoder();

		if(height >= heights[3] - liftDeadzone && heights[3] + liftDeadzone >= height){

		} else if(height > heights[3]){
			MoveLift(liftAutoSpeed);
		} else if(height < heights[3]){
			MoveLift(-liftAutoSpeed);
		}

	}

	public void gotorHatch3(){

		//0 for hatch

		double height = getEncoder();

		if(height >= (heights[4]+3) - liftDeadzone && (heights[4]+3) + liftDeadzone >= height){

		} else if(height > (heights[4]+3)){
			MoveLift(liftAutoSpeed);
		} else if(height < (heights[4]+3)){
			MoveLift(-liftAutoSpeed);
		}

	}

    public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = TALONLIFT.getSensorCollection().getPulseWidthPosition();

		/**
		 * If there is a discontinuity in our measured range, subtract one half
		 * rotation to remove it
		 */
		if (kDiscontinuityPresent) {

			/* Calculate the center */
			int newCenter;
			newCenter = (kBookEnd_0 + kBookEnd_1) / 2;
			newCenter &= 0xFFF;

			/**
			 * Apply the offset so the discontinuity is in the unused portion of
			 * the sensor
			 */
			pulseWidth -= newCenter;
		}

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
		pulseWidth = pulseWidth & 0xFFF;

		/* Update Quadrature position */
		TALONLIFT.getSensorCollection().setQuadraturePosition(pulseWidth, kTimeoutMs);
	}

    @Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		//setDefaultCommand(new DriveArcade());
	}

}