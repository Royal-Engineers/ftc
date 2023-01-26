package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

	double leftPower;
	double rightPower;
	public static int target = 0;
	public boolean requestStop = false;
	public DcMotorEx left, right;
	public int mainTarget = 0;

	public void moveLift(Constants.LiftTargets input) {
		switch (input) {
			case PICKUP:
				moveLift(0);
				break;

			case LOW:
				moveLift(1700); // 1558.636363636
				break;

			case MEDIUM:
				moveLift(2900); // 2713.181818182
				break;

			case HIGH:
				moveLift(4050); // 3867.727272727
				break;

			case PUTDOWN:
				moveLift(870);
				break;
		}
	}

	public void moveLift(int target) {
		setLiftPosition(target);
		mainTarget = target;
	}

	public void init(HardwareMap hardwareMap) {
		left = hardwareMap.get(DcMotorEx.class,"liftStanga" );
		left.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		right = hardwareMap.get(DcMotorEx.class, "liftDreapta" );
		right.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	public void setLiftPosition(int height) {
		target = height;
		double currentPos = left.getCurrentPosition();

		if (currentPos < target) {
			// Going up
			leftPower = 0.2;
			rightPower = -0.2;
		} else if (currentPos > target) {
			// Going down
			leftPower = -0.2;
			rightPower = 0.2;
		}
		
		// se schimba aici cu + sau - in functie de directie
		// sens pozitiv = - aici
		// sens negativ = + aici
		left.setTargetPosition((int) -target);
		right.setTargetPosition((int) -target);

		left.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		right.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		left.setPower(leftPower);
		right.setPower(rightPower);
	}

	public int getPosition() {
		// Using the average of two left and right
		return (left.getCurrentPosition())/* - (right.getCurrentPosition())) / 2*/;
	}

	public int getTarget() {
		return mainTarget;
	}
}
