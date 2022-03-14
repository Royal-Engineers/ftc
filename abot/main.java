package org.firstinspires.ftc.teamcode.A_Bot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;

@TeleOp(name = "A_bot", group = "A_bot")

public class A_BOT extends LinearOpMode {
	private Blinker control_Hub;
	private Blinker expansion_Hub_2;
	private DcMotor mana;
	private DcMotor brat_drept;
	private DcMotor brat_stang;
	private DcMotor carusel;
	private Servo manusa;
	private Gyroscope imu;
	private DcMotor tractiune_1;
	private DcMotor tractiune_2;
	private float joy_l_x;
	private float joy_l_y;
	private float joy_r_x;
	private float joy_r_y;
	
	@Override
	public void runOpMode(){
		//senzori
	//	touch = hardwareMap.get(TouchSensor.class,"touch");
		//motoare
		tractiune_1 = hardwareMap.get(DcMotor.class,"tractiune_1");
		tractiune_2 = hardwareMap.get(DcMotor.class,"tractiune_2");
		brat_stang = hardwareMap.get(DcMotor.class,"brat_stang");
		brat_drept = hardwareMap.get(DcMotor.class,"brat_drept");
		carusel = hardwareMap.get(DcMotor.class, "carusel");
		manusa = hardwareMap.get(Servo.class, "manusa");
		mana = hardwareMap.get(DcMotor.class,"brat");

		mana.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// silviu
		float putere,putere2, sens = 1;
		double prins = 0;
		// silviu
		
		telemetry.addData("Inceput","ok");
		telemetry.update();
		waitForStart();
		while(opModeIsActive()){
			joy_l_x = gamepad1.left_stick_x/2;
			joy_l_y = gamepad1.left_stick_y/2;
			joy_r_x = gamepad1.right_stick_x/2;
			joy_r_y = gamepad1.right_stick_y/2;
			putere = gamepad1.right_trigger - gamepad1.left_trigger;
			putere2 = gamepad2.right_trigger - gamepad2.left_trigger;
			putere2/=5;
			if(putere + putere2< 0) sens = -1;
			else if (putere + putere2 > 0) sens = 1;
			telemetry.addData("Directie:","Stanga");
			telemetry.addData("joy_l_x",joy_l_x);
			brat_stang.setPower(-joy_l_x * sens + -gamepad2.right_stick_x/10*sens);
			brat_drept.setPower(-joy_l_x * sens + -gamepad2.right_stick_x/10*sens);
			tractiune_1.setPower(putere+putere2);
			tractiune_2.setPower(-putere-putere2);
			carusel.setPower((gamepad2.b == true)? 0.3:0);
			if(gamepad2.a == true)
			{
				prins = 1;
				
			}
			else prins =0;
			manusa.setPosition(prins);
			mana.setPower(-gamepad2.left_stick_y);

			telemetry.addData("Mana",mana.getPower());
			telemetry.addData("Pozitie mana",mana.getCurrentPosition());
			telemetry.update();
		}
	}
}
