package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class TestDrive extends LinearOpMode {
   public static double leftFront = 0.95, leftBack = 1, rightFront = 0.95, rightBack = 1;
   @Override
   public void runOpMode() throws InterruptedException {
      Servo servo = hardwareMap.get(Servo.class, "Hood2");
      // Declare our motors
      // Make sure your ID's match your configuration
      DcMotor frontLeftMotor = hardwareMap.dcMotor.get("LF");
      DcMotor backLeftMotor = hardwareMap.dcMotor.get("LR");
      DcMotor frontRightMotor = hardwareMap.dcMotor.get("RF");
      DcMotor backRightMotor = hardwareMap.dcMotor.get("RR");

      // Reverse the right side motors. This may be wrong for your setup.
      // If your robot moves backwards when commanded to go forwards,
      // reverse the left side instead.
      // See the note about this earlier on this page.
      frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
      servo.setPosition(0);

      waitForStart();

      if (isStopRequested()) return;

      while (opModeIsActive()) {
         double y = -gamepad1.left_stick_y * 0.85; // Remember, Y stick value is reversed
         double x = gamepad1.left_stick_x * 1; // Counteract imperfect strafing
         double rx = gamepad1.right_stick_x * 0.5;

         // Denominator is the largest motor power (absolute value) or 1
         // This ensures all the powers maintain the same ratio,
         // but only if at least one is out of the range [-1, 1]
         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         double frontLeftPower = (y + x + rx) / denominator;
         double backLeftPower = (y - x + rx) / denominator;
         double frontRightPower = (y - x - rx) / denominator;
         double backRightPower = (y + x - rx) / denominator;

         frontLeftMotor.setPower(frontLeftPower);
         backLeftMotor.setPower(backLeftPower );
         frontRightMotor.setPower(frontRightPower);
         backRightMotor.setPower(backRightPower);
      }
   }
}