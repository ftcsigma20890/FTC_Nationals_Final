package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
//import org.firstinspires.ftc.teamcode.Mechanisms.ColorCommand;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp
public class TestTeleOp extends LinearOpMode {
   DcMotor lf, lr, rf, rr;
   public static double strafePower = 0.7;
   public static double longShootPos = 0;
   public static double shortShootPos = 0;
   public static double rampPosition = 0.09;
   Follower follower;
   Shooter shooter;
   Servo ramp;
   Servo hoodServo;
   Mechanisms mechanisms;

   //   ColorCommand colorCommand;
   ElapsedTime IntakeTimer= new ElapsedTime();

   @Override
   public void runOpMode() throws InterruptedException {
      lf = hardwareMap.get(DcMotor.class, "LF");
      lr = hardwareMap.get(DcMotor.class, "LR");
      rf = hardwareMap.get(DcMotor.class, "RF");
      rr = hardwareMap.get(DcMotor.class, "RR");

      lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      lf.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
      lr.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
      rf.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
      rr.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);


      mechanisms = new Mechanisms(hardwareMap, telemetry);
      follower = Constants.createFollower(hardwareMap);
      shooter = new Shooter(hardwareMap);
//      colorCommand = new ColorCommand(hardwareMap, "colorSensor");
      ramp = hardwareMap.get(Servo.class, "ramp");
      ramp.setPosition(0);
      hoodServo = hardwareMap.get(Servo.class, "Hood2");
      hoodServo.setPosition(0);
//      follower.startTeleopDrive(false);
      follower.update();
      mechanisms.pipelineSwitch(0);
      waitForStart();
      while (opModeIsActive()) {
         if (gamepad2.dpad_down) {
            shooter.startShortShooter();
            hoodServo.setPosition(shortShootPos);
         } else if (gamepad2.dpad_up) {
            shooter.startLongShooter();
            hoodServo.setPosition(longShootPos);
         } else if (gamepad2.right_bumper) {
            shooter.startShooter();
         } else {
            shooter.stopShooter();
         }
         if (gamepad2.left_bumper) {
            mechanisms.startIntake();
            ramp.setPosition(0);
         } else if (gamepad2.a) {
            mechanisms.startIntake();
            ramp.setPosition(rampPosition);
         } else if (gamepad2.left_trigger > 0.25) {
            mechanisms.reverseIntake();
         } else {
            mechanisms.stopIntake();
         }

         if (gamepad2.yWasPressed()) {
            mechanisms.setTurretTicks(0);
         }

         if (gamepad2.x){
            //if the balls get stuck then the bot will intake 0.2secs and then outake ofr 0.75
            IntakeTimer.reset();
         } else if (gamepad2.xWasReleased()){
            mechanisms.reverseIntake();
            if (IntakeTimer.seconds() >= 0.2) {
               mechanisms.startIntake();
               if(IntakeTimer.seconds() > 1){
                  mechanisms.stopIntake();
               }
            }

         }
         if (gamepad2.yWasPressed()) {
            mechanisms.shoot();
         }

         if (shooter.isVelocityReached()) {
            gamepad2.rumble(100);
         }
//         if (colorCommand.ledState==true){
//            gamepad2.rumble(2500);
//         }

//         follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x * strafePower, -gamepad1.right_stick_x * 0.67, true);
//         follower.setTeleOpDrive(
//            -gamepad1.left_stick_y,
//            -gamepad1.left_stick_x * strafePower,
//            -gamepad1.right_stick_x * 0.7,
//            true
//         );
         double y = -gamepad1.left_stick_y;   // forward
         double x =  gamepad1.left_stick_x * 0.6;
         x = Math.copySign(x * x, x);
         if(x > 0) x *=0.65;
         x = x * 1.25;// strafe
         double rx = gamepad1.right_stick_x * 0.6; // turn
         if(x > 0.4 && Math.abs(y)< 0.2){
            rx -= 0.18 * x;
         }
         double denominator = Math.max(
            Math.abs(y) + Math.abs(x) + Math.abs(rx), 1
         );

         double lfPower = (y + x + rx) / denominator;
         double lrPower = (y - x + rx) / denominator;
         double rfPower = (y - x - rx) / denominator;
         double rrPower = (y + x - rx) / denominator;

         lf.setPower(lfPower);
         lr.setPower(lrPower);
         rf.setPower(rfPower);
         rr.setPower(rrPower);
         follower.update();
         mechanisms.update();
//         colorCommand.updateLED();
//         colorCommand.checkledstate();

      }
   }
}