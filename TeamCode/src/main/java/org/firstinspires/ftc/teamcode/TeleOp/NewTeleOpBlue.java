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
public class NewTeleOpBlue extends LinearOpMode {
   public static double strafePower = 0.7;
   public static double longShootPos = 0;
   public static double shortShootPos = 0;
   public static double rampPosition = 0.09;
   DcMotor lf, lr, rf, rr;
   Follower follower;
   Shooter shooter;
   Servo ramp;
   Servo hoodServo;
   Mechanisms mechanisms;

   //   ColorCommand colorCommand;
   ElapsedTime IntakeTimer = new ElapsedTime();

   @Override
   public void runOpMode() throws InterruptedException {


      mechanisms = new Mechanisms(hardwareMap, telemetry);
      follower = Constants.createFollower(hardwareMap);
      shooter = new Shooter(hardwareMap);
//      colorCommand = new ColorCommand(hardwareMap, "colorSensor");
      ramp = hardwareMap.get(Servo.class, "ramp");
      ramp.setPosition(0);
      hoodServo = hardwareMap.get(Servo.class, "Hood2");
      hoodServo.setPosition(0);
      follower.startTeleopDrive(false);
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


         if (gamepad2.yWasPressed()) {
            mechanisms.shoot();
         }

         if (shooter.isVelocityReached()) {
            gamepad2.rumble(100);
         }
//         if (colorCommand.ledState==true){
//            gamepad2.rumble(2500);
//         }
         if (gamepad1.right_trigger > 0.25) {
            follower.setMaxPower(0.4);
         } else {
            follower.setMaxPower(1);
         }
         follower.setTeleOpDrive(-gamepad1.left_stick_y * 0.9, -gamepad1.left_stick_x * 0.9, -gamepad1.right_stick_x * 0.67, true);
//

         follower.update();
         mechanisms.update();
//         colorCommand.updateLED();
//         colorCommand.checkledstate();
      }

   }
}
