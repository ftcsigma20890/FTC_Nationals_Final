package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp(name = "NewTeleOpBlue_Fixed", group = "TeleOp")
public class TestTele extends LinearOpMode {
   // Adjusted strafePower to 1.0 to prevent clipping by default
   public static double strafePower = 1.0;
   public static double longShootPos = 0;
   public static double shortShootPos = 0;
   public static double rampPosition = 0.09;

   Follower follower;
   Shooter shooter;
   Servo ramp;
   Servo hoodServo;
   Mechanisms mechanisms;

   @Override
   public void runOpMode() throws InterruptedException {
      // Initialize hardware
      mechanisms = new Mechanisms(hardwareMap, telemetry);
      follower = Constants.createFollower(hardwareMap);
      shooter = new Shooter(hardwareMap);
      ramp = hardwareMap.get(Servo.class, "ramp");
      hoodServo = hardwareMap.get(Servo.class, "Hood2");

      // Set initial positions
      ramp.setPosition(0);
      hoodServo.setPosition(0);

      // Initialize Pedro Pathing follower
      follower.startTeleopDrive(true);
      mechanisms.pipelineSwitch(0);

      telemetry.addData("Status", "Initialized - Check Motor Directions in Constants");
      telemetry.update();

      waitForStart();

      while (opModeIsActive()) {
         // --- DRIVE LOGIC WITH NORMALIZATION ---
         // Note: gamepad y is inverted
         double axial = -gamepad1.left_stick_y;
         double lateral = gamepad1.left_stick_x * strafePower;
         double yaw = -gamepad1.right_stick_x;

         // Normalize powers so the robot doesn't "shudder" when combining movements
         // This prevents the "clipping" issue seen in your video.
         double denominator = Math.max(Math.abs(axial) + Math.abs(lateral) + Math.abs(yaw), 1.0);

         double driveAxial = axial / denominator;
         double driveLateral = lateral / denominator;
         double driveYaw = yaw / denominator;

         follower.setTeleOpDrive(driveAxial, driveLateral, driveYaw, true);

         // --- SHOOTER CONTROLS ---
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

         // --- INTAKE CONTROLS ---
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

         // Haptic feedback for driver
         if (shooter.isVelocityReached()) {
            gamepad2.rumble(100);
         }

         // Update systems
         follower.update();
         mechanisms.update();

         // Debugging Telemetry
         telemetry.addData("Drive Power", "A: %.2f, L: %.2f, Y: %.2f", driveAxial, driveLateral, driveYaw);
         telemetry.update();
      }
   }
}