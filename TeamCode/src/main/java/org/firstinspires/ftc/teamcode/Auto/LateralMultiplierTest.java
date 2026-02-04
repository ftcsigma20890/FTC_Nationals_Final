package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name="Lateral Multiplier Test")
public class LateralMultiplierTest extends LinearOpMode {

   // 🔁 ONLY CHANGE THIS NUMBER
   public static double LATERAL_MULTIPLIER = 1.45;

   Follower follower;

   @Override
   public void runOpMode() {

      // Apply multiplier
      Constants.LATERAL_MULTIPLIER = LATERAL_MULTIPLIER;

      follower = Constants.createFollower(hardwareMap);
      follower.setMaxPower(0.6);
      // Start Pose
      follower.setStartingPose(new Pose(30, 130, Math.toRadians(0)));

      PathChain strafeTest = follower.pathBuilder()
         // Strafe Right
         .addPath(new BezierLine(
            new Pose(30, 130),
            new Pose(30, 60)
         ))
         .setConstantHeadingInterpolation(Math.toRadians(0))

         // Strafe Left (return)
         .addPath(new BezierLine(
            new Pose(30, 130),
            new Pose(30, 60)
         ))
         .setConstantHeadingInterpolation(Math.toRadians(180))

         .build();

      waitForStart();

      follower.followPath(strafeTest.getPath(0));

      while (opModeIsActive()) {

         follower.update();

         if (!follower.isBusy()) {
            follower.followPath(strafeTest.getPath(1));
            break;
         }

         telemetry.addData("Testing Lateral Multiplier", LATERAL_MULTIPLIER);
         telemetry.update();
      }
   }
}