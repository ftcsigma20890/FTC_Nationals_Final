package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoRedClose12 extends LinearOpMode {
   public static double intakeDriveSpeed = 1;
   Follower follower;
   int pathState = 0;
   Mechanisms mechanisms;

   @Override
   public void runOpMode() throws InterruptedException {

      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(109.660, 135.955, Math.toRadians(0)));
      follower.setMaxPower(1);
      mechanisms = new Mechanisms(hardwareMap, telemetry);

      PathChain myPath = follower
         .pathBuilder()
         .addPath(
            new BezierLine(
               new Pose(109.660, 135.955),

               new Pose(88.000, 84.334)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
         .addPath(
            new BezierLine(
               new Pose(88.000, 84.334),

               new Pose(127.476, 83.330)
            )
         ).setTangentHeadingInterpolation()
         .addPath(
            new BezierLine(
               new Pose(127.476, 83.330),

               new Pose(87.916, 84.370)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-49))
         .addPath(
            new BezierCurve(
               new Pose(87.916, 84.370),
               new Pose(96.398, 55.348),
               new Pose(128.145, 59.228)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(-49), Math.toRadians(0))
         .addPath(
            new BezierLine(
               new Pose(128.145, 59.228),

               new Pose(87.744, 84.621)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-49))
         .addPath(
            new BezierCurve(
               new Pose(87.744, 84.621),
               new Pose(86.522, 42.304),
               new Pose(105.163, 32.685),
               new Pose(130.214, 34.889)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(-49), Math.toRadians(0))
         .addPath(
            new BezierLine(
               new Pose(130.214, 34.889),

               new Pose(87.705, 84.772)
            )
         ).setTangentHeadingInterpolation()
         .setReversed()
         .addPath(
            new BezierLine(
               new Pose(87.705, 84.772),

               new Pose(120.618, 69.786)
            )
         ).setTangentHeadingInterpolation()

         .build();

      follower.followPath(myPath.getPath(pathState));
      mechanisms.setTurretTicks(-98);
      mechanisms.pipelineSwitch(2);
      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 2 || pathState == 4 || pathState == 6) {
               mechanisms.stopIntake();
               while (!mechanisms.isTurretAligned()) {
                  mechanisms.update();
               }
               mechanisms.setTurretPower(0);
               mechanisms.startShortShooter();
               mechanisms.shoot();
               mechanisms.startIntake();
            }
            if (pathState == 0) {
               mechanisms.setTurretTicks(-235);
            }
            if (pathState == 2) {
               mechanisms.setTurretTicks(-230);
            }
            if (pathState == 4) {
               mechanisms.setTurretTicks(-255);
            }
            pathState++;
            follower.followPath(myPath.getPath(pathState));
         } else {
            if (pathState == 0) {
               mechanisms.startShortShooter();
            }
         }
         telemetry.update();
         follower.update();
      }
   }

}