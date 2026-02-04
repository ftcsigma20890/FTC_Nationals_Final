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
public class AutoRedCloseall12 extends LinearOpMode {
   public static double intakeDriveSpeed = 1;
   Follower follower;
   int pathState = 0;
   Mechanisms mechanisms;

   @Override
   public void runOpMode() throws InterruptedException {

      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(122.543, 123.543, Math.toRadians(35)));
      follower.setMaxPower(1);
      mechanisms = new Mechanisms(hardwareMap, telemetry);

      PathChain myPath = follower
         .pathBuilder()
         .addPath(
            new BezierLine(
               new Pose(122.543, 123.543),

               new Pose(88.000, 84.000)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(35))
         //2
         .addPath(
            new BezierCurve(
               new Pose(88.000, 84.000),
               new Pose(80.191, 52.869),
               new Pose(133.513, 60.267)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
            //3
         .addPath(
            new BezierCurve(
               new Pose(133.513, 60.267),
               new Pose(85.989, 55.464),
               new Pose(137.630, 70.900)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
         //4
         .addPath(
            new BezierCurve(
               new Pose(137.630, 70.900),
               new Pose(102.412, 61.942),
               new Pose(87.630, 84.660)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
         //5
         .addPath(
            new BezierLine(
               new Pose(87.630, 84.660),

               new Pose(128.808, 84.705)
            )
         ).setTangentHeadingInterpolation()
         //6
         .addPath(
            new BezierLine(
               new Pose(128.808, 84.705),

               new Pose(87.128, 84.630)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
         //7
         .addPath(
            new BezierCurve(
               new Pose(87.128, 84.630),
               new Pose(78.430, 31.504),
               new Pose(127.877, 34.958)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

         //8
         .addPath(
            new BezierLine(
               new Pose(127.877, 34.958),

               new Pose(87.320, 84.967)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
         //9
         .addPath(
            new BezierLine(
               new Pose(87.320, 84.967),

               new Pose(121.841, 70.850)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))

         .build();

      follower.followPath(myPath.getPath(pathState));
      mechanisms.pipelineSwitch(2);
      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 2 || pathState == 5 || pathState == 7) {
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
            if (pathState == 5) {
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