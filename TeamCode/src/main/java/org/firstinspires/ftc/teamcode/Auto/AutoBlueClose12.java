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
public class AutoBlueClose12 extends LinearOpMode {
   public static double intakeDriveSpeed = 1;
   Follower follower;
   int pathState = 0;
   Mechanisms mechanisms;

   @Override
   public void runOpMode() {

      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(34.298, 135.777, Math.toRadians(180)));
      follower.setMaxPower(1);
      mechanisms = new Mechanisms(hardwareMap, telemetry);

      PathChain myPath = follower
         .pathBuilder()
         // Path1
         .addPath(
            new BezierLine(
               new Pose(34.340, 135.955),
               new Pose(54.000, 83.500)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))

         // Path2
         .addPath(
            new BezierLine(
               new Pose(50.000, 83.500),
               new Pose(20.000, 83.500)
            )
         )
         .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

         // Path4
         .addPath(
            new BezierLine(
               new Pose(20.000, 83.500),
               new Pose(50.000, 83.500)
            )
         )
         .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(245))
         .addPath(
            new BezierCurve(
               new Pose(50.000, 83.500),
               new Pose(53.850, 57.003),
               new Pose(20.000, 60.200)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(180))
         .addPath(
            new BezierLine(
               new Pose(20.000, 60.200),

               new Pose(50.571, 84.022)
            )
         )
         .setTangentHeadingInterpolation()
         .setReversed()
         .addPath(
            new BezierCurve(
               new Pose(50.571, 84.022),
               new Pose(55.990, 36.312),
               new Pose(39.946, 34.312),
               new Pose(12.886, 35.933)
            )
         ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(180))
         .addPath(
            new BezierLine(
               new Pose(12.886, 35.933),

               new Pose(51.752, 84.565)
            )
         ).setTangentHeadingInterpolation()
         .setReversed()

         .build();

      follower.followPath(new PathChain(myPath.getPath(pathState)), 0.7, true);
      mechanisms.setTurretTicks(170);
      mechanisms.pipelineSwitch(0);

      waitForStart();
      try {
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
                  mechanisms.setTurretTicks(533);
               }
               if (pathState == 2) {
                  mechanisms.setTurretTicks(383);
               }
               if (pathState == 4) {
                  mechanisms.setTurretTicks(425);
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
      } finally {
         mechanisms.setTurretTicks(0);
      }
   }

}
