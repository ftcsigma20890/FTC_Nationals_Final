package org.firstinspires.ftc.teamcode.Auto.BlueAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoBlueClose extends LinearOpMode {
   public static double intakeDriveSpeed = 1;
   Follower follower;
   int pathState = 0;
//   Mechanisms mechanisms;

   @Override
   public void runOpMode() throws InterruptedException {

      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(34.298, 135.777, Math.toRadians(180)));
      follower.setMaxPower(1);
//      mechanisms = new Mechanisms(hardwareMap, telemetry);

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
         .setConstantHeadingInterpolation(Math.toRadians(180))

         // Path5
         .addPath(
            new BezierCurve(
               new Pose(50.000, 83.500),
               new Pose(54.251, 57.404),
               new Pose(20.000, 60.200)
            )
         )
         .setConstantHeadingInterpolation(Math.toRadians(180))

         // Path7
         .addPath(
            new BezierLine(
               new Pose(20.000, 60.200),
               new Pose(50.000, 82.500)
            )
         ).setConstantHeadingInterpolation(Math.toRadians(180))
         .build();

      follower.followPath(myPath.getPath(pathState));
//      mechanisms.setTurretTicks(-98);
//      mechanisms.pipelineSwitch(0);

      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
//            if (pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9) {
//               mechanisms.startShortShooter();
//               mechanisms.shoot();
//               mechanisms.startIntake();
//            }
            pathState++;
            follower.followPath(myPath.getPath(pathState));
         } else {
//            if (pathState == 0 || pathState == 3 || pathState == 6 || pathState == 9) {
//               mechanisms.startShortShooter();
//            }
         }
         telemetry.update();
//         mechanisms.update();
         follower.update();
      }
   }

}