package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoBlueStrafe extends LinearOpMode {
   public static double LATERAL_MULTIPLIER = 1.45;

   Mechanisms mechanisms;
   Follower follower;
   int pathState = 0;

   @Override
   public void runOpMode() throws InterruptedException {

      mechanisms = new Mechanisms(hardwareMap, telemetry);
      follower = Constants.createFollower(hardwareMap);
      follower.setStartingPose(new Pose(33.942, 136.357, Math.toRadians(180)));
      follower.setMaxPower(0.7);
      PathChain myPath =
         follower.pathBuilder().addPath(
               new BezierLine(
                  new Pose(33.942, 136.357),

                  new Pose(49.783, 84.535)
               )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .addPath(
               new BezierLine(
                  new Pose(49.783, 84.535),

                  new Pose(16.448, 84.033)
               )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .addPath(
               new BezierLine(
                  new Pose(16.448, 84.033),

                  new Pose(49.259, 84.390)
               )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
            .addPath(
               new BezierCurve(
                  new Pose(49.259, 84.390),
                  new Pose(52.870, 59.245),
                  new Pose(15.786, 59.766)
               )
            ).setConstantHeadingInterpolation(Math.toRadians(180))
            .addPath(
               new BezierLine(
                  new Pose(15.786, 59.766),

                  new Pose(49.331, 84.649)
               )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

            .build();
      follower.followPath(myPath.getPath(pathState));
      mechanisms.setTurretTicks(125);
      mechanisms.pipelineSwitch(0);

      waitForStart();
      while (opModeIsActive()) {
         if (!follower.isBusy()) {
            if (pathState == 0 || pathState == 2 || pathState == 4 || pathState == 6) {
               mechanisms.stopIntake();
//               while (!mechanisms.isTurretAligned()) {
//                  mechanisms.update();
//               }
               mechanisms.setTurretPower(0);
               mechanisms.startShortShooter();
               mechanisms.shoot();
               mechanisms.startIntake();
            }

            pathState++;
            if (pathState == 4) {
               follower.followPath(new PathChain(myPath.getPath(pathState)), 1, true);
            } else {
               follower.followPath(myPath.getPath(pathState));
            }
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
