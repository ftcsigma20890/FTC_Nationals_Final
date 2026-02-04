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
public class Test extends LinearOpMode {
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

                  new Pose(35.543, 85.337)
               )
            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
            .build();
      follower.followPath(myPath.getPath(pathState));
//      mechanisms.setTurretTicks(125);
//      mechanisms.pipelineSwitch(0);

      waitForStart();
      while (opModeIsActive() && follower.isBusy()) {

         telemetry.update();
         follower.update();
      }
   }
}