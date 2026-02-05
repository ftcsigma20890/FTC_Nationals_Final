package org.firstinspires.ftc.teamcode.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class AutoBlueFar9 extends LinearOpMode {
    public static double intakeDriveSpeed = 1;
    Follower follower;
    int pathState = 0;
    Mechanisms mechanisms;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(55.000, 8.000, Math.toRadians(90)));
        follower.setMaxPower(1);
        mechanisms = new Mechanisms(hardwareMap, telemetry);

        PathChain myPath = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.000, 8.000),

                                new Pose(58.000, 16.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                .addPath(
                        new BezierCurve(
                                new Pose(58.000, 16.000),
                                new Pose(54.576, 37.909),
                                new Pose(15.661, 35.179)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(15.661, 35.179),
                                new Pose(58.006, 16.081)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierCurve(
                                new Pose(58.006, 16.081),
                                new Pose(60.781, 64.637),
                                new Pose(15.430, 59.383)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(15.430, 59.383),
                                new Pose(57.958, 16.192)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()

                .build();

        follower.followPath(myPath.getPath(pathState));
        mechanisms.pipelineSwitch(0);

        waitForStart();
        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                if (pathState == 0 || pathState == 2 || pathState == 4 || pathState == 6) {
                    mechanisms.stopIntake();
                    while (!mechanisms.isTurretAligned()) {
                        mechanisms.update();
                    }
                    mechanisms.setTurretPower(0);
                    mechanisms.startLongShooter();
                    mechanisms.shoot();
                    mechanisms.startIntake();
                }
                if (pathState == 1 || pathState == 3 || pathState == 5) {
                    mechanisms.stopIntake();
                }
                pathState++;
                if (pathState == 2) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), 0.8, true);

                }
                else {


                    follower.followPath(myPath.getPath(pathState));
                }
            } else {
                if (pathState == 0) {
                    mechanisms.startLongShooter();
                }
                if (pathState == 2) {
                    mechanisms.setTurretTicks(190);
                }
                else {
                    mechanisms.setTurretTicks(20);
                }
            }
            telemetry.update();
            follower.update();
        }
    }

}