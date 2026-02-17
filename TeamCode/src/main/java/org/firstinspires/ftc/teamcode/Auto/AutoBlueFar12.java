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
public class AutoBlueFar12 extends LinearOpMode {
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
                .pathBuilder().addPath(
                        new BezierLine(
                                new Pose(55.000, 8.000),

                                new Pose(58.000, 16.000)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(
                        new BezierCurve(
                                new Pose(58.000, 16.000),
                                new Pose(54.576, 41.572),
                                new Pose(58.347, 33.889),
                                new Pose(15.661, 35.179)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(15.661, 35.179),
                                new Pose(58.006, 16.081)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(58.006, 16.081),

                                new Pose(13, 16.135)
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(13, 16.135),

                                new Pose(13, 8.205)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(13, 8.205),

                                new Pose(57.875, 15.266)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(57.875, 15.266),

                                new Pose(13, 15.730)
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(13, 15.730),

                                new Pose(13, 8.166)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(13, 8.166),

                                new Pose(57.879, 15.973)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(57.879, 15.973),

                                new Pose(46.454, 22.395)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        follower.followPath(myPath.getPath(pathState));
        mechanisms.pipelineSwitch(0);

        waitForStart();
        while (opModeIsActive()) {
            if (!follower.isBusy()) {
                if (pathState == 0 || pathState == 2 || pathState == 5 || pathState == 8) {
                    mechanisms.stopIntake();
                    while (!mechanisms.isTurretAligned()) {
                        mechanisms.update();
                    }
                    mechanisms.setTurretPower(0);
                    mechanisms.startLongShooter();
                    mechanisms.shoot();
                    mechanisms.startIntake();
                }
                pathState++;
                if (pathState == 2) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), 0.65, true);
                } else {
                    follower.followPath(myPath.getPath(pathState));
                }
            } else {
                if (pathState == 0) {
                    mechanisms.startLongShooter();
                }
                if (pathState == 2) {
                    mechanisms.setTurretTicks(190);

                }else if(pathState == 5){
                    mechanisms.setTurretTicks(312);
                }
                else if (pathState == myPath.length() - 1) {
                    mechanisms.setTurretTicks(0);
                } else {
                    mechanisms.setTurretTicks(20);
                }
            }
            telemetry.update();
            follower.update();
        }
    }

}