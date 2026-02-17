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
public class AutoBlueFar extends LinearOpMode {
    public static double intakeDriveSpeed = 1;
    Follower follower;
    int pathState = 0;
    Mechanisms mechanisms;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(55.000, 8.000, Math.toRadians(180)));
        follower.setMaxPower(1);
        mechanisms = new Mechanisms(hardwareMap, telemetry);

        PathChain myPath = follower
                .pathBuilder()
                .addPath(
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

                                new Pose(57.695, 15.986)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(57.695, 15.986),

                                new Pose(10.712, 10.431)
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(10.712, 10.431),

                                 new Pose(57.890, 15.994)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(57.695, 15.986),

                                new Pose(10.712, 10.431)
                        )
                ).setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(10.712, 10.431),

                                 new Pose(57.890, 15.994)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .addPath(
                        new BezierLine(
                                new Pose(57.890, 15.994),

                                new Pose(42.511, 14.536)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        follower.followPath(myPath.getPath(pathState));

        mechanisms.pipelineSwitch(0);
        mechanisms.setTurretTicks(240);
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
                pathState++;
                if (pathState == 4 || pathState == 6) {
                    sleep(1000);
                }
                if (pathState == 1 || pathState == 2 || pathState == 4) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), 0.55, true);
                } else {
                    follower.followPath(myPath.getPath(pathState));
                }
            } else {
                if (pathState == 0) {
                    mechanisms.startLongShooter();
                }
                if (pathState == myPath.length() - 1) {
                    mechanisms.setTurretTicks(0);
                } else if (pathState == 0) {
                    mechanisms.setTurretTicks(240);
                } else if (pathState == 2) {
                    mechanisms.setTurretTicks(190);
                } else if (pathState == 4 || pathState == 6) {
                    mechanisms.setTurretTicks(332);
                } else {
                    mechanisms.setTurretTicks(20);
                }
            }
            telemetry.update();
            follower.update();
        }
    }

}