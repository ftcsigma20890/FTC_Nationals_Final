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
public class AutoRedFar9 extends LinearOpMode {
    public static double intakeDriveSpeed = 1;
    Follower follower;
    int pathState = 0;
    Mechanisms mechanisms;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(89.000, 8.000, Math.toRadians(90)));
        follower.setMaxPower(1);
        mechanisms = new Mechanisms(hardwareMap, telemetry);

        PathChain myPath = follower
                .pathBuilder()

                // Path 1
                .addPath(
                        new BezierLine(
                                new Pose(89.000, 8.000),
                                new Pose(86.000, 16.000)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(70)
                )

                // Path 2
                .addPath(
                        new BezierCurve(
                                new Pose(86.000, 16.000),
                                new Pose(89.424, 37.909),
                                new Pose(128.339, 35.179)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(70),
                        Math.toRadians(0)
                )

                // Path 3
                .addPath(
                        new BezierLine(
                                new Pose(128.339, 35.179),
                                new Pose(85.994, 16.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                // Path 4
                .addPath(
                        new BezierCurve(
                                new Pose(85.994, 16.000),
                                new Pose(83.219, 64.637),
                                new Pose(128.570, 59.383)
                        )
                )
                .setLinearHeadingInterpolation(
                        Math.toRadians(70),
                        Math.toRadians(0)
                )

                // Path 5
                .addPath(
                        new BezierCurve(
                                new Pose(128.570, 59.383),
                                new Pose(95.416, 41.197),
                                new Pose(86.042, 16.192)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        follower.followPath(myPath.getPath(pathState));
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
                    mechanisms.startLongShooter();
                    mechanisms.shoot();
                    mechanisms.startIntake();
                }
                if (pathState == 1 || pathState == 3 || pathState == 5) {
                    mechanisms.stopIntake();
                }

                pathState++;
                if ( pathState == 1 || pathState == 2 || pathState == 3|| pathState == 4) {
                    follower.followPath(new PathChain(myPath.getPath(pathState)), 0.7  , true);

                } else {


                    follower.followPath(myPath.getPath(pathState));
                }
            } else {
                if (pathState == 0) {
                    mechanisms.startLongShooter();
                }
                if (pathState == 2) {
                    mechanisms.setTurretTicks(-190);
                } else {
                    mechanisms.setTurretTicks(-20);
                }
            }
            telemetry.update();
            follower.update();
        }
    }

}