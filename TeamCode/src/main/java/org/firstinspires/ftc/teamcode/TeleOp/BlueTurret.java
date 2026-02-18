package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class BlueTurret extends LinearOpMode {

    Turret turret;

    double targetX = 17;
    double targetY = 132;

    // Fixed point A
    double Ax = 8;
    double Ay = 139;

    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(32, 135, Math.toRadians(90)));
        follower.update();

        turret = new Turret(hardwareMap);
        follower.startTeleOpDrive(true);

        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            follower.setTeleOpDrive(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x
            ,true);

            Pose pose = follower.getPose();
            double Bx = pose.getX();
            double By = pose.getY();

            // C follows robot X but fixed Y = 144
            double Cx;
            Cx = Bx;
            double Cy = 144;

            double theta = calculateAngle(Ax, Ay, Bx, By, Cx, Cy);
            double turretDegrees = getTurretDegrees(theta, getNormalizedHeading(Math.toDegrees(follower.getHeading())));

            telemetry.addData("Heading:", getNormalizedHeading(Math.toDegrees(follower.getHeading())));
            telemetry.addData("Total Heading:", Math.toDegrees(follower.getTotalHeading()));
            telemetry.addData("Turret ticks:", turret.getTicks());
            telemetry.addData("Turret Angle: ", turret.getTurretDegrees());
            telemetry.addData("Theta Angle:", theta);
            telemetry.addData("Turret Degree: ", turretDegrees);
            telemetry.addData("Distance", getDistance(pose));   
            telemetry.update();
            turret.setTurretTicks(turret.angleToTicks(turretDegrees));
        }
    }

    public double calculateAngle(double Ax, double Ay,
                                 double Bx, double By,
                                 double Cx, double Cy) {

        double BAx = Ax - Bx;
        double BAy = Ay - By;
        double BCx = Cx - Bx;
        double BCy = Cy - By;

        double dot = BAx * BCx + BAy * BCy;
        double magBA = Math.sqrt(BAx * BAx + BAy * BAy);
        double magBC = Math.sqrt(BCx * BCx + BCy * BCy);

        if (magBA == 0 || magBC == 0) return 0;

        double cosTheta = dot / (magBA * magBC);
        cosTheta = Math.max(-1, Math.min(1, cosTheta));

        return Math.toDegrees(Math.acos(cosTheta));
    }

    public double getNormalizedHeading(double heading) {
        if (heading > 0) return heading;
        return 180 + (180 - Math.abs(heading));
    }

    public double getTurretDegrees(double theta, double headingDegrees) {
        double turretDegrees = 180 + 90 + theta - headingDegrees + 10;
        if (turretDegrees > 360) {
            turretDegrees %= 360;
        } else if (turretDegrees < 0) {
            turretDegrees = 360 - turretDegrees;
        }
        return turretDegrees;
    }

    public double getDistance(Pose pose){
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();

        return Math.sqrt(dx * dx + dy * dy);

    }

}
