package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    private DcMotor ElevatorMotor;
    private CRServo servo;
    private CRServo servo1;
    public static double DISTANCE = 30; // in
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorMotor= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        servo = hardwareMap.get(CRServo.class, "servo");
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
/*
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
         */
                //.turn(Math.toRadians(60))
        Trajectory turn = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(65,.2,Math.toRadians(-46.7)))
                .addTemporalMarker(.5, () -> {
                    //specify time here
                    // elevator set to .4 power
                    ElevatorMotor.getCurrentPosition();
                    ElevatorMotor.setTargetPosition(ElevatorMotor.getCurrentPosition()+2880);
                    ElevatorMotor.setPower(.7);
                })
                .addTemporalMarker(3, () -> {
                    //specify time here
                    // elevator set to .4 power
                    ElevatorMotor.getCurrentPosition();
                    ElevatorMotor.setTargetPosition(369);
                    ElevatorMotor.setPower(-.5);
                })
                .addTemporalMarker(3, () -> {
                    //specify time here
                    // elevator set to .4 power
                    servo.setPower(-1);
                    servo1.setPower(1);

                })
                .build();

        Trajectory back = drive.trajectoryBuilder(new Pose2d())
                .back(15.5)
                .addTemporalMarker(0, () -> {
                    //specify time here
                    // elevator set to .4 power
                    servo.setPower(0);
                    servo1.setPower(0);

                })
                .build();




        Trajectory stuff = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(.1,.1,Math.toRadians(124.5)))



                .build();
        Trajectory front = drive.trajectoryBuilder((new Pose2d()))
                .forward(18)
                .build();
        Trajectory cyan = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    //specify time here
                    // Turn off motor
                    servo.setPower(1);
                    servo1.setPower(-1);
                })
                .forward(4)

                .build();
        Trajectory cyan2 = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    //specify time here
                    // Turn off motor
                    ElevatorMotor.getCurrentPosition();
                    ElevatorMotor.setTargetPosition(ElevatorMotor.getCurrentPosition()+2400);
                    ElevatorMotor.setPower(.7);
                    servo.setPower(0);
                    servo1.setPower(0);
                })

                .back(4)



                .build();
        Trajectory stuff2 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-21,5,Math.toRadians(-131)))

                .build();
        Trajectory stuff3 = drive.trajectoryBuilder(new Pose2d())

                .forward(15)
                .addTemporalMarker(.8, () -> {
                    //specify time here
                    // Turn off motor
                    ElevatorMotor.getCurrentPosition();
                    ElevatorMotor.setTargetPosition(369);
                    ElevatorMotor.setPower(-.5);
                })
                .addTemporalMarker(1.1, () -> {
                    //specify time here
                    // elevator set to .4 power
                    servo.setPower(-1);
                    servo1.setPower(1);

                })

                .build();
        Trajectory stuff4 = drive.trajectoryBuilder(new Pose2d())
                .back(15.5)

                .build();
        Trajectory stuff5 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(.1,.1,Math.toRadians(124.5)))


                .addTemporalMarker(0, () -> {
                    //specify time here
                    // Turn off motor
                    ElevatorMotor.setPower(.5);
                })
                .addTemporalMarker(0.8, () -> {
                    //specify time here
                    // Turn off motor
                    ElevatorMotor.setPower(.01);
                })
                .build();
        Trajectory front2 = drive.trajectoryBuilder((new Pose2d()))
                .forward(18)
                .build();
        Trajectory cyan3 = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    //specify time here
                    // Turn off motor
                    servo.setPower(1);
                    servo1.setPower(-1);
                })
                .forward(4)

                .build();
        Trajectory cyan4 = drive.trajectoryBuilder(new Pose2d())
                .addTemporalMarker(0, () -> {
                    //specify time here
                    // Turn off motor
                    ElevatorMotor.setPower(.8);
                    servo.setPower(0);
                    servo1.setPower(0);
                })
                .addTemporalMarker(1.3, () -> {
                    //specify time here
                    // Turn off motor
                    ElevatorMotor.setPower(0);

                })
                .back(4)
                .build();
















        //drive.followTrajectory(trajectory);
        drive.followTrajectory(turn);
        drive.followTrajectory(back);

        drive.followTrajectory(stuff);
        drive.followTrajectory(front);

        drive.followTrajectory(cyan);
        drive.followTrajectory(cyan2);

        drive.followTrajectory(stuff2);
        drive.followTrajectory(stuff3);

        drive.followTrajectory(stuff4);

        drive.followTrajectory(stuff5);

        drive.followTrajectory(front2);
        drive.followTrajectory(cyan3);

        drive.followTrajectory(cyan4);



        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}

//notes from cyan: this is very temporal
