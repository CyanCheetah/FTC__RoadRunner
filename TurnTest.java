package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    private DcMotor ElevatorMotor;

    public static double ANGLE = 105; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorMotor= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElevatorMotor.getCurrentPosition();
        ElevatorMotor.setTargetPosition(ElevatorMotor.getCurrentPosition()+2900);
        ElevatorMotor.setPower(.6);
        ElevatorMotor.getCurrentPosition();
        ElevatorMotor.setTargetPosition(ElevatorMotor.getCurrentPosition()-400);
        ElevatorMotor.setPower(-.1);



        waitForStart();

        if (isStopRequested()) return;

    }
}
