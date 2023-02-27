package org.firstinspires.ftc.teamcode.AUTO;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.AUTO.AprilTagAutonomousInitDetectionExample;
import org.openftc.apriltag.AprilTagDetection;


@Autonomous(name = "AutonomoTeste1", group = "LinearOpMode")

public class autonomous extends LinearOpMode {
    public AprilTagAutonomousInitDetectionExample cam = new AprilTagAutonomousInitDetectionExample();
    public DcMotor motorEsquerdoF, motorEsquerdoT, motorDireitoF, motorDireitoT = null;
    BNO055IMU imu;


    @Override
    public void runOpMode() {
        motorEsquerdoF = hardwareMap.get(DcMotor.class, "LeftDriveUp");
        motorDireitoF = hardwareMap.get(DcMotor.class, "RightDriveUp");
        motorEsquerdoT = hardwareMap.get(DcMotor.class, "LeftDriveDown");
        motorDireitoT = hardwareMap.get(DcMotor.class, "RightDriveDown");


        motorEsquerdoF.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoF.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdoT.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoT.setDirection(DcMotor.Direction.FORWARD);


        resetRuntime();
        waitForStart();

        if (opModeIsActive()) {

            cam.runOpMode();


            while (opModeIsActive() && motorDireitoF.isBusy() && motorDireitoT.isBusy() && motorEsquerdoF.isBusy() && motorEsquerdoT.isBusy()) {
                idle();

            }
        }
    }
    }
