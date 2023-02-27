package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class PIDlinear extends OpMode {

    DcMotorEx Arm;

    public static PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0);
    double integral = 0;
    double error = 0;
    double currvel = 0;
    double derivate = 0;
    double deltaError = 0;


    ElapsedTime tempo = new ElapsedTime();

    private double lastError = 0;
    @Override
    public void init() {
        Arm = hardwareMap.get(DcMotorEx.class, "Arm");

        Arm.setDirection(DcMotor.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void loop() {
        if (gamepad1.right_bumper) {
            Arm.setPower(pidLinear(0.6));
        }
        else if (gamepad1.left_bumper) {
            Arm.setPower((pidLinear(-0.6)));
        }
        if (gamepad1.x){
            Arm.setPower((pidLinear(0.6)));
            Arm.setPower((pidLinear(-0.6)));
        }
        else{
            Arm.setPower(0);
        }
        telemetry.addData("O erro é de",error);
        telemetry.addData("O curvel é de",currvel);
        telemetry.addData("O deltaerro é de",deltaError);
        telemetry.addData("O ULTIMO erro é de", lastError);
    }

    public double pidLinear(double velocidade){


        currvel = Arm.getVelocity();

        error = velocidade - currvel;

        integral += error * tempo.seconds();

        deltaError = (error - lastError);

        derivate = deltaError / tempo.seconds();

        lastError = error;

        pidGains.p = error * pidCoeffs.p;
        pidGains.i = integral * pidCoeffs.i;
        pidGains.d = derivate * pidCoeffs.d;

        tempo.reset();

        double output = (pidGains.p + pidGains.i + pidGains.d + velocidade);
        return output;


    }

}

