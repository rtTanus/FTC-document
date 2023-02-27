package org.firstinspires.ftc.teamcode.Testes;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Encoder2", group="OpMode")
public class Linear2 extends OpMode{
    public DcMotor Arm;


    @Override
    public void init() {

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setDirection(DcMotor.Direction.FORWARD);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    public void loop() {
        boolean poderCima = gamepad2.right_bumper;
        boolean poderBaixo = gamepad2.left_bumper;
        boolean poderImovel = gamepad2.x;
        double pow;

        if (poderCima) {
            pow = -0.9;
            Arm.setPower(pow);
        }

        if (poderBaixo) {
            pow = 0.5;
            Arm.setPower(pow);
        }
        if (poderImovel) {
            pow = -0.1;
            Arm.setPower(pow);
        }
        else {
            pow = 0;
            Arm.setPower(pow);
        }
        if(gamepad2.a){
            Arm.setTargetPosition(-300);
            Arm.setPower(1);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(gamepad2.x){
            Arm.setTargetPosition(-900);
            Arm.setPower(1);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(gamepad2.y){
            Arm.setTargetPosition(-1000);
            Arm.setPower(1);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.b){
            Arm.setPower(0);
            Arm.setTargetPosition(0);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        telemetry.addData("A potencia do motor do sistema linear é de", pow);
        telemetry.addData("Posição do enconder", Arm.getCurrentPosition());

    }

}
