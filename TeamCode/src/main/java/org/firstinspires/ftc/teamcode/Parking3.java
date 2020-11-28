package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Parking3")
public class Parking3 extends LinearOpMode {

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;
    private CRServo arm;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.8956));
        blMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.8956));
        flMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.8956));
        brMotor.setTargetPosition((int) Math.round(-Constants.TICKS_PER_METER * 2.8956));

        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        setAllPower(-1.0);

        while(opModeIsActive() && (frMotor.isBusy() || flMotor.isBusy() || brMotor.isBusy() || blMotor.isBusy())) {
            idle();
        }
        setAllPower(0);

        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER *  0.8636));
        blMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER *  0.8636));
        flMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER *  0.8636));
        brMotor.setTargetPosition((int) Math.round(Constants.TICKS_PER_METER *  0.8636));
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        setAllPower(1.0);
        while(opModeIsActive() && (frMotor.isBusy() || flMotor.isBusy() || brMotor.isBusy() || blMotor.isBusy())) {
            idle();
        }
        setAllPower(0);
    }

    private void initialize() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        arm = hardwareMap.get(CRServo.class, "wobbleArm");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void setMotorMode(DcMotor.RunMode runMode) {
        frMotor.setMode(runMode);
        flMotor.setMode(runMode);
        brMotor.setMode(runMode);
        blMotor.setMode(runMode);
    }

    private void setAllPower(double power) {
        frMotor.setPower(power);
        flMotor.setPower(power);
        brMotor.setPower(power);
        blMotor.setPower(power);
    }
}
