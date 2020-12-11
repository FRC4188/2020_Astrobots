
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Parking2testing", group="Linear Opmode")

public class Parking2testing extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frMotor, flMotor, brMotor, blMotor, intakeMotor, magazineMotor, shooterMotor;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        runtime.reset();



        while(opModeIsActive()) {
            double time = runtime.seconds();

            if (time > 1 && time < 4){
                drivetrain(0.7,0,0.02);
            }
            if (time > 4 && time < 5){
                drivetrain(0,0,-0.28);
            }
            if (time > 5 && time < 6.2){
                drivetrain(0.7,0,0);
            }
            if (time > 6.2 && time < 8){
                drivetrain(-0.37,0,0.14);
            }

            if (time > 8 && time < 14){
                shooterMotor.setPower(0.65);
                drivetrain(0,0,0);
            }
            if (time > 14 && time < 15.5){
                magazineMotor.setPower(1.0);
            }
            if (time > 15.5 && time < 17){
                magazineMotor.setPower(0.0);
                drivetrain(0,0.1, 0);
            }
            if (time > 17 && time < 18.5){
                magazineMotor.setPower(1.0);
            }
            if (time > 18.5 && time < 20){
                magazineMotor.setPower(0.0);
                drivetrain(0, 0.1, 0);
                intakeMotor.setPower(-1.0);
            }
            if (time > 20 && time < 23){
                magazineMotor.setPower(1.0);
            }
            if (time > 23 && time < 24){
                intakeMotor.setPower(0.0);
                magazineMotor.setPower(0.0);
                shooterMotor.setPower(0.0);
                drivetrain(0.4,0,0);
            }
            if (time > 24){
                drivetrain(0,0,0);
            }
        }
    }

    private void initialize() {
        frMotor = hardwareMap.get(DcMotor.class, "frMotor");
        flMotor = hardwareMap.get(DcMotor.class, "flMotor");
        brMotor = hardwareMap.get(DcMotor.class, "brMotor");
        blMotor = hardwareMap.get(DcMotor.class, "blMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        magazineMotor = hardwareMap.get(DcMotor.class, "magazineMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        magazineMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime.reset();
    }

    private void drivetrain(double forward, double strafe, double rotation) {
        frMotor.setPower(forward - strafe - rotation);
        flMotor.setPower(forward + strafe + rotation);
        brMotor.setPower(forward + strafe - rotation);
        blMotor.setPower(forward - strafe + rotation);
    }

}
