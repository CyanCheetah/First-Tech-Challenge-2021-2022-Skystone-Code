package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
@Autonomous

public class CyanRocks extends LinearOpMode{

    private void PIDController(double[] pwr, DcMotor[] mtrs, int[] tarPos) {  //needs brake, runwithoutencoder, and reverse
        double r = getRuntime();
        int mtrIndex = 0;
        boolean robotIsRunning = true;
        for (DcMotor mtr:mtrs) {
            mtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtr.setPower(pwr[mtrIndex]);
            mtrIndex += 1;

        }


        //while (robotIsRunning) { //||mtrs[1].getCurrentPosition()  < tarPos[1] ||mtrs[2].getCurrentPosition()  < tarPos[2] ||mtrs[3].getCurrentPosition()  < tarPos[3]  ) {
        while (Math.abs(mtrs[0].getCurrentPosition())<Math.abs(tarPos[0])||Math.abs(mtrs[1].getCurrentPosition())<Math.abs(tarPos[1])||Math.abs(mtrs[2].getCurrentPosition())<Math.abs(tarPos[2])||Math.abs(mtrs[3].getCurrentPosition())<Math.abs(tarPos[3])) {


            telemetry.addLine("Robot is running");
            telemetry.update();
            idle();
        }

    }

    public void runOpMode() {
        //Initialization


        //Motor Initialization Stuff
        DcMotor fl= hardwareMap.get(DcMotor.class, "frontl");
        DcMotor fr= hardwareMap.get(DcMotor.class, "frontr");
        DcMotor bl= hardwareMap.get(DcMotor.class, "bottoml");
        DcMotor br= hardwareMap.get(DcMotor.class, "bottomr");
        CRServo s = hardwareMap.get(CRServo.class,"servo");
        DcMotor cm = hardwareMap.get(DcMotor.class, "CarouselMotor");
        DcMotor em= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        DcMotor[] dm = {fl,fr,bl,br};

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //Function Creation


        ElapsedTime runtime = new ElapsedTime();

        //Play is pressed


        waitForStart();
        for (DcMotor m:
             dm) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setTargetPosition(0);

        }
        while(fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()) {
            idle();
        }

        for (DcMotor m:dm
             ) {

            m.setPower(-0.5);
        }

        sleep(1000);


    }
}
