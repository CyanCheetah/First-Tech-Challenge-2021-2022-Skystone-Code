package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class CyanCheetahAuto extends LinearOpMode {
    private Blinker control_Hub__Upper_;
    private Blinker expansion_Hub__Lower_;
    private DcMotor bottoml;
    private DcMotor bottomr;
    private DcMotor frontl;
    private DcMotor frontr;
    private int ticksPerRevolution = 145  + 1/10;
    
    private double initialBatteryVoltage;
    
    
    public void setPowerAll(double frontrPower, double frontlPower, double bottomrPower, double bottomlPower) {
        
        frontr.setPower(frontrPower);
        frontl.setPower(frontlPower);
        bottomr.setPower(bottomrPower);
        bottoml.setPower(bottomlPower);
        
    }    
    
    public void targetPositionAll (int position) {
            frontr.setTargetPosition(position);
            frontl.setTargetPosition(-position);
            bottomr.setTargetPosition(position);
            bottoml.setTargetPosition(-position);
            
            
        
    }
    

    public void runOpMode() throws InterruptedException {
        while(!opModeIsActive()) {
            frontl= hardwareMap.get(DcMotor.class, "frontl");
            frontr= hardwareMap.get(DcMotor.class, "frontr");
            bottoml= hardwareMap.get(DcMotor.class, "bottoml");
            bottomr= hardwareMap.get(DcMotor.class, "bottomr");
            
            frontl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottoml.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottomr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            
            
            frontl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottoml.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        }
     //   waitForStart();

        if(opModeIsActive()) {
            
            
            setPowerAll(0.4,0.4,0.4,0.4);
            targetPositionAll(1 *(145 + 1/10));
            
            
            frontl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottoml.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bottomr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);
            
           
        }
        
        
    }
    // todo: write your code here
}
