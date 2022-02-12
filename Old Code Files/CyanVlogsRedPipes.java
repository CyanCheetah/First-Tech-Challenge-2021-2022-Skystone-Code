package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@Autonomous
public class CyanVlogsRedPipes extends LinearOpMode {




    private void PIDController(double[] pwr, DcMotor[] mtrs, int[] tarPos) {  //needs brake, runwithoutencoder, and reverse
        double r = getRuntime();
        int mtrIndex = 0;
        boolean robotIsRunning = true;
        for (DcMotor mtr:mtrs) {
            mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtr.setPower(pwr[mtrIndex]);
            mtrIndex += 1;

        }
        if (mtrs.length == 4) {

            while (Math.abs(mtrs[0].getCurrentPosition())<Math.abs(tarPos[0])||Math.abs(mtrs[1].getCurrentPosition())<Math.abs(tarPos[1])||Math.abs(mtrs[2].getCurrentPosition())<Math.abs(tarPos[2])||Math.abs(mtrs[3].getCurrentPosition())<Math.abs(tarPos[3])) {
                mtrIndex = 0;
            /*for (DcMotor mtr:mtrs) {
                if (mtr.getCurrentPosition() >= tarPos[mtrIndex]) {
                    robotIsRunning = false;
                }
            }*/



                telemetry.addLine("Robot is running");
                telemetry.update();
                idle();
            }
        }

        //mtr.wait

        if (mtrs.length == 1) {
            while (Math.abs(mtrs[0].getCurrentPosition())<Math.abs(tarPos[0])) {
                telemetry.addLine("Robot is running");
                telemetry.update();
                idle();
            }
        }
        for (DcMotor mtr :mtrs) {
            mtr.setPower(0);
        }
    }

    OpenCvCamera webcam;
    // OpenCvCamera webcam2;
    TripleRectPipeline pipeline;
    // SamplePipeline cyanline;
    private DcMotor bottoml;
    private DcMotor bottomr;
    private DcMotor frontl;
    private DcMotor frontr;
    private  DcMotor ElevatorMotor;
    private CRServo  servo;
    private int ticksPerRevolution = 145  + 1/10;

    int a;
    int b;
    int c;

    public void sleepWhileBusy() {
        while (frontr.isBusy() || frontl.isBusy() || bottoml.isBusy() || bottomr.isBusy()) {


        }
    }



    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "OpenCvCamera"), cameraMonitorViewId);
        // webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "OpenCvCamera"), cameraMonitorViewId);

        pipeline = new TripleRectPipeline();
        webcam.setPipeline(pipeline);
        frontl= hardwareMap.get(DcMotor.class, "frontl");
        frontr= hardwareMap.get(DcMotor.class, "frontr");
        bottoml= hardwareMap.get(DcMotor.class, "bottoml");
        bottomr= hardwareMap.get(DcMotor.class, "bottomr");
        ElevatorMotor= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        servo = hardwareMap.get(CRServo.class, "servo");

        DcMotor[] driveMotors = {frontl,frontr, bottoml, bottomr};
        if(!opModeIsActive()) {

            frontl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottoml.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottomr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





            frontl.setDirection(DcMotorSimple.Direction.REVERSE);
            frontr.setDirection(DcMotorSimple.Direction.FORWARD);
            ElevatorMotor.setDirection(DcMotor.Direction.FORWARD);
            bottoml.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomr.setDirection(DcMotorSimple.Direction.FORWARD);

            frontl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottoml.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //1280 x 720 resolution, aka 720p
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        waitForStart();

        if (opModeIsActive()) {


            while (pipeline.getRegionA() == 0 && pipeline.getRegionB() == 0 && pipeline.getRegionC() == 0)  {

                sleep(1000);
                telemetry.addData("Box A", pipeline.getRegionA());
                telemetry.addData("Box B", pipeline.getRegionB());
                telemetry.addData("Box C", pipeline.getRegionC());



                a = pipeline.getRegionA();
                b = pipeline.getRegionB();
                c = pipeline.getRegionC();
            }
            //Get to hub

            //Backwards
            PIDController(new double[] {0.25,0.25,0.25,0.25}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {550,550,550,550});
            //strafe
            PIDController(new double[] {0.25,-0.25,-0.25,0.25}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {30,-30,-30,30});
            //Turn in place 90 Degrees
            PIDController(new double[] {-0.25,0.25,-0.25,0.25}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {-300,300,-300,300});
            //Square Up
            frontl.setPower(-0.25);
            frontr.setPower(-0.25);
            bottoml.setPower(-0.25);
            bottomr.setPower(-0.25);
            sleep(4000);
            for (DcMotor mtr:
                    driveMotors) {
                mtr.setPower(0);
            }

            //Head to hub
            PIDController(new double[] {0.25,0.25,0.25,0.25}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {350, 350, 350, 350});




            sleep(200);

            sleep(200);
            //PIDController(new double[] {0.25,0.25,0.25,0.25}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {60, 60, 60, 60});
            if ( a < b && a < c ) {
                telemetry.addLine("The ARTIFACT is in BOX A");

                PIDController(new double[] {1}, new DcMotor[] {ElevatorMotor}, new int[] {100});
                // sleep(500);
                //  servo.setPower(-0.5);
                //   sleep(350);
                //   servo.setPower(0);

            }

            if(b < a && b < c) {
                telemetry.addLine("The ARTIFACT is in BOX B");

                PIDController(new double[] {1}, new DcMotor[] {ElevatorMotor}, new int[] {3500});
                //      sleep(500);
                //       servo.setPower(-0.5);
                //       sleep(350);
                //     servo.setPower(0);

            }
            if (c<a && c<b) {
                telemetry.addLine("The ARTIFACT is in BOX C");


                PIDController(new double[]{1}, new DcMotor[]{ElevatorMotor}, new int[]{6000});
            }

            //Arm the box
            sleep(500);
            servo.setPower(-0.5);
            sleep(350);
            servo.setPower(0);
            //Go forward
            PIDController(new double[] {0.25,0.25,0.25,0.25}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {75, 75, 75, 75});
            //deploy the box
            servo.setPower(-0.5);
            sleep(50);

            //go back to wall
            PIDController(new double[] {-0.25,-0.25,-0.25,-0.25}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {-400,-400,-400,-400});



        }
        // sleep(500);//  if things break, uncomment
        stop();
    }

    public class TripleRectPipeline extends OpenCvPipeline {
        //Colors Defined
        private final Scalar RED = new Scalar(255,0,0);
        private final Scalar BLUE = new Scalar(0, 0, 255);
        private final Scalar GREEN = new Scalar(0, 255, 0);

        //I just want someone to tickle my balls<> -Noah
        private static final int THRESHOLD = 150;

        /*

        [A]    [B]     [C]
         */
        Point aTopLeft = new Point(70, 260);
        Point aBottomRight = new Point(220, 410);

        Point bTopLeft = new Point(570, 270);
        Point bBottomRight = new Point(700, 390);

        Point cTopLeft = new Point(1050, 290);
        Point cBottomRight = new Point(1180, 415);

        //YCrCb  --> Y = Brightness, Cr = Red - Luma, Cb = Blue - Luma
        Mat regionA_Cb;
        Mat regionB_Cb;
        Mat regionC_Cb;

        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private volatile int averageA;
        private volatile int averageB;
        private volatile int averageC;
        //private string type = "N/a";
        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            regionA_Cb = Cb.submat(new Rect(aTopLeft, aBottomRight));
            regionB_Cb = Cb.submat(new Rect(bTopLeft, bBottomRight));
            regionC_Cb = Cb.submat(new Rect(cTopLeft, cBottomRight));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            averageA = (int) Core.mean(regionA_Cb).val[0];
            averageB = (int) Core.mean(regionB_Cb).val[0];
            averageC = (int) Core.mean(regionC_Cb).val[0];

            //A
            Imgproc.rectangle(input, aTopLeft, aBottomRight, RED,2);
            //B
            Imgproc.rectangle(input, bTopLeft, bBottomRight, BLUE,2);
            //C
            Imgproc.rectangle(input, cTopLeft, cBottomRight, GREEN,2);




            return input;
        }


        public int getAverage() {
            return averageA;
        }

        public int getRegionA() {
            return averageA;
        }

        public int getRegionB() { return averageB; }

        public int getRegionC() { return averageC; }

        public int getReg;

    }
}
