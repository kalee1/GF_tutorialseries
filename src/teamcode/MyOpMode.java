package teamcode;



public class MyOpMode extends OpMode{


    @Override
    public void init() {

    }

    @Override
    public void loop() {
        RobotMovement.goToPosition(358/2, 358/2,0.5,Math.toRadians(90),0.3);
    }
}
