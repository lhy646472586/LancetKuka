package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import javax.inject.Inject;

import com.kuka.generated.ioAccess.SafeDataIOGroup;
import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;

/**
 * This example activates a SmartServoLIN motion in Cartesian impedance control mode, sends a sequence of Cartesian set
 * points, describing a sine function in z-direction and modifies compliance parameters during the motion.
 * 
 */
public class motion1 extends RoboticsAPIApplication
{
	@Inject
	private TCPServerSendDataApplication _vi;
    private LBR _lbr;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private ISmartServoLINRuntime _theSmartServoLINRuntime = null;
    public int s=1;
    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 2.2;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { -44.99, 2.67, 120.35 };

    private static final int NUM_RUNS = 60000;
    private static final double AMPLITUDE = 70;
    private static final double FREQENCY = 0.6;
    private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };
    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
   
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);
        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool_ForPlanetest", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    public void moveToInitialPosition()
    {
    	JointPosition jReady =_lbr.getCurrentJointPosition();
        _lbr.move(ptp(jReady.get(JointEnum.J1), jReady.get(JointEnum.J2), jReady.get(JointEnum.J3), jReady.get(JointEnum.J4), jReady.get(JointEnum.J5),
        		jReady.get(JointEnum.J6), jReady.get(JointEnum.J7)).setJointVelocityRel(0.1));
        System.out.println("getCurrentJointPosition");
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        boolean bReady=false;
        while (bReady==false){
        	 try{
                 if (!ServoMotion.validateForImpedanceMode(_lbr))
                 {
                 	System.out.println("2wwww23");
                     getLogger()
                             .info("Validation of torque model failed - correct your mass property settings");
                     getLogger()
                             .info("Servo motion will be available for position controlled mode only, until validation is performed");
                 }
                 bReady=true;
             }
             catch (Exception e)
             {
             	ThreadUtil.milliSleep(1000);
             	System.out.println("ttt");
   
             }
        }
       
        
//        if (!ServoMotion.validateForImpedanceMode(_lbr))
//        {
//        	System.out.println("2wwww23");
//            getLogger()
//                    .info("Validation of torque model failed - correct your mass property settings");
//            getLogger()
//                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
//        }
//        else{
//        	System.out.println("2wwww3");
//        }
		 System.out.println("233");
    }

    @Override
    public void run()
    {
    	
//    	//ForTest
//    	////////////////////////////////////////////
//    	 Frame initialPosition = _lbr.getCurrentCartesianPosition(_lbr
//                 .getFlange());
//    	 Frame initialPosition1 = _lbr.getCurrentCartesianPosition(_lbr
//                 .getFlange());
//    	 System.out.println("initialPosition"+initialPosition);
//    	 
////    	 
//// 		Frame Ptest1= _lbr.getCurrentCartesianPosition(_lbr.getFlange());		
//// 		Frame Ptest2 = Ptest1.transform((Transformation.ofTranslation(0, 200, 0)));
//// 		System.out.println("Ptest21"+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad())); 
//// 		System.out.println("Ptest22"+" X:"+Ptest2.getX()+" b:"+Ptest2.getY()+" c:"+Ptest2.getZ()); 
//// 		
////// 		Ptest1= _lbr.getCurrentCartesianPosition(_lbr.getFlange());	
////// 		 Ptest2 = Ptest1.transform((Transformation.ofDeg(0, 0, 0, 0, 10, 0)));
////// 		System.out.println("Ptest23"+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad())); 
//// 	
//// 		Ptest1= _lbr.getCurrentCartesianPosition(_lbr.getFlange());	
////		 Ptest2 = Ptest1.transform((Transformation.ofDeg(0, 0, 0, 0, 5.72957795130, 0)));
////		System.out.println("Ptest24"+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad())); 
//
//		
//
////    	 while(true)
////    	 {
//    		 Frame destFrame = _lbr.getCurrentCartesianPosition(_lbr.getFlange());
//////    		 
//////    		 initialPosition.setX(initialPosition1.getX());
//////    		 initialPosition.setY(initialPosition1.getY());
//////    		 initialPosition.setZ(initialPosition1.getZ());
//////    		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
//////    		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
//////    		 initialPosition.setGammaRad(initialPosition1.getGammaRad());
//////    		 
//////    		 Frame Ptest2 = initialPosition.transform((Transformation.ofDeg(0, 0, 0, 0, -(Math.toDegrees(destFrame.getBetaRad()-initialPosition.getBetaRad()))*5.72957795130, 0)));
//////    		 System.out.println("err1: "+(-(Math.toDegrees(destFrame.getBetaRad()-initialPosition.getBetaRad())))+"Ptest2"+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad()));
//////    		 
//////    		 initialPosition.setX(initialPosition1.getX());
//////    		 initialPosition.setY(initialPosition1.getY());
//////    		 initialPosition.setZ(initialPosition1.getZ());
//////    		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
//////    		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
//////    		 initialPosition.setGammaRad(initialPosition1.getGammaRad());
//////    		 Ptest2 = initialPosition.transform((Transformation.ofDeg(0, 0, 0, 0, (Math.toDegrees(destFrame.getBetaRad()-initialPosition.getBetaRad()))*5.72957795130, 0)));
//////    		 System.out.println("err2: "+(Math.toDegrees(destFrame.getBetaRad()-initialPosition.getBetaRad()))+"Ptest22"+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad()));
//////    		 
//////    		 initialPosition.setX(initialPosition1.getX());
//////    		 initialPosition.setY(initialPosition1.getY());
//////    		 initialPosition.setZ(initialPosition1.getZ());
//////    		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
//////    		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
//////    		 initialPosition.setGammaRad(initialPosition1.getGammaRad());
//////    		 Ptest2 = initialPosition.transform((Transformation.ofDeg(0, 0, 0, 0, 5.72957795130, 0)));
//////    		 System.out.println("err3: "+(Math.toDegrees(destFrame.getBetaRad()-initialPosition.getBetaRad()))+"Ptest22"+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad()));
//////    		 
//////    		 initialPosition.setX(initialPosition1.getX());
//////    		 initialPosition.setY(initialPosition1.getY());
//////    		 initialPosition.setZ(initialPosition1.getZ());
//////    		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
//////    		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
//////    		 initialPosition.setGammaRad(initialPosition1.getGammaRad());
//////    		 Ptest2 = initialPosition.transform((Transformation.ofDeg(0, 0, 0, 0, -5.72957795130, 0)));
//////    		 System.out.println("err4: "+(Math.toDegrees(destFrame.getBetaRad()-initialPosition.getBetaRad()))+"Ptest22"+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad()));
////    		 
////    		 initialPosition.setX(initialPosition1.getX());
////    		 initialPosition.setY(initialPosition1.getY());
////    		 initialPosition.setZ(initialPosition1.getZ());
////    		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
////    		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
////    		 initialPosition.setGammaRad(initialPosition1.getGammaRad());
//////    		 System.out.println("1"+Math.toDegrees(initialPosition.getBetaRad()));
//////    		 System.out.println("2"+Math.toDegrees(destFrame.getBetaRad()));
////    		 System.out.println("1"+initialPosition.getX());
////    		 System.out.println("2"+destFrame.getX());
////    		 
//////    		 Frame Ptest2 = initialPosition.transform((Transformation.ofDeg(destFrame.getX()-initialPosition.getX(), 0, destFrame.getY()-initialPosition.getY(), 0, 0, 0)));
////    		 Frame cmdPos = initialPosition.transform(Transformation.ofTranslation(-20,31.63,-13.09));
////    		 System.out.println("err1: "+" x:"+Ptest2.getX()+" y:"+Ptest2.getY()+" z:"+Ptest2.getZ());
////    		 
////    		 
//    		 initialPosition.setX(initialPosition1.getX());
//    		 initialPosition.setY(initialPosition1.getY());
//    		 initialPosition.setZ(initialPosition1.getZ());
//    		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
//    		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
//    		 initialPosition.setGammaRad(initialPosition1.getGammaRad());
////    		 System.out.println("1"+Math.toDegrees(initialPosition.getBetaRad()));
////    		 System.out.println("2"+Math.toDegrees(destFrame.getBetaRad()));
////    		 System.out.println("1"+initialPosition.getX());
////    		 System.out.println("2"+destFrame.getX());
//    		 
////    		 Frame Ptest2 = initialPosition.transform((Transformation.ofDeg(destFrame.getX()-initialPosition.getX(), 0, destFrame.getY()-initialPosition.getY(), 0, 0, 0)));
//    	     Frame Ptest2 = initialPosition.transform(Transformation.ofTranslation(-40,0,0));
//    	     
//    		 System.out.println("err2: "+" x:"+Ptest2.getX()+" y:"+Ptest2.getY()+" z:"+Ptest2.getZ());
//    		 
//    		 
////    		 System.out.println(Math.toDegrees(Math.toRadians(1)));
////    		 Frame Ptest3 = initialPosition.transform((Transformation.ofDeg(0, 0, 0, 0, 5.72957795130, 0)));
////    		 System.out.println("err2: "+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad()));
//    		 ThreadUtil.milliSleep(1000);
//    		 
//				Frame cmdPos2 = _lbr.getCurrentCartesianPosition(_lbr.getFlange());
//				cmdPos2.setX(-659.13);
//				cmdPos2.setY(48.82);
//				cmdPos2.setZ(631.26);
//				cmdPos2.setAlphaRad(0);
//				cmdPos2.setBetaRad(Math.toRadians(0));
//				cmdPos2.setGammaRad(0);
//			    Frame cmdPos=_lbr.getCurrentCartesianPosition(_lbr.getFlange(), cmdPos2);
//				 System.out.println("err2: "+" x:"+cmdPos.getX()+" y:"+cmdPos.getY()+" z:"+cmdPos.getZ());
				
//    	 }
    	///////////////////////////////////////////
    	
    	while (true)
    	{
    		
    	
//    	initialize();
        getLogger().info("Move to start position.");
        moveToInitialPosition();

        // Initialize Cartesian impedance control mode
        final CartesianImpedanceControlMode cartImp = createCartImp();

        getLogger()
                .info("Sample Application - SmartServoLIN motion in cartesian impedance control mode");
        runSmartServoLINMotion(cartImp);
        ThreadUtil.milliSleep(2000);
        
    	}
        
        
//        // Return to initial position
//        moveToInitialPosition();
//
//        // Initialize position control mode
//        final PositionControlMode positionCtrlMode = new PositionControlMode();
//
//        getLogger()
//                .info("Sample Application -  SmartServoLIN motion in position control mode");
//        runSmartServoLINMotion(positionCtrlMode);
    }

    /**
     * Creates a smartServoLIN Motion with the given control mode and moves around.
     * 
     * @param controlMode
     *            the control mode which shall be used
     * @see {@link CartesianImpedanceControlMode}
     */

    protected void runSmartServoLINMotion(final IMotionControlMode controlMode)
    {
        AbstractFrame initialPosition = _lbr.getCurrentCartesianPosition(_lbr
                .getFlange());

        // Create a new smart servo linear motion
        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

        aSmartServoLINMotion.setMaxTranslationVelocity(MAX_TRANSLATION_VELOCITY);
        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServoLIN in " + controlMode);
        _lbr.moveAsync(aSmartServoLINMotion.setMode(controlMode));

        getLogger().info("Get the runtime of the SmartServoLIN motion");
        _theSmartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        StatisticTimer timing = new StatisticTimer();

        getLogger().info("Do sine movement");
        timing = startSineMovement(_theSmartServoLINRuntime, timing, controlMode);

        ThreadUtil.milliSleep(500);

        // Print statistic timing
        getLogger().info(
                getClass().getName() + _theSmartServoLINRuntime.toString());

        getLogger().info("Stop the SmartServoLIN motion");
        _theSmartServoLINRuntime.stopMotion();

        // Statistic Timing of sine movement loop
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger()
                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger()
                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
        }
    }

    /**
     * Create the CartesianImpedanceControlMode class for motion parameterization.
     * 
     * @see {@link CartesianImpedanceControlMode}
     * @return the created control mode
     */
    private CartesianImpedanceControlMode createCartImp()
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);
        cartImp.parametrize(CartDOF.C).setStiffness(5.0);
        cartImp.parametrize(CartDOF.TRANSL).setStiffness(5000.0);
        cartImp.parametrize(CartDOF.Z).setStiffness(50.0);
        cartImp.parametrize(CartDOF.Y).setStiffness(50.0);
        return cartImp;
    }

    private StatisticTimer startSineMovement(
            ISmartServoLINRuntime theSmartServoLINRuntime,
            StatisticTimer timing, IMotionControlMode mode)
    {
        Frame aFrame = theSmartServoLINRuntime
                .getCurrentCartesianDestination(_lbr.getFlange());

        try
        {
            getLogger().info("Start SmartServoLIN sine movement");
            double omega = FREQENCY * 2 * Math.PI * 1e-9;

            long startTimeStamp = System.nanoTime();
           
            int i;
       	 Frame initialPosition = _lbr.getCurrentCartesianPosition(_lbr
                 .getFlange());
    	 Frame initialPosition1 = _lbr.getCurrentCartesianPosition(_lbr
                 .getFlange());
    
//    	 System.out.println("initialPosition"+initialPosition);
            
            double i1=0;
            double nAix4=0;
            //for (i = 0; i < NUM_RUNS; ++i)
            boolean bOnlyForPlane=false;
    	 while(Math.abs(nAix4)<115 && bOnlyForPlane==false )
            {
//    		 System.out.println(i1);
    		 
//    		 bOnlyForPlane=_vi.MotionType();
//    		 
    		 if(bOnlyForPlane==true){
    			 System.out.println("11");
    		 }
    
    		 
    		 JointPosition jReady =_lbr.getCurrentJointPosition();
    		 nAix4=Math.toDegrees(jReady.get(JointEnum.J1));
            	
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system

                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                theSmartServoLINRuntime.updateWithRealtimeSystem();

                double curTime = System.nanoTime() - startTimeStamp;
                double sinArgument = omega * curTime;

                // Compute the sine function
                Frame destFrame = _lbr.getCurrentCartesianPosition(_lbr.getFlange());
                

       		 initialPosition.setX(initialPosition1.getX());
       		 initialPosition.setY(initialPosition1.getY());
       		 initialPosition.setZ(initialPosition1.getZ());
       		 initialPosition.setAlphaRad(initialPosition1.getAlphaRad());
       		 initialPosition.setBetaRad(initialPosition1.getBetaRad());
       		 initialPosition.setGammaRad(initialPosition1.getGammaRad());
//       		 System.out.println("1"+Math.toDegrees(initialPosition.getBetaRad()));
//       		 System.out.println("2"+Math.toDegrees(destFrame.getBetaRad()));
                
                
                // Set new destination
       		double Distance=Math.sqrt( (initialPosition.getX()-destFrame.getX())*(initialPosition.getX()-destFrame.getX())+(initialPosition.getY()-destFrame.getY())*(initialPosition.getY()-destFrame.getY())+(initialPosition.getZ()-destFrame.getZ())*(initialPosition.getZ()-destFrame.getZ()));
//       		System.out.println("Distance"+Distance);
//       		System.out.println(initialPosition);
//       		System.out.println(destFrame);
       		
//       		ThreadUtil.milliSleep(100);
//       		 Frame Ptest2 = initialPosition.transform((Transformation.ofDeg(0, 0, 0, 0, 5.729577951308232*(Math.toDegrees(destFrame.getBetaRad())-Math.toDegrees(initialPosition.getBetaRad())), 0)));
//       		 System.out.println("err1: "+" a:"+Math.toDegrees(Ptest2.getAlphaRad())+" b:"+Math.toDegrees(Ptest2.getBetaRad())+" c:"+Math.toDegrees(Ptest2.getGammaRad()));
//             System.out.println("a: "+Math.toDegrees(Ptest2.getAlphaRad())+"b: "+Math.toDegrees(Ptest2.getBetaRad())+"c "+Math.toDegrees(Ptest2.getGammaRad()));
      
             theSmartServoLINRuntime.setDestination(initialPosition);
//             ThreadUtil.milliSleep(1000);
//             System.out.println("x: "+Ptest2.getX()+"y: "+Ptest2.getY()+"c "+"z: "+Ptest2.getZ());
//             System.out.println("a: "+Math.toDegrees(Ptest2.getAlphaRad())+"b: "+Math.toDegrees(Ptest2.getBetaRad())+"c "+Math.toDegrees(Ptest2.getGammaRad()));
             // Modify the stiffness settings every now and then
       		i1++;
       		
             if (i1 % 2 == 0)
             {
//            	 System.out.println("ss");
                 if (mode instanceof CartesianImpedanceControlMode)
                 {
               		 if (Distance>100)
               		 {
               			 double nForceY=50+3*(Distance-100);
               			 double nForceZ=50+3*(Distance-100);
               			 if (nForceY>5000)
               			 {
               				nForceY=5000;
               			 }
               			 if (nForceZ>5000)
               			 {
               				nForceZ=5000;
               			 }
                         final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) mode;
//                       final double aTransStiffVal = Math.max(100. * (i
//                               / (double) NUM_RUNS + 1), 1000.);
//                       final double aRotStiffVal = Math.max(10. * (i
//                               / (double) NUM_RUNS + 1), 150.);
                       cartImp.parametrize(CartDOF.Y).setStiffness(
                      		 nForceY);
                       cartImp.parametrize(CartDOF.Z).setStiffness(
                        		 nForceY);
               			 System.out.println(nForceY);
               			theSmartServoLINRuntime.changeControlModeSettings(cartImp);
               		 }
               		 else
               		 {
               			 final CartesianImpedanceControlMode cartImp1 = (CartesianImpedanceControlMode) mode;
                         cartImp1.parametrize(CartDOF.Y).setStiffness(
                          		 50);
                           cartImp1.parametrize(CartDOF.Z).setStiffness(
                            		 50);
                           theSmartServoLINRuntime.changeControlModeSettings(cartImp1);
               		 }
                     // We are in CartImp Mode,
                     // Modify the settings:
                     // NOTE: YOU HAVE TO REMAIN POSITIVE SEMI-DEFINITE !!
                     // NOTE: DONT CHANGE TOO FAST THE SETTINGS, ELSE YOU
                     // WILL DESTABILIZE THE CONTROLLER

//                     cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);

                     // Send the new Stiffness settings down to the
                     // controller
                     
                 }
             }
             
             if(i1==10000){
            	 i1=0;
             }
             aStep.end();
            }

 
        }
        catch (Exception e)
        {
        	System.out.println("66");
            getLogger().error(e.getLocalizedMessage());
            e.printStackTrace();
        }
        return timing;
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main()
    {
    	
        final motion2 app = new motion2();
        app.runApplication();
    }
}
