package kitbot;
import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Toolkit;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.highgui.Highgui;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.*;

import kitbot.BotClient;

public class KitBotMain {
	static Point oldP;
	static int state;
	static double area;
	private static Point GetClosest( List<MatOfPoint> contours, double dist, boolean teal)
	   {
		   int minArea = 10;
		   
		   Point p = new Point();
		   for (int idx = 0; idx < contours.size(); idx++) {
		        Mat contour = contours.get(idx);
		        double contourarea = Imgproc.contourArea(contour);
		        if (contourarea > minArea) 
		        {
		        	Moments mu = Imgproc.moments(contour, true);
		            int x = (int) (mu.get_m10() / mu.get_m00());
		            int y = (int) (mu.get_m01() / mu.get_m00());

				    System.out.println(oldP);
				    
		            double newDist = Math.sqrt(Math.pow((x-oldP.x), 2)+Math.pow((y-oldP.y), 2));
		            if(newDist < dist)
		            {
		            	if(teal && contourarea>area)
		            	{
		            		area = contourarea;
		            	}
		            	p = new Point(x,y);
		            	dist = newDist;
		            }
		        }
		    }
		   return p;
	   }
	
    public static void main(String[] args) {
    	int width = 1366;
    	int height = 768;
    	
    	try {
    		Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
    		width = dim.width;
    		height = dim.height;
    	} catch ( Exception e ) {
    		System.out.println( e );
    	}
    	
    	JFrame window = new JFrame("Kit Bot Interface");
    	window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    	
    	KitBotModel model = new KitBotModel();
    	KitBotView view = new KitBotView(width,height,window);
    	KitBotController controller = new KitBotController(model,view);
    	
    	window.setSize(width,height);
    	window.setVisible(true);                                                                  
    	 System.out.println("Hello, OpenCV");
		    // Load the native library.
		    System.loadLibrary("opencv_java248");
		    VideoCapture camera = new VideoCapture(1);
		    camera.open(1); //Not Useless, actually //AC- Yay!
		    boolean hset = camera.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT,480);
		    boolean wset = camera.set(Highgui.CV_CAP_PROP_FRAME_WIDTH,720);
		    System.out.println(hset);
		    System.out.println(wset);
		    
		    System.out.println("CameraSet!");
		    if(!camera.isOpened()){
		        System.out.println("Camera Error");
		    }
		    else{
		        System.out.println("Camera OK?");
		    }
		    
		    Mat frame = new Mat(50,50,1);
		    Mat frameOut = new Mat();

		    Mat mask = new Mat();
		    Mat maskTwo = new Mat();
		    Mat maskOut = new Mat();
		    //camera.grab();
		    //System.out.println("Frame Grabbed");
		    //camera.retrieve(frame);
		    //System.out.println("Frame Decoded");
		     width = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_WIDTH));
			 height = (int) (camera.get(Highgui.CV_CAP_PROP_FRAME_HEIGHT));
			JLabel opencvPane = createWindow("OpenCV output", width, height);
			state = 2;
		//r forward
		/*while(true){
			model.updatePos();
			if(Math.abs(model.heading)> 0.1){
				model.setMotors(0.5*model.heading, -0.5*model.heading);
			}else{
				model.setMotors(0.1,0.1);
				if(model.y>0.5){
					model.setMotors(0,0);
					break;
				}
			}
		}
		
		*
		BotClient botclient = new BotClient("18.150.7.174:6667","mT82Qi240y",false);
		
		while( !botclient.gameStarted() ) {
		}
		System.out.println("***GAME STARTED***");
		System.out.println("MAP --> " + botclient.getMap());
	
			*/
        //Chase Ball
    	long time = System.nanoTime();
    	while ( true ) {
    		try {
    			long duration = (System.nanoTime()- time)/(long)Math.pow(10.0,9.0);// In seconds
    			System.out.println("Current Fps:"+ 1.0/duration + "frame/Second.");
    			time = time + duration;
    			camera.read(frame);
 			    Imgproc.cvtColor(frame, frameOut, Imgproc.COLOR_BGR2HSV);
 			    frameOut.copyTo(mask);
 			   //RED: 
 			    boolean teal=false;

 			    if(state ==1) //BALL COLLECT
 			    {
				    Core.inRange(frameOut,new Scalar(0,160,60) , new Scalar(10,256,256), mask); 
				    //Core.inRange(frameOut,new Scalar(170,160,60) , new Scalar(180,256,256), maskTwo); 
				    //GREEN:
				    Core.inRange(frameOut,new Scalar(38,160,60) , new Scalar(75,256,256), maskTwo); 

				    Core.bitwise_or(maskTwo, mask, mask);
 			    }
 			    if(state ==2) //SEEK GOAL
 			    {
 			    	Core.inRange(frameOut,new Scalar(80,160,160) , new Scalar(100,256,256), mask); 
 			    	teal=true;
 			    }
				Imgproc.GaussianBlur(mask, maskOut,new Size(3,3), .2,.2);

			   // Core.inRange(frameOut,new Scalar(0,160,60) , new Scalar(10,256,256), mask); 
			   // Core.inRange(frameOut,new Scalar(170,160,60) , new Scalar(180,256,256), mask); 
			    //GREEN:
			    /* No difference
			    camera.release();1
			    */
			    //Okay! Here's the contour finding for getting the center of mass of each blob
			    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
			    Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
			   
			    Point p = GetClosest(contours,10000000, teal);
			    //Imgproc.findContours(maskTwo, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
			    
			    //p = GetClosest(contours, ( Math.sqrt(Math.pow((p.x-oldP.x), 2)+Math.pow((p.y-oldP.y), 2))));

			    
            	Core.circle(frame, p, 4, new Scalar(255,49,0,255));
            	oldP =p;
			   
				updateWindow(opencvPane, frame);
 			    
 			    System.out.println("area of viewed thingy : " + area);
 			    System.out.println("x" + p.x +"y:"  +p.y);
 			    //Tracking
 			    if(Double.isNaN(p.x)|| p.x == 0){ 
 			    	p.x = frame.width()/2;
 			    	p.y = frame.height()/2;
 			    	model.setMotors(-0.17,-0.08);
 			    	continue;
 			    }
 			    double rolMag = 0.5*((p.x - frame.width()/2 ) / frame.width());
 			    double camHeight = 0.1778;//m
 			    double desiredDist = -0.2;//m;
 			    double setCamAngle =  0.785398163;//radian
 			    //Some Math
 			    //p.yConstrain
 			    double trackAngle = ((p.y - frame.height()/2) / frame.height())*0.589048625;//radian
 			    if(trackAngle + setCamAngle > Math.PI/2-0.1){ //Set hard limit that ball can only be 2m away.
 			    	trackAngle = Math.PI/2 - 0.1 -setCamAngle;
 			    }
 			    double forMag = -(float)(frame.height()-p.y)/frame.height();//2*(Math.tan(trackAngle+setCamAngle)*camHeight-desiredDist);
 			    System.out.println("Forward:" + forMag);
 			    if(controller.EmgStop == true){
 			    	model.finalize();
 			    	break;
 			    }
 			    model.setMotors(forMag-rolMag, forMag+rolMag);//0.4,0.4);//
 			    //model.updatePos();
 			    //view.repaint();
    		} catch ( Exception e ) {}
    	}
    }
    private static JLabel createWindow(String name, int width, int height) {    
        JFrame imageFrame = new JFrame(name);
        imageFrame.setSize(width, height);
        imageFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        JLabel imagePane = new JLabel();
        imagePane.setLayout(new BorderLayout());
        imageFrame.setContentPane(imagePane);
        
        imageFrame.setVisible(true);
        return imagePane;
    }
    private static void updateWindow(JLabel imagePane, Mat mat) {
    	int w = (int) (mat.size().width);
    	int h = (int) (mat.size().height);
    	if (imagePane.getWidth() != w || imagePane.getHeight() != h) {
    		imagePane.setSize(w, h);
    	}
    	BufferedImage bufferedImage = Mat2Image.getImage(mat);
    	imagePane.setIcon(new ImageIcon(bufferedImage));
    }

    
}
