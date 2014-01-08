package kitbot;
import java.awt.Dimension;
import java.awt.Toolkit;

import javax.swing.JFrame;

import org.opencv.core.Core;
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

public class KitBotMain {
	
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
		    camera.open(1); //Useless
		    if(!camera.isOpened()){
		        System.out.println("Camera Error");
		    }
		    else{
		        System.out.println("Camera OK?");
		    }
		    
		    Mat frame = new Mat();
		    Mat frameOut = new Mat();

		    Mat mask = new Mat();
		    Mat maskOut = new Mat();
		    //camera.grab();
		    //System.out.println("Frame Grabbed");
		    //camera.retrieve(frame);
		    //System.out.println("Frame Decoded");
		    
			  
    	while ( true ) {
    		try {
    			Thread.sleep(10);
    			camera.read(frame);
 			   
 			    Imgproc.cvtColor(frame, frameOut, Imgproc.COLOR_BGR2HSV);
 			   
 			    frameOut.copyTo(mask);
 			    
 			    Core.inRange(frameOut,new Scalar(0,160,60) , new Scalar(10,256,256), mask); 
 			    Imgproc.GaussianBlur(mask, maskOut,new Size(3,3), .8,.8);
 			    Moments mu = Imgproc.moments(maskOut, true);
 			    
 			    Point p = new Point(mu.get_m10()/mu.get_m00() , mu.get_m01()/mu.get_m00() );
 			    /* No difference
 			    camera.release();1
 			    */
 			    
 			    System.out.println("Captured Frame Width " + frame.width());
 			    System.out.println("x" + p.x +"y:"  +p.y);
 			    if(Double.isNaN(p.x)){
 			    	p.x = frame.width()/2;
 			    	p.y = frame.height()/2;
 			    }
 			    float rolMag = (float) (0.6*((p.x - frame.width()/2 ) / frame.width()));
 			    float forMag = (float) (0.8*((p.y - frame.height()/2 ) / frame.height()));
 			    if(controller.EmgStop == true)
 			    {
 			    	model.setMotors(0,0);
 			    	break;
 			    }
 			    model.setMotors(-forMag-rolMag, -forMag +rolMag);
 			    view.repaint();
    		} catch ( Exception e ) {}
    	}
    }
    
}
