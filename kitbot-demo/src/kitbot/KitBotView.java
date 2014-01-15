package kitbot;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Polygon;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class KitBotView extends JPanel {
	private int width, height;
	private JFrame window;
	
	public final Polygon left,forward,right,stop;
	/**
	 * initializer
	 * It draws out the view's GUI.  This view/JPanel is where methods can be attached on, but the
	 * methods and functions are usually located in the controller.  Research Model-Controller-View Model.
	 * 
	 * @param width of the window
	 * @param height of the window
	 * @param window  The Jframe given.
	 */
	public KitBotView( int width, int height, JFrame window ) {
		this.width = width;
		this.height = height;
		this.window = window;
		
		left = new Polygon();
		left.addPoint(-150, 0);
		left.addPoint(150, -150);
		left.addPoint(150, 150);
		left.translate(width*1/4,height/2);
		
		right = new Polygon();
		right.addPoint(-150, -150);
		right.addPoint(150, 0);
		right.addPoint(-150, 150);
		right.translate(width*3/4,height/2);
		
		forward = new Polygon();
		forward.addPoint(-150, 150);
		forward.addPoint(0, -150);
		forward.addPoint(150, 150);
		forward.translate(width/2,height/4);
		
		stop = new Polygon();
		for ( int n = 0; n < 8; n++ ) {
			double x = Math.cos(n*Math.PI/4+Math.PI/8)*150;
			double y = Math.sin(n*Math.PI/4+Math.PI/8)*150;
			stop.addPoint((int)x,(int)y);	
		}
		stop.translate(width/2,height*3/4);
		
		window.getContentPane().add(this);
	}
	/**
	 * paintComponent paints Component on the given Graphics g.
	 */
	public void paintComponent( Graphics g ) {
		g.setColor( new Color(120,120,255) );
		g.fillRect(0, 0, width, height);
		
		g.setColor( Color.WHITE );
		g.fillPolygon(left);
		g.fillPolygon(right);
		g.fillPolygon(forward);
		
		g.setColor( Color.RED);
		g.fillPolygon(stop);
	}
	/**
	 * A fetch method for the window/JFrame that was passed on from the initializer.
	 * @return JFrame window.
	 */
	public JFrame getwindow() {
		return window;
	}
}
