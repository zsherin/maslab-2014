import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.swing.JComponent;
import javax.swing.JFrame;

public class BotClientMap {
	public double gridSize;
	public Pose startPose;
	public ArrayList<Wall> walls;
	public Pose robotPose;
	/**
	 * Initator
	 * Open up an arraylist for walls
	 */
	public BotClientMap() {
		walls = new ArrayList<>();
	}
	/**
	 * Load a map from a given string.
	 * Most likely provided by botClient
	 * @params s String to be inputed.
	 */
	public void load(String s) {
		String[] parts = s.split(":");
		int i = 0;
		this.gridSize = parseGridSize(parts[i++]);
		this.startPose = parsePose(parts[i++]);
		
		for (; i < parts.length; i++) {
			walls.add(parseWall(parts[i]));
		}
	}
	/**
	 * Retrieve the grid dimension from a string.
	 * @param s String.
	 * @return double representing grid size.
	 */
	private double parseGridSize(String s) {
		return Double.valueOf(s);
	}
	/**
	 * Retrieve pose data from a string.
	 * This is used to get the starting robot pose, but can be used for anything else.
	 * @param s String
	 * @return a Pose of the data given.
	 */
	private Pose parsePose(String s) {
		String[] parts = s.split(",");
		return new Pose(Double.valueOf(parts[0]), Double.valueOf(parts[1]), Double.valueOf(parts[2]));
	}
	/**
	 * Retrieve wall data from a string.
	 * @param s String
	 * @return Wall.
	 */
	private Wall parseWall(String s) {
		String[] parts = s.split(",");
		
		Point start = new Point(Double.valueOf(parts[0]), Double.valueOf(parts[1]));
		Point end = new Point(Double.valueOf(parts[2]), Double.valueOf(parts[3]));
		Wall.WallType type = Wall.WallType.values()[Wall.WallTypeShort.valueOf(parts[4]).ordinal()];
		
		return new Wall(start, end, type);
	}
	/**
	 * Simple Point class.
	 *
	 */
	public static class Point {
		public final double x;
		public final double y;
		
		public Point (double x, double y) {
			this.x = x;
			this.y = y;
		}
		
		@Override
		public String toString() {
			return String.format("(%.2f, %.2f)", x, y);
		}
	}
	/**
	 * Simple Pose class, which extends Point.
	 */
	public static class Pose extends Point{
		public final double theta;
		
		public Pose(double x, double y, double theta) {
			super(x, y);
			this.theta = theta;
		}
		
		@Override
		public String toString() {
			return String.format("(%.2f, %.2f, %.2f)", x, y, theta);
		}
	}
	/**
	 * a Wall class.
	 * Comes in four type:
	 * Normal, Opponent (Wall that is between our and enemy's field), silo, reactor
	 * 
	 *
	 */
	public static class Wall {
		enum WallTypeShort {N, O, S, R};
		enum WallType {NORMAL, OPPONENT, SILO, REACTOR};
		
		public final WallType type;
		public final Point start;
		public final Point end;
		
		public Wall(Point start, Point end, WallType type) {
			this.start = start;
			this.end = end;
			this.type = type;
		}
		

		@Override
		public String toString() {
			return String.format("Wall: %s\t[%s - %s]", type, start, end);
		}
	}
	/**
	 * to String of the Map.
	 * Just in case that wasn't clear.
	 * It seems to be designed for the console rather than to be parse by another program.
	 */
	@Override
	public String toString() {
		String mapString = String.format("Grid Size: %.2f\n", gridSize);
		mapString += "Pose: " + this.startPose.toString();
		for (Wall w : walls)
			mapString += "\n" + w.toString();
		return mapString;
	}
	/**
	 * A manually inputed default map.
	 * @return BotClientMap of the default map. 
	 */
	public static BotClientMap getDefaultMap() {
		String mapString = "22.0:4.5,5.5,3.14159:";
		mapString += "0,0,0,1,N:0,1,1,2,O:1,2,1,3,O:1,3,0,4,O:0,4,0,6,S:0,6,3,6,N:3,6,4,6,R:4,6,7,6,N:7,6,7,1,N:7,1,6,0,N:";
		mapString += "6,0,5,0,R:5,0,3,0,N:3,0,2,1,N:2,1,1,0,N:1,0,0,0,R:";
		mapString += "4,0,4,5,N:4,5,5,5,N:";
		mapString += "4,1,3,2,N:3,2,2,2,N:2,2,2,3,N:2,3,4,3,N:";
		mapString += "5,1,5,3,N:5,3,6,3,N:6,3,6,2,N:6,2,5,1,N:";
		
		BotClientMap m = new BotClientMap();
		m.load(mapString);
		return m;
	}
	/**
	 * draws the Map of a new JFrame.
	 * use WallPainter if you want to update the map.
	 */
	public void drawMap() {
		JFrame jf = new JFrame();
		jf.setContentPane(new WallPainter());
		jf.setSize(500,500);
		jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		jf.setBackground(Color.white);
		jf.setVisible(true);
	}
	/**
	 * WallPainter acts as a contentPane.
	 * It should be set as a contentPane to the desired JFrame output.
	 * WallPainter is designed to be only used by BotClientMap.  So Any changes must to the JFrame output,
	 * should be written within the BotClientMap.
	 */
	private class WallPainter extends JComponent {
		@Override
		public void paint(Graphics g) {
			super.paint(g);
			int size = 50;
			g.translate(1*size, 8*size);
		    ((Graphics2D)g).setStroke(new BasicStroke(3));

			Color[] colors = new Color[] {Color.black, Color.yellow, new Color(255,0,255), Color.green};
			for (Wall w : walls) {
				g.setColor(colors[w.type.ordinal()]);
				g.drawLine(size * (int)w.start.x, size * -(int)w.start.y, size * (int)w.end.x, size * -(int)w.end.y);
			}
			
			g.fillOval((int) (size*startPose.x) - (int)(size/4.0),(int) (-size*startPose.y) - (int)(size/4.0), (int)(size/2.0), (int)(size/2.0));
			
			double DX = Math.cos(startPose.theta) * size/2.0;
			double DY = -Math.sin(startPose.theta) * size/2.0;
			g.drawLine((int) (size*startPose.x), (int) (-size*startPose.y), (int) (size*startPose.x + DX), (int) (-size*startPose.y + DY));
		}
	}	
	public static void main(String[] args) {	
		BotClientMap map = getDefaultMap();
		map.drawMap();
	}
}