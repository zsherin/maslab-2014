package kitbot;

import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortList;

public class KitBotModel {
	private SerialPort serialPort;
    private byte motorA = 0;
    private byte motorB = 0;
    public double x = 0;
    public double y = 0;
    public double heading = 0;
    private byte sonar[] = new byte[7];
    /**
     * Initializer
     * It starts the serial communications using the first serial port it could find.
     * buadRate: 115200
     * databits: 8
     * stopbits: 1
     * parity: None
     */
	public KitBotModel() {
		try {
			serialPort = new SerialPort("COM6");//Will attempt to read the first port found.
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
        }
        catch (Exception ex){
            System.out.println(ex);
        }
	}
	/**
	 * setMotors sets the motor values.  This is the main method for external services to
	 * alter the motor values.
	 * @param powerA  Left Motor
	 * @param powerB  Right Motor
	 */
	public void setMotors( double powerA, double powerB ) {
    	motorA = (byte)(-motorConstrain(powerA)*127);
		motorB = (byte)(motorConstrain(powerB)*127);
		updateMotor();

		System.out.println("Motors:" + motorA + "," + motorB);
	}
	/**
	 * Sets Hard Limit to the motors
	 * @param oldpower   The original motor power value.
	 * @return  double   The new motor power value after constrains.
	 */
	private double motorConstrain(double oldpower){
		if(oldpower > 1)
			return 1;
		else if (oldpower <-1)
			return -1;
		return oldpower;
	}
	
	public void releaseG(){
		try{
			serialPort.writeByte((byte)'F');
		}catch(Exception ex){
			System.out.println(ex);
		}
	}
	public void releaseR(){
		try{
			serialPort.writeByte((byte)'G');
		}catch(Exception ex){
			System.out.println(ex);
		}
	}
	public void disableWallDetect(){
		try{
			byte[] data = new byte[2];
			serialPort.writeByte((byte)'H');
		}catch(Exception ex){
			System.out.println(ex);
		}
	}
	//SENSORY UPDATES
	/**
	 * updateMotor sends command to the Microcontroller and updates the motors.
	 * It uses the motor value within, so no arguments needed.
	 */
	public void updateMotor(){
		try {
			byte[] data = new byte[4];
			data[0] = 'A';		// Start signal "S"
			data[1] = motorA;	// Motor A data
			data[2] = motorB;	// Motor B data
			data[3] = 'E';		// End signal "E"
			serialPort.writeBytes(data);
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
	/**
	 * updateSonar gets the filtered sonar reading from the microcontroller.
	 * It will not only return the data for external services, but also update
	 * its own copy.
	 * @return byte[] unsigned sonar distances in cm. Sonar from Left to Right.
	 */
	public byte[] updateSonar(){
		try{
			serialPort.writeByte((byte)'C');
			sonar = serialPort.readBytes(5);
			serialPort.writeByte((byte)'E');
			System.out.println("Got Sonar Data:" + sonar);
		}catch (Exception ex){
		}
		return sonar;
	}
	/**
	 * updatePos gets positional updates from the microcontroller
	 * @return byte[] :  Heading, change in x, change in y.
	 */
	public String updatePos(){
		String s = " ";
		try{
			serialPort.writeByte((byte)'B');
			s = serialPort.readString();
			String data[] = s.split(" ");
			serialPort.writeByte((byte)'E');
			heading = Double.parseDouble(data[0]);
			x +=  Double.parseDouble(data[1]);;
			y +=  Double.parseDouble(data[2]);;
		}catch(Exception ex){
		}
		return s;
	}
	/**
	 * finalize closes the Serial Communication.
	 * This is used to stop the robot from functions.
	 * 
	 * TODO: Consider some end state behaviors for the robot.
	 * Like Stopping Motor,  Turn off sensory updates.
	 */
	public void finalize() {
		try {
			serialPort.writeByte((byte)'D');//Kill Code.
			serialPort.closePort();
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
}
