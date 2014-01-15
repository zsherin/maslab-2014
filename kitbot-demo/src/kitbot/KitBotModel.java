package kitbot;

import jssc.SerialPort;
import jssc.SerialPortException;
import jssc.SerialPortList;

public class KitBotModel {
	private SerialPort serialPort;
    private byte motorA = 0;
    private byte motorB = 0;
    private double x = 0;
    private double y = 0;
    private double heading = 0;
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
			serialPort = new SerialPort(SerialPortList.getPortNames()[0]);//Will attempt to read the first port found.
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
			sonar = serialPort.readBytes(7);
			serialPort.writeByte((byte)'E');
		}catch (Exception ex){
			
		}
		return sonar;
	}
	/**
	 * updatePos gets positional updates from the microcontroller
	 * @return byte[] :  Heading, change in x, change in y.
	 */
	public byte[] updatePos(){
		byte data[] = new byte[3];
		try{
			serialPort.writeByte((byte)'B');
			data = serialPort.readBytes(3);
			serialPort.writeByte((byte)'E');
			heading = data[0];
			x += data[1];
			y += data[2];
		}catch(Exception ex){
			
		}
		return data;
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
			serialPort.closePort();
		} catch ( Exception ex ) {
			System.out.println(ex);
		}
	}
}
