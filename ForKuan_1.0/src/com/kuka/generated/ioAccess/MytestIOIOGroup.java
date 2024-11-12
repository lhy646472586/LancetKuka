package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>MytestIO</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
@Singleton
public class MytestIOIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'MytestIO'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'MytestIO'
	 */
	@Inject
	public MytestIOIOGroup(Controller controller)
	{
		super(controller, "MytestIO");

		addInput("Input1", IOTypes.BOOLEAN, 1);
		addInput("Input2", IOTypes.BOOLEAN, 1);
		addInput("Input3", IOTypes.BOOLEAN, 1);
		addInput("Input4", IOTypes.BOOLEAN, 1);
		addInput("Input5", IOTypes.BOOLEAN, 1);
		addInput("Input6", IOTypes.BOOLEAN, 1);
		addInput("Input7", IOTypes.BOOLEAN, 1);
		addInput("Input8", IOTypes.BOOLEAN, 1);
		addInput("Input9", IOTypes.BOOLEAN, 1);
		addInput("Input10", IOTypes.BOOLEAN, 1);
		addInput("Input11", IOTypes.BOOLEAN, 1);
		addInput("Input12", IOTypes.BOOLEAN, 1);
		addInput("Input13", IOTypes.BOOLEAN, 1);
		addInput("Input14", IOTypes.BOOLEAN, 1);
		addInput("Input15", IOTypes.BOOLEAN, 1);
		addInput("Input16", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output1", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output2", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output3", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output4", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output5", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output6", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output7", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output8", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output9", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output10", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output11", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output12", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output13", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output14", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output15", IOTypes.BOOLEAN, 1);
		addDigitalOutput("Output16", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input1'
	 */
	public boolean getInput1()
	{
		return getBooleanIOValue("Input1", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input2'
	 */
	public boolean getInput2()
	{
		return getBooleanIOValue("Input2", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input3'
	 */
	public boolean getInput3()
	{
		return getBooleanIOValue("Input3", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input4'
	 */
	public boolean getInput4()
	{
		return getBooleanIOValue("Input4", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input5'
	 */
	public boolean getInput5()
	{
		return getBooleanIOValue("Input5", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input6</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input6'
	 */
	public boolean getInput6()
	{
		return getBooleanIOValue("Input6", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input7</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input7'
	 */
	public boolean getInput7()
	{
		return getBooleanIOValue("Input7", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input8</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input8'
	 */
	public boolean getInput8()
	{
		return getBooleanIOValue("Input8", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input9</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input9'
	 */
	public boolean getInput9()
	{
		return getBooleanIOValue("Input9", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input10'
	 */
	public boolean getInput10()
	{
		return getBooleanIOValue("Input10", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input11'
	 */
	public boolean getInput11()
	{
		return getBooleanIOValue("Input11", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input12'
	 */
	public boolean getInput12()
	{
		return getBooleanIOValue("Input12", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input13'
	 */
	public boolean getInput13()
	{
		return getBooleanIOValue("Input13", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input14'
	 */
	public boolean getInput14()
	{
		return getBooleanIOValue("Input14", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input15'
	 */
	public boolean getInput15()
	{
		return getBooleanIOValue("Input15", false);
	}

	/**
	 * Gets the value of the <b>digital input '<i>Input16</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital input 'Input16'
	 */
	public boolean getInput16()
	{
		return getBooleanIOValue("Input16", false);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output1'
	 */
	public boolean getOutput1()
	{
		return getBooleanIOValue("Output1", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output1'
	 */
	public void setOutput1(java.lang.Boolean value)
	{
		setDigitalOutput("Output1", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output2'
	 */
	public boolean getOutput2()
	{
		return getBooleanIOValue("Output2", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output2'
	 */
	public void setOutput2(java.lang.Boolean value)
	{
		setDigitalOutput("Output2", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output3'
	 */
	public boolean getOutput3()
	{
		return getBooleanIOValue("Output3", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output3'
	 */
	public void setOutput3(java.lang.Boolean value)
	{
		setDigitalOutput("Output3", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output4'
	 */
	public boolean getOutput4()
	{
		return getBooleanIOValue("Output4", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output4'
	 */
	public void setOutput4(java.lang.Boolean value)
	{
		setDigitalOutput("Output4", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output5'
	 */
	public boolean getOutput5()
	{
		return getBooleanIOValue("Output5", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output5'
	 */
	public void setOutput5(java.lang.Boolean value)
	{
		setDigitalOutput("Output5", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output6</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output6'
	 */
	public boolean getOutput6()
	{
		return getBooleanIOValue("Output6", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output6</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output6'
	 */
	public void setOutput6(java.lang.Boolean value)
	{
		setDigitalOutput("Output6", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output7</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output7'
	 */
	public boolean getOutput7()
	{
		return getBooleanIOValue("Output7", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output7</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output7'
	 */
	public void setOutput7(java.lang.Boolean value)
	{
		setDigitalOutput("Output7", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output8</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output8'
	 */
	public boolean getOutput8()
	{
		return getBooleanIOValue("Output8", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output8</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output8'
	 */
	public void setOutput8(java.lang.Boolean value)
	{
		setDigitalOutput("Output8", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output9</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output9'
	 */
	public boolean getOutput9()
	{
		return getBooleanIOValue("Output9", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output9</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output9'
	 */
	public void setOutput9(java.lang.Boolean value)
	{
		setDigitalOutput("Output9", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output10'
	 */
	public boolean getOutput10()
	{
		return getBooleanIOValue("Output10", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output10</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output10'
	 */
	public void setOutput10(java.lang.Boolean value)
	{
		setDigitalOutput("Output10", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output11'
	 */
	public boolean getOutput11()
	{
		return getBooleanIOValue("Output11", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output11</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output11'
	 */
	public void setOutput11(java.lang.Boolean value)
	{
		setDigitalOutput("Output11", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output12'
	 */
	public boolean getOutput12()
	{
		return getBooleanIOValue("Output12", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output12</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output12'
	 */
	public void setOutput12(java.lang.Boolean value)
	{
		setDigitalOutput("Output12", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output13'
	 */
	public boolean getOutput13()
	{
		return getBooleanIOValue("Output13", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output13</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output13'
	 */
	public void setOutput13(java.lang.Boolean value)
	{
		setDigitalOutput("Output13", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output14'
	 */
	public boolean getOutput14()
	{
		return getBooleanIOValue("Output14", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output14</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output14'
	 */
	public void setOutput14(java.lang.Boolean value)
	{
		setDigitalOutput("Output14", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output15'
	 */
	public boolean getOutput15()
	{
		return getBooleanIOValue("Output15", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output15</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output15'
	 */
	public void setOutput15(java.lang.Boolean value)
	{
		setDigitalOutput("Output15", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>Output16</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'Output16'
	 */
	public boolean getOutput16()
	{
		return getBooleanIOValue("Output16", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>Output16</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'Output16'
	 */
	public void setOutput16(java.lang.Boolean value)
	{
		setDigitalOutput("Output16", value);
	}

}
