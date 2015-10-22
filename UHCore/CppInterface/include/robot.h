#ifndef ROBOT_INTERFACE
#define ROBOT_INTERFACE
#include "pythonInterface.h"
#include <string>
#include <vector>

/*
 * Exception that is thrown if the robot instance fails to be created
 * This is usually a result of an invalid configuration on the python
 * side.  Check Core/config.py for configuration options
 */
class RobotBuildException: public std::exception {
private:
	const char* message;

public:
	RobotBuildException(const char* message) {
		RobotBuildException::message = message;
	}

	virtual const char* what() const throw () {
		return RobotBuildException::message;
	}
};

/*
 * This is the public c++ interface into the UHCore python backend
 * Notes on usage: While this module is threadsafe, the PYTHON interpreter is NOT inherently threadsafe
 * All reasonable precautions have been taken to add thread safety, but it is a known issue that if
 * this code is called from within a C++ thread that is Terminated, the python interpreter will become
 * unstable and the subsequent calls WILL result in a segfault.  This problem can be avoided by allowing
 * the destructor to run as the destructor on this class cleans up and attempts to unload any python resources
 * that were used.
 */
class Robot: public PythonInterface {
public:
	/*
	 * Lightweight object to hold returned location information
	 * For sending Locations to setComponentState, use the more generic Position struct
	 */
	struct Location {
		// x position in world coordinates
		double x;

		// y position in world coordinates
		double y;

		// orientation in degrees
		double orientation;

		// if known, the name of the location
		std::string name;
	};

	/*
	 * Lightweight object for storing possible named positions
	 * Either name or positions can be used in setComponentState() to activate this position
	 */
	struct Position {
		// the name of the position
		std::string name;

		// the joint values of the position
		std::vector<double> positions;
	};

	/*
	 * Lightweight object used to store details about a current components position
	 * Joints is a vector that stores the name of all joints on this component
	 * The current goals of the joint are stored in the goals array
	 * There is a 1-to-1 map between indexes of these two arrays
	 */
	struct State:Position {
		// the joint names used in the position
		std::vector<std::string> joints;

		// the goal values for the joints
		std::vector<double> goals;
	};

	// Returns the name of the current robot
	std::string getName();

	// modulePath is the URI to the UHCore module
	// create an interface to the specified robot
	// if robotName not specified, create an interface to the current active robot
	Robot(std::string modulePath, std::string robotName = "");

	// set the light to the specified RGB value
	void setLight(int color[]);

	// set the light to the specified color
	void setLight(std::string color);

	// returns the image from the robots camera, rotated if needed
	char* getImage(std::string retFormat = "jpg");

	// return the current location of the robot
	Location getLocation();

	// set the named component to the named state
	std::string setComponentState(std::string name, std::string value, bool blocking = true);

	// set the named component to the specified position
	std::string setComponentState(std::string name, std::vector<double> jointGoals, bool blocking = true);

	// return all possible positions for a component
	std::vector<Position> getComponentPositions(std::string componentName);

	// return a string vector containing the name of all the components
	std::vector<std::string> getComponents();

	// return the current state of a component
	State getComponentState(std::string componentName);

	// use text to speech to say the specified string, using the specified language code (and optional sub-code)
	// http://en.wikipedia.org/wiki/Language_localisation#Language_tags_and_codes
	void say(std::string text, std::string languageCode = "en-gb", bool blocking = true);

	// play the specified file
	void play(std::string fileName, bool blocking = true);

	// block for the specified time
	void sleep(int milliseconds);

	// Stops the named component, or all if empty name
	void stop(std::string componentName = "");
protected:
	std::vector<double> parseDoubleArray(PyObject* array);
	std::vector<std::string> parseStringArray(PyObject* array);
	PyObject* getDefaultClassInstance();

private:
	PyObject* pInstance;
	std::string name;
};

#endif //#ROBOT_INTERFACE
