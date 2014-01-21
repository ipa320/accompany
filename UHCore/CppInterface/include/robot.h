#ifndef ROBOT_INTERFACE
#define ROBOT_INTERFACE
#include "pythonInterface.h"
#include <string>
#include <vector>

class Robot: public PythonInterface {
public:
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

	struct Position {
		// the name of the position
		std::string name;

		// the joint values of the position
		std::vector<double> positions;
	};

	struct State: Position {

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
	Robot(std::string modulePath, std::string robotName="");

	// set the light to the specified RGB value
	void setLight(int color[]);

	// set the light to the specified color
	void setLight(std::string color);

	// returns the image from the robots camera, rotated if needed
	char* getImage(std::string retFormat="jpg");

	// return the current location of the robot
	Location getLocation();

	// set the named component to the named state
	std::string setComponentState(std::string name, std::string value, bool blocking=true);

	// set the named component to the specified position
	std::string setComponentState(std::string name, std::vector<double> jointGoals, bool blocking=true);

	// return all possible positions for a component
	std::vector<Position> getComponentPositions(std::string componentName);

	// return a string vector containing the name of all the components
	std::vector<std::string> getComponents();

	// return the current state of a component
	State getComponentState(std::string componentName);

	// use text to speech to say the specified string, using the specified language code (and optional sub-code)
	// http://en.wikipedia.org/wiki/Language_localisation#Language_tags_and_codes
	void say(std::string text, std::string languageCode="en-gb", bool blocking=true);

	// play the specified file
	void play(std::string fileName, bool blocking=true);

	// block for the specified time
	void sleep(int milliseconds);

	// Stops the named component, or all if empty name
	void stop(std::string componentName="");
protected:
	std::vector<double> parseDoubleArray(PyObject* array);
	std::vector<std::string> parseStringArray(PyObject* array);
	PyObject* getDefaultClassInstance();

private:
	PyObject* pInstance;
	std::string name;
};

#endif //#ROBOT_INTERFACE
