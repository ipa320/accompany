#include "include/robot.h"
#include "boost/python.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <stdio.h>

Robot::Robot(std::string modulePath, std::string robotName) :
		PythonInterface(modulePath) {
	Robot::name = robotName;
	Robot::pInstance = NULL;
	Robot::pInstance = getDefaultClassInstance();

	if(Robot::pInstance == NULL) {
		throw RobotBuildException(("getInstance returned null for robot '" + robotName + "', check console for messages.").c_str());
	}
}

PyObject* Robot::getDefaultClassInstance() {
	if (Robot::pInstance == NULL) {
		PyObject* pClass = getClassObject("Robots.robotFactory", "Factory");
		if (pClass == NULL) {
			std::cerr << "Error locating class object Robots.robotFactory.Factory" << std::endl;
		} else {
			if (Robot::name != "") {
				std::cout << "Getting class for robot: " << name << std::endl;
				Robot::pInstance = callMethod(pClass, "getRobot", Robot::name.c_str());
			} else {
				std::cout << "Getting class for active robot" << std::endl;
				Robot::pInstance = callMethod(pClass, "getCurrentRobot");
			}

			{
				PythonLock lock = PythonLock();
				const char* pName = strdup("name");
				PyObject* nm = PyObject_GetAttrString(Robot::pInstance, pName);
				Robot::name = std::string(PyString_AsString(nm));
				Py_XDECREF(nm);
				Py_DECREF(pClass);
			}
		}
	}

	return Robot::pInstance;
}

std::string Robot::getName() {
	return Robot::name;
}

void Robot::setLight(int color[]) {
	PyObject *pValue = callMethod("setLight", "([i,i,i])", color[0], color[1], color[2]);

	{
		PythonLock lock = PythonLock();
		if (pValue == NULL) {
			std::cerr << "Error while calling setLight (" << color[0] << "," << color[1] << "," << color[2] << ")"
					<< std::endl;
			PyErr_Print();
			PyErr_Clear();
		}

		Py_XDECREF(pValue);
	}
}

void Robot::setLight(std::string color) {

	PyObject *pValue = callMethod("setLight", color);

	{
		PythonLock lock = PythonLock();

		if (pValue == NULL) {
			std::cerr << "Error while calling setLight (" << color << ")" << std::endl;
			PyErr_Print();
			PyErr_Clear();
		}

		Py_XDECREF(pValue);
	}
}

void Robot::stop(std::string componentName) {
	PyObject *pValue = callMethod("stop", componentName);

	{
		PythonLock lock = PythonLock();
		Py_XDECREF(pValue);
	}
}

Robot::Location Robot::getLocation() {

	Robot::Location l;
	PyObject *pValue = callMethod("getLocation");
	/** pValue = ('locName', (x, y, orientation)) **/

	if (pValue != NULL) {
		PythonLock lock = PythonLock();

		if (!PySequence_Check(pValue)) {
			//problem!
		} else {
			l.name = std::string(PyString_AsString(PySequence_GetItem(pValue, 0)));
			l.x = PyFloat_AsDouble(PySequence_GetItem(PySequence_GetItem(pValue, 1), 0));
			l.y = PyFloat_AsDouble(PySequence_GetItem(PySequence_GetItem(pValue, 1), 1));
			l.orientation = PyFloat_AsDouble(PySequence_GetItem(PySequence_GetItem(pValue, 1), 2));
		}

		Py_DECREF(pValue);
	}

	return l;
}

std::string Robot::setComponentState(std::string name, std::vector<double> jointGoals, bool blocking) {

	PyObject* g;
	{
		PythonLock lock = PythonLock();
		g = PyList_New(jointGoals.size());
		for (int i = 0; i < (int) jointGoals.size(); i++) {
			PyList_SetItem(g, i, Py_BuildValue("d", jointGoals[i]));
		}
	}

	std::string format = "(s, O, b)";
	char* n = strdup(name.c_str());
	char* f = strdup(format.c_str());

	PyObject *pValue = callMethod("setComponentState", f, n, g, blocking);
	/** pValue = "SUCCESS" **/

	std::string ret;
	{
		PythonLock lock = PythonLock();
		Py_DECREF(g);
		if (pValue != NULL) {
			ret = std::string(PyString_AsString(pValue));
			Py_DECREF(pValue);
		} else {
			std::cerr << "Error while calling setComponentState (" << name << "," << "{vectorGoals}" << ")"
					<< std::endl;
			PyErr_Print();
			PyErr_Clear();
			ret = "Error";
		}
	}

	return ret;
}

std::string Robot::setComponentState(std::string name, std::string value, bool blocking) {

	char* n = strdup(name.c_str());
	char* v = strdup(value.c_str());
	PyObject *pValue = callMethod("setComponentState", "(s,s, b)", n, v, blocking);
	/** pValue = "SUCCESS" **/

	std::string ret;
	{
		PythonLock lock = PythonLock();
		if (pValue != NULL) {
			std::string ret = std::string(PyString_AsString(pValue));
			Py_DECREF(pValue);
			return ret;
		} else {
			std::cerr << "Error while calling setComponentState (" << name << "," << value << ")" << std::endl;
			PyErr_Print();
			PyErr_Clear();
			ret = "Error";
		}
	}

	return ret;
}

std::vector<Robot::Position> Robot::getComponentPositions(std::string componentName) {
	PyObject *pValue = callMethod("getComponentPositions", componentName);
	/** pValue = {folded:(0.0, 1.1, ...)), wave:(in, out), joint_names:('elbo', 'wrist'), ...} **/

	std::vector<Robot::Position> ret = std::vector<Robot::Position>();
	if (pValue != NULL) {
		PythonLock lock = PythonLock();

		if (!PyDict_Check(pValue)) {
			std::cerr << "Expected dictionary from getComponentPositions, got " << pValue->ob_type->tp_name
					<< std::endl;
		} else {
			PyObject *key, *value;
			Py_ssize_t pos = 0;

			while (PyDict_Next(pValue, &pos, &key, &value)) {
				Robot::Position p;
				p.name = std::string(PyString_AsString(key));
				p.positions = parseDoubleArray(value);
				ret.push_back(p);
			}
		}

		Py_DECREF(pValue);
	}

	return ret;
}

std::vector<std::string> Robot::getComponents() {

	PyObject *pValue = callMethod("getComponents");
	/** pValue = ('arm', 'head', ...) **/

	std::vector<std::string> ret = std::vector<std::string>();
	if (pValue != NULL) {
		ret = parseStringArray(pValue);
		{
			PythonLock lock = PythonLock();
			Py_DECREF(pValue);
		}
	}

	return ret;
}

Robot::State Robot::getComponentState(std::string componentName) {
	PyObject *pValue = callMethod("getComponentState", componentName);
	/** pValue = ('home', {name:'arm', joints:('elbo', 'wrist', ...), positions:(0.0, 1.1, ...), goals:(0.0, 1.1, ...)}) **/

	Robot::State s;

	if (pValue != NULL) {
		PythonLock lock = PythonLock();

		if (!PySequence_Check(pValue)) {
			//Error!
		} else {
			s.name = std::string(PyString_AsString(PySequence_GetItem(pValue, 0)));
			PyObject* values = PySequence_GetItem(pValue, 1);
			if (PyDict_Check(values)) {
				PyObject* key = PyString_FromString("goals");
				s.goals = parseDoubleArray(PyDict_GetItem(values, key));
				Py_DecRef(key);
				key = PyString_FromString("positions");
				s.positions = parseDoubleArray(PyDict_GetItem(values, key));
				Py_DecRef(key);
				key = PyString_FromString("joints");
				s.joints = parseStringArray(PyDict_GetItem(values, key));
				Py_DecRef(key);
			}
		}

		Py_DecRef(pValue);
	}

	return s;
}

// block for the specified time
void Robot::sleep(int milliseconds) {
	callMethod("sleep", "(i)", milliseconds);
}

// use text to speech to say the specified string, using the specified language code (and optional sub-code)
// http://en.wikipedia.org/wiki/Language_localisation#Language_tags_and_codes
void Robot::say(std::string text, std::string languageCode, bool blocking) {
	if (!text.empty()) {
		if (languageCode.empty()) {
			languageCode = "en-gb";
		}

		char* lc = strdup(languageCode.c_str());
		char* te = strdup(text.c_str());

		callMethod("say", "(s, s, b)", te, lc, blocking);
	}
}

// play the specified file
void Robot::play(std::string fileName, bool blocking) {
	if (!fileName.empty()) {
		char* f = strdup(fileName.c_str());
		callMethod("play", "(s, b)", f, blocking);
	}
}

char* Robot::getImage(std::string retFormat) {
	char* img = NULL;
	char* r = strdup(retFormat.c_str());
	PyObject *pValue = callMethod("getImage", r);

	if (pValue != NULL) {
		PythonLock lock = PythonLock();

		//Create a copy of the string as PyString returns a pointer to its internal buffer
		// which must be modified and will be dereferenced when this function returns
		char* src = PyString_AsString(pValue);
		Py_DECREF(pValue);
		img = new char[strlen(src)];
		strcpy(img, src);
	}

	return img;
}

std::vector<double> Robot::parseDoubleArray(PyObject* array) {
	std::vector<double> ret = std::vector<double>();
	{
		PythonLock lock = PythonLock();
		if (PySequence_Check(array)) {
			for (int i = 0; i < PySequence_Size(array); i++) {
				ret.push_back(PyFloat_AsDouble(PySequence_GetItem(array, i)));
			}
		}
	}

	return ret;
}

std::vector<std::string> Robot::parseStringArray(PyObject* array) {
	std::vector<std::string> ret = std::vector<std::string>();
	{
		PythonLock lock = PythonLock();
		if (PySequence_Check(array)) {
			for (int i = 0; i < PySequence_Size(array); i++) {
				ret.push_back(std::string(PyString_AsString(PySequence_GetItem(array, i))));
			}
		}
	}

	return ret;
}
