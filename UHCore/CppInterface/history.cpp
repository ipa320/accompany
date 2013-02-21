#include "history.h"
#include "Python.h"
#include <string>
#include <iostream>
#include <stdio.h>

ActionHistory::ActionHistory(const char* modulePath) :
		PythonInterface(modulePath) {
	}

std::string ActionHistory::getModuleName() {
	return std::string("history");
}
std::string ActionHistory::getClassName() {
	return std::string("ActionHistory");
}

bool ActionHistory::cancelPollingHistory(const char* ruleName) {
	PyObject* pValue = callMethod("cancelPollingHistory", ruleName);
	bool ret = false;
	if (PyObject_IsTrue(pValue)) {
		ret = true;
	}

	Py_DECREF(pValue);

	return ret;
}

char* ActionHistory::addPollingHistory(const char* ruleName, float delaySeconds) {

	char* m = strdup("addPollingHistory");
	char* f = strdup("(sf)");
	PyObject *pValue = PyObject_CallMethod(getClassInstance(), m, f, ruleName,
			delaySeconds);
	delete m;
	delete f;

	if (pValue != NULL) {
		char* ret = PyString_AsString(pValue);
		Py_DECREF(pValue);
		return ret;
	} else {
		std::cout << "Error while calling method" << '\n';
		PyErr_Print();
		return NULL;
	}
}

void ActionHistory::addHistoryAsync(const char* ruleName) {
	PyObject* pValue = callMethod("addHistoryAsync", ruleName);
	Py_DECREF(pValue);
}

bool ActionHistory::addHistory(const char* ruleName) {
	PyObject* pValue = callMethod("addHistory", ruleName);
	bool ret = false;
	if (PyObject_IsTrue(pValue)) {
		ret = true;
	}

	Py_DECREF(pValue);

	return ret;
}
