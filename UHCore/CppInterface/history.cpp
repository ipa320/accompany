#include "include/history.h"
#include "Python.h"
#include <string>
#include <iostream>
#include <stdio.h>

ActionHistory::ActionHistory(std::string modulePath) :
		PythonInterface(modulePath) {
	pInstance = NULL;
}

PyObject* ActionHistory::getDefaultClassInstance() {
	if (pInstance == NULL) {
		pInstance = getClassInstance("history", "ActionHistory", NULL);
	}

	return pInstance;
}

bool ActionHistory::cancelPollingHistory(std::string ruleName) {
	PyObject* pValue = callMethod("cancelPollingHistory", ruleName);

	bool ret;
	{
		PythonLock lock = PythonLock();
		ret = PyObject_IsTrue(pValue);

		Py_DECREF(pValue);
	}

	return ret;
}

char* ActionHistory::addPollingHistory(std::string ruleName, float delaySeconds) {
	char* m = strdup("addPollingHistory");
	char* f = strdup("(sf)");

	PyObject *pValue = callMethod(m, f, ruleName.c_str(), delaySeconds);

	char* ret;
	{
		PythonLock lock = PythonLock();
		if (pValue != NULL) {
			ret = PyString_AsString(pValue);
			Py_DECREF(pValue);
		} else {
			std::cerr << "Error while calling method" << std::endl;
			PyErr_Print();
			PyErr_Clear();
		}
	}

	return ret;
}

void ActionHistory::addHistoryAsync(std::string ruleName) {

	PyObject* pValue = callMethod("addHistoryAsync", ruleName);
	{
		PythonLock lock = PythonLock();
		Py_DECREF(pValue);
	}
}

void ActionHistory::addHistoryCompleteAsync(std::string ruleName) {

	PyObject* pValue = callMethod("addHistoryCompleteAsync", ruleName);
	{
		PythonLock lock = PythonLock();
		Py_DECREF(pValue);
	}
}

bool ActionHistory::addHistory(std::string ruleName) {
	PyObject* pValue = callMethod("addHistory", ruleName);
	bool ret;
	{
		PythonLock lock = PythonLock();

		ret = PyObject_IsTrue(pValue);
		Py_DECREF(pValue);
	}
	return ret;
}
