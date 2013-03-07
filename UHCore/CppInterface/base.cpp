#include "history.h"
#include "Python.h"
#include <string>
#include <iostream>
#include <exception>
#include <stdio.h>

PythonInterface::PythonInterface(const char* modulePath) {
	PythonInterface::modulePath = modulePath;
}

PythonInterface::~PythonInterface() {
	if (pInstance != NULL) {
		Py_DECREF(pInstance);
	}

	Py_Finalize();
}

PyObject* PythonInterface::getClassInstance() {
	if (pInstance == NULL) {
		Py_Initialize();

		if (modulePath != NULL) {
			PyRun_SimpleString("import sys");
			PyRun_SimpleString(("sys.path.append(\"" + std::string(modulePath) + "\")").c_str());
		}

		std::string modName = getModuleName();
		PyObject *pName = PyString_FromString(modName.c_str());
		PyObject *pModule = PyImport_Import(pName);
		Py_DECREF(pName);

		char* fileName = PyModule_GetFilename(pModule);
		char* argv[1] = { fileName };
		PySys_SetArgvEx(1, argv, 0);

		PyObject *pDict = PyModule_GetDict(pModule);
		Py_DECREF(pModule);

		PyObject *pClass = PyDict_GetItemString(pDict, getClassName().c_str());
		Py_DECREF(pDict);

		pInstance = PyObject_CallObject(pClass, NULL);
		Py_DECREF(pClass);
	}

	return pInstance;
}

PyObject* PythonInterface::callMethod(const char* methodName, const char* arg1) {
	char* m = strdup(methodName);
	char* f = strdup("(s)");
	PyObject *pValue = PyObject_CallMethod(getClassInstance(), m, f, arg1);
	delete m;
	delete f;

	if (pValue != NULL) {
		return pValue;
	} else {
		std::cout << "Error while calling method" << '\n';
		PyErr_Print();
		return NULL;
	}
}
