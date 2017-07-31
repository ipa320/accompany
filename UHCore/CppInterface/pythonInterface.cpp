#include "include/history.h"
#include "boost/python.hpp"
#include <string>
#include <iostream>
#include <exception>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifndef PYTHON_REFCOUNT
#define	PYTHON_REFCOUNT

int PythonInterface::refCount = 0;
PyThreadState* PythonInterface::threadState = NULL;

#endif

PythonLock::PythonLock() {
	if (PyEval_ThreadsInitialized()) {
		gstate = PyGILState_Ensure();
	}
}

PythonLock::~PythonLock() {
	if (PyEval_ThreadsInitialized()) {
		PyGILState_Release(gstate);
	}
}

PythonInterface::PythonInterface(std::string modulePath) {
	if(!modulePath.empty()){
		char* fullPath = realpath(modulePath.c_str(), NULL);
		PythonInterface::modulePath = std::string(fullPath);
	} else {
		PythonInterface::modulePath = modulePath;
	}

	if (!Py_IsInitialized()) {
		Py_Initialize();
		std::cout << "Python Initialized" << std::endl;
	}

	PyEval_InitThreads();

	if (threadState == NULL) {
		threadState = PyEval_SaveThread();
	}

	refCount++;
}

PythonInterface::~PythonInterface() {

	refCount--;
	if (refCount == 0) {
		PyEval_RestoreThread(threadState);
	}

	for (std::map<std::string, PyObject*>::iterator ii = pObjectCache.begin(); ii != pObjectCache.end(); ++ii) {
		Py_DECREF((*ii).second);
	}

	Py_Finalize();
}

PyObject* PythonInterface::getClassObject(std::string moduleName, std::string className) {
	PythonLock lock = PythonLock();

	if (!modulePath.empty()) {
		PyRun_SimpleString("import sys");
		PyRun_SimpleString(("sys.path.append(\"" + std::string(modulePath) + "\")").c_str());
	}

	std::string modName = moduleName;
	PyObject *pName = PyString_FromString(modName.c_str());
	PyObject *pModule = PyImport_Import(pName);
	Py_DECREF(pName);

	if (pModule == NULL) {
		std::cerr << "Error while importing module: " << moduleName << std::endl;
		if (!modulePath.empty()) {
			std::cerr << " Base module path: " << modulePath << std::endl;
		}
		std::cerr << " Check that the module path and name are correct" << std::endl;
		PyErr_Print();
		PyErr_Clear();
		return NULL;
	}

	char* fileName = PyModule_GetFilename(pModule);
	char* argv[1] = { fileName };
	PySys_SetArgv(1, argv); //Change to old SetArgv for compatibility (SetArgvEx not available in python < 2.2.6)

	PyObject *pDict = PyModule_GetDict(pModule);
	Py_DECREF(pModule);

	PyObject *pClass = PyDict_GetItemString(pDict, className.c_str());
	Py_DECREF(pDict);

	if (pClass == NULL) {
		std::cerr << "Error while getting class definition for " << className << std::endl;
		PyErr_Print();
		PyErr_Clear();
	}

	return pClass;
}

PyObject* PythonInterface::getClassInstance(std::string moduleName, std::string className, PyObject *pClassArgs) {

	if (pObjectCache.find(moduleName + className) == pObjectCache.end()) {
		PyObject *pClass = getClassObject(moduleName, className);
		if (pClass == NULL) {
			return NULL;
		}

		{
			PythonLock lock = PythonLock();
			if (pClassArgs != NULL && !PyTuple_Check(pClassArgs)) {
				PyObject *temp = pClassArgs;
				pClassArgs = PyTuple_Pack(1, temp);
				Py_DECREF(temp);
			}

			PyObject* pInstance = PyObject_CallObject(pClass, pClassArgs);
			if (pInstance == NULL) {
				std::cerr << "Error while getting instance of " << className << " in module " << moduleName
						<< std::endl;
				PyErr_Print();
				PyErr_Clear();
				return NULL;
			}

			pObjectCache[moduleName + className] = pInstance;

			Py_DECREF(pClass);
			Py_XDECREF(pClassArgs);

			std::cout << moduleName << " Initialized" << std::endl;
		}
	}

	return pObjectCache[moduleName + className];
}

