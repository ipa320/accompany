#ifndef PYTHON_INTERFACE
#define PYTHON_INTERFACE
#include "boost/python.hpp"
#include <string>
#include <vector>
#include <iostream>
#include <map>

class PythonLock {
public:
	PythonLock();
	~PythonLock();
private:
	PyGILState_STATE gstate;
};

class PythonInterface {
public:
	PythonInterface(std::string modulePath);
	virtual ~PythonInterface();
protected:
	virtual PyObject* getDefaultClassInstance() = 0;
	PyObject* getClassInstance(std::string moduleName, std::string className, PyObject* constructorArgs);
	PyObject* getClassObject(std::string moduleName, std::string className);
	template<typename T>
	PyObject* callMethod(std::string methodName, std::string argFormat, T arg) {
		return callMethod(getDefaultClassInstance(), methodName, argFormat, arg);
	}
	template<typename T, typename R>
	PyObject* callMethod(std::string methodName, std::string argFormat, T arg1, R arg2) {
		return callMethod(getDefaultClassInstance(), methodName, argFormat, arg1, arg2);
	}
	template<typename T, typename R, typename K>
	PyObject* callMethod(std::string methodName, std::string argFormat, T arg1, R arg2, K arg3) {
		return callMethod(getDefaultClassInstance(), methodName, argFormat, arg1, arg2, arg3);
	}
	PyObject* callMethod(std::string methodName) {
		return callMethod(getDefaultClassInstance(), methodName);
	}
	PyObject* callMethod(std::string methodName, std::string value) {
		return callMethod(getDefaultClassInstance(), methodName, value);
	}

	PyObject* callMethod(PyObject* pInstance, std::string methodName) {
		return callMethod(pInstance, methodName, "", "");
	}
	PyObject* callMethod(PyObject* pInstance, std::string methodName, std::string value) {
		char* v = strdup(value.c_str());
		char* f = strdup("(s)");
		return callMethod(pInstance, methodName, f, v);
	}
	template<typename T>
	PyObject* callMethod(PyObject* pInstance, std::string methodName, std::string argFormat, T arg);
	template<typename T, typename R>
	PyObject* callMethod(PyObject* pInstance, std::string methodName, std::string argFormat, T arg1, R arg2);
	template<typename T, typename R, typename K>
	PyObject* callMethod(PyObject* pInstance, std::string methodName, std::string argFormat, T arg1, R arg2, K arg3);
private:
	std::string modulePath;
	std::map<std::string, PyObject*> pObjectCache;
	static PyThreadState* threadState;
	static int refCount;
};

template<typename T, typename R, typename K>
PyObject* PythonInterface::callMethod(PyObject* instance, std::string methodName, std::string argFormat, T arg1, R arg2,
		K arg3) {
	PyObject *pValue;
	if (instance == NULL) {
		std::cerr << "Cannot call method " << methodName << " on NULL instance" << std::endl;
	} else {
		char* m = strdup(methodName.c_str());
		{
			PythonLock lock = PythonLock();
			if (!argFormat.empty()) {
				char* f = strdup(argFormat.c_str());
				pValue = PyObject_CallMethod(instance, m, f, arg1, arg2, arg3);
			} else {
				pValue = PyObject_CallMethod(instance, m, NULL);
			}

			if (pValue == NULL) {
				std::cerr << "Error while calling method " << methodName << std::endl;
				PyErr_Print();
				PyErr_Clear();
			}
		}
	}

	return pValue;
}

template<typename T, typename R>
PyObject* PythonInterface::callMethod(PyObject* instance, std::string methodName, std::string argFormat, T arg1,
		R arg2) {
	PyObject *pValue;
	if (instance == NULL) {
		std::cerr << "Cannot call method " << methodName << " on NULL instance" << std::endl;
	} else {
		char* m = strdup(methodName.c_str());
		{
			PythonLock lock = PythonLock();
			if (!argFormat.empty()) {
				char* f = strdup(argFormat.c_str());
				pValue = PyObject_CallMethod(instance, m, f, arg1, arg2);
			} else {
				pValue = PyObject_CallMethod(instance, m, NULL);
			}

			if (pValue == NULL) {
				std::cerr << "Error while calling method " << methodName << std::endl;
				PyErr_Print();
				PyErr_Clear();
			}
		}
	}

	return pValue;
}

template<typename T>
PyObject* PythonInterface::callMethod(PyObject* instance, std::string methodName, std::string argFormat, T arg) {
	PyObject *pValue;
	if (instance == NULL) {
		std::cerr << "Cannot call method " << methodName << " on NULL instance" << std::endl;
	} else {
		char* m = strdup(methodName.c_str());
		{
			PythonLock lock = PythonLock();
			if (!argFormat.empty()) {
				char* f = strdup(argFormat.c_str());
				pValue = PyObject_CallMethod(instance, m, f, arg);
			} else {
				pValue = PyObject_CallMethod(instance, m, NULL);
			}

			if (pValue == NULL) {
				std::cerr << "Error while calling method " << methodName << std::endl;
				PyErr_Print();
				PyErr_Clear();
			}
		}
	}

	return pValue;
}

#endif //PYTHON_INTERFACE
