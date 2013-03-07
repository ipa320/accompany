#ifndef PYTHON_INTERFACE
#define PYTHON_INTERFACE
#include "Python.h"
#include <string>

class PythonInterface {
public:
	PythonInterface(const char* modulePath);
	virtual ~PythonInterface();
protected:
	virtual std::string getModuleName() = 0;
	virtual std::string getClassName() = 0;
	PyObject* callMethod(const char* methodName, const char* arg1);
	PyObject* getClassInstance();

private:
	const char* modulePath;
	PyObject* pInstance;
};

#endif //PYTHON_INTERFACE
