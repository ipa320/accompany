#ifndef ACTION_HISTORY
#define ACTION_HISTORY
#include "pythonInterface.h"
#include <string>

class ActionHistory: public PythonInterface {
public:
	ActionHistory(std::string modulePath);
	bool cancelPollingHistory(std::string ruleName);
	char* addPollingHistory(std::string ruleName, float delaySeconds);
	void addHistoryAsync(std::string ruleName);
    void addHistoryCompleteAsync(std::string ruleName);
	bool addHistory(std::string ruleName);
private:
	PyObject* pInstance;
	PyObject* getDefaultClassInstance();
};

#endif //ACTION_HISTORY
