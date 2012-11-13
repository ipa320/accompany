#ifndef ACTION_HISTORY
#define ACTION_HISTORY
#include "base.h"
#include <string>

class ActionHistory: public PythonInterface {
public:
	ActionHistory(const char* modulePath);
	bool cancelPollingHistory(const char* ruleName);
	char* addPollingHistory(const char* ruleName, float delaySeconds);
	void addHistoryAsync(const char* ruleName);
	bool addHistory(const char* ruleName);
protected:
	std::string getModuleName();
	std::string getClassName();
private:
};

#endif //ACTION_HISTORY
