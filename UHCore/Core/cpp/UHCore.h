
#include <string>

class PythonInterface {
public:
	PythonInterface(const char* modulePath);
	virtual ~PythonInterface();
protected:
	virtual std::string getModuleName() = 0;
	virtual std::string getClassName() = 0;
};

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
};
