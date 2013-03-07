#include "UHCore.h"

/*
be sure to link python during compilation ('-lpython2.6' for gcc linker)
*/

int main(int argc, char *argv[])
{
	ActionHistory *hist = new ActionHistory("/home/nathan/Dropbox/PhD/Code/UHCore/Core");
	const char* ruleName = "testPythonInterface";
	bool success = hist->addHistory(ruleName);
	hist->addHistoryAsync(ruleName);
	delete hist;
	return 0;
}
