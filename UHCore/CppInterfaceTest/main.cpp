#include "UHCore.h"
#include <iostream>

/*
be sure to link in python during compilation ('-lpython2.6' for gcc linker)
*/

int main(int argc, char *argv[])
{
	ActionHistory *hist = new ActionHistory("/home/nathan/Dropbox/PhD/Code/UHCore/Core");
	const char* ruleName = "testPythonInterface";
	std::cout << hist->addHistory(ruleName) << '\n';
	hist->addHistoryAsync(ruleName);
	delete hist;
	return 0;
}
