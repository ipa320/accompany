#include "robot.h"
#include "history.h"
#include <iostream>
#include <sstream>
#include <time.h>
#include <boost/thread.hpp>

using namespace std;

// python sources need to be included (-I/usr/include/python2.x)
// python library (-lpython2.x) and UHCore (-lUHCore) libraries need to be linked

template<typename T>
string vectorPrint(vector<T> v) {
	int size = v.size();
	stringstream ret;
	ret << "[";
	for (int i = 0; i < size; i++) {
		ret << v.at(i);
		if (i != size - 1) {
			ret << ",";
		}
	}
	ret << "]";
	return ret.str();
}

Robot *rob;

void testRobotFuncs() {
	//	Robot::State state = rob->getComponentState("arm");
	//
	//	cout << "State: " << state.name << endl;
	//	cout << " Joints: " << vectorPrint(state.joints) << endl;
	//	cout << " Positions: " << vectorPrint(state.positions) << endl;
	//	cout << " Goals: " << vectorPrint(state.goals) << endl;
	//
	string result = "";
	//	result = rob->setComponentState("arm", "wave", false);
	//	cout << "Set arm to 'wave', result: " << result << endl;
	//
	//	Robot::Location pos = rob->getLocation();
	//	cout << "Start Pose: [" << pos.x << "," << pos.y << "," << pos.orientation << "]" << endl;
	//
	//	vector<double> newPos(3);
	//	newPos[0] = pos.x;
	//	newPos[1] = pos.y;
	//	newPos[2] = (pos.orientation + 90) * 0.0174532925;
	//
	//	cout << "Pose: " << vectorPrint(newPos) << endl;
	//	result = rob->setComponentState("base", newPos, true);
	//	cout << "Rotate 90 degrees: " << result << endl;

	//result = rob->setComponentState("base", "userLocation", true);
	//cout << "Robot to user: " << result << endl;

	result = rob->setComponentState("tray", "raised", true);
	cout << "Set tray to 'raised', result: " << result << endl;

	result = rob->setComponentState("torso", "shake", true);
	cout << "Set torso to 'left', result: " << result << endl;

	cout << "Sleep for 500ms...";
	rob->sleep(500);

	int red[] = { 1, 0, 0 };
	rob->setLight(red);
	cout << "Set light to [1,0,0]" << endl;

	rob->setLight("white");
	cout << "Set light to 'white'" << endl;

	rob->play("filename.wav");
	rob->say("test");
	rob->say("test", "en-us");

}

boost::thread threads[10][10];
bool done = false;

void threadTerminator() {
	std::srand(time(NULL));
	boost::posix_time::time_duration noTime(0, 0, 0, 0);
	while (!done) {
		int x = std::rand() % 9 + 1;
		int y = std::rand() % 10;
		if (!threads[x][y].timed_join(noTime)) {
			std::cerr << "Killing thread x:" << x << " y:" << y << std::endl;
			threads[x][y].interrupt();
			sleep(rand() % 3);
		}
	}
}

void threadChainer(int x) {
	for (int i = 1; i < 9; i++) {
		threads[x][i] = boost::thread(testRobotFuncs);
	}

	for (int i = 1; i < 9; i++) {
		threads[x][i].join();
	}
}

int main(int argc, char *argv[]) {
	string modulePath = "../../Core";
	rob = new Robot(modulePath); //use the current robot specified in the sessioncontrol table
	cout << "Got interface for: " << rob->getName() << endl;

	ActionHistory *hist = new ActionHistory(modulePath);
	string ruleName = "testPythonInterface";
	cout << hist->addHistory(ruleName) << endl;
//	hist->addHistoryAsync(ruleName);

//	for (int i = 0; i < 10; i++) {
//		threads[i][0] = boost::thread(threadChainer, i);
//	}
//
//	boost::thread arnold(threadTerminator);
//
//	for (int i = 0; i < 10; i++) {
//		threads[i][0].join();
//	}
//
//	arnold.interrupt();

	cout << "Done" << endl;

	return 0;
}
