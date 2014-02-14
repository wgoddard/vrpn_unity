#include "vrpn_Tracker.h"
#include "vrpn_Button.h"
#include "vrpn_Analog.h"

#include <iostream>
using namespace std;

void VRPN_CALLBACK handle_analog(void* userData, const vrpn_ANALOGCB a)
{
	int nbChannels = a.num_channel;

	cout << "Analog : ";

	for (int i = 0; i < a.num_channel; i++)
	{
		cout << a.channel[i] << " ";
	}

	cout << endl;
}

void VRPN_CALLBACK handle_button(void* userData, const vrpn_BUTTONCB b)
{
	cout << "Button '" << b.button << "': " << b.state << endl;
}


void VRPN_CALLBACK handle_tracker(void* userData, const vrpn_TRACKERCB t)
{
	cout << "Tracker '" << t.sensor << "' : " << t.pos[0] << "," << t.pos[1] << "," << t.pos[2] << endl;
}


int main(int argc, char* argv[])
{

	vrpn_Tracker_Remote* vrpnTracker = new vrpn_Tracker_Remote("Body_1@192.168.254.1");

	vrpnTracker->register_change_handler(0, handle_tracker);

	while (1)
	{
		vrpnTracker->mainloop();
	}

	return 0;
}