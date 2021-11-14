#include <iostream>

#include <Pipeline.h>

using namespace std;

#include "Application.h"
wxIMPLEMENT_APP(MyApp);
#include "MainFrame.h"

bool MyApp::OnInit()
{
    MyFrame *frame = new MyFrame( "Mesh simplification and skeleton extraction", wxPoint(50, 50), wxSize(800, 600) );
    frame->Show( true );
    Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(MyApp::OnIdle));
    return true;
}

void MyApp::OnIdle(wxIdleEvent& evt) {
        evt.RequestMore(); // render continuously, not only once on idle
}