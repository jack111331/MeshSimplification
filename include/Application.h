//
// Created by Edge on 2021/7/8.
//

#ifndef RAY_APPLICATION_H
#define RAY_APPLICATION_H

#include <wx/wxprec.h>
#include <wx/string.h>
#include <wx/menu.h>
#include <wx/textctrl.h>
#include "wx/thread.h"
#include "wx/dynarray.h"
#ifndef WX_PRECOMP
#include <wx/wx.h>


#endif

WX_DEFINE_ARRAY_PTR(wxThread *, wxArrayThread);

class MyApp: public wxApp
{
public:
    virtual bool OnInit() wxOVERRIDE;
    void OnIdle(wxIdleEvent& evt);

    // critical section protects access to all of the fields below
    wxCriticalSection m_critsect;

    // all the threads currently alive - as soon as the thread terminates, it's
    // removed from the array
    wxArrayThread m_threads;
};

DECLARE_APP ( MyApp );


#endif //RAY_APPLICATION_H
