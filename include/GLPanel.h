//
// Created by Edge on 2021/7/9.
//

#ifndef RAY_GLPANEL_H
#define RAY_GLPANEL_H

#include <wx/glcanvas.h>
#include "Pipeline.h"
#include "Simplification.h"

class GLPanel : public wxGLCanvas {
public:
    GLPanel(wxWindow *parent, const wxGLAttributes &canvasAttrs);

    ~GLPanel();

public:
    void OnSize(wxSizeEvent &event);

    void OnPaint(wxPaintEvent &event);

    void OnWheel(wxMouseEvent &event);

    void OnMouseMove(wxMouseEvent &event);

    void OnLeftMouseDown(wxMouseEvent &event);

    void OnLeftMouseUp(wxMouseEvent &event);

    void OnRightMouseDown(wxMouseEvent &event);

    void OnRightMouseUp(wxMouseEvent &event);

    void updated();

private:
    wxGLContext *m_GLContext;

    int m_winHeight;

    bool m_isLoaded = false;

    int m_prevMouseX, m_prevMouseY;
    unsigned char m_mouseState = 0;

    enum {
        NONE=0,
        LEFT_PRESS=1,
        RIGHT_PRESS=2,
    };

wxDECLARE_EVENT_TABLE();
};

#endif //RAY_GLPANEL_H
