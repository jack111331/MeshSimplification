//
// Created by Edge on 2021/7/9.
//
#include <wx/splitter.h>
#include <glad/glad.h>
#include <Context.h>
#include <PhongShadingPipeline.h>
#include "wx/wx.h"
#include "MainFrame.h"
#include "Application.h"

//#define USE_FILE_LOG
#define USE_WINDOW_LOG

wxBEGIN_EVENT_TABLE(MyFrame, wxFrame)
                EVT_MENU(ID_Hello, MyFrame::OnHello)
                EVT_MENU(MENU_Exit, MyFrame::OnExit)
                EVT_MENU(MENU_About, MyFrame::OnAbout)
                EVT_MENU(MENU_Open, MyFrame::OpenFile)
                EVT_MENU(MENU_OpenMesh, MyFrame::OpenMeshFile)
                EVT_COMMAND_SCROLL(SLIDER_Simplification, MyFrame::OnSimplification)
                EVT_BUTTON (BUTTON_Render, MyFrame::OnRender)
                EVT_BUTTON (BUTTON_Skeleton, MyFrame::OnSkeleton)
                EVT_BUTTON (BUTTON_SkeletonToLine, MyFrame::OnSkeletonToLine)
wxEND_EVENT_TABLE()

MyFrame::MyFrame(const wxString &title, const wxPoint &pos, const wxSize &size)
        : wxFrame(NULL, wxID_ANY, title, pos, size) {
    wxMenu *menuFile = new wxMenu;
    menuFile->Append(ID_Hello, "&Hello...\tCtrl-H",
                     "Help string shown in status bar for this menu item");
    menuFile->AppendSeparator();
    menuFile->Append(MENU_Open, "&Open",
                     "Open config file");
    menuFile->AppendSeparator();
    menuFile->Append(MENU_OpenMesh, "&Open Mesh",
                     "Open mesh file");
    menuFile->AppendSeparator();
    menuFile->Append(MENU_Exit, "&Exit", "Exit program");

    wxMenu *menuHelp = new wxMenu;
    menuHelp->Append(MENU_About, "About");
    wxMenuBar *menuBar = new wxMenuBar;
    menuBar->Append(menuFile, "&File");
    menuBar->Append(menuHelp, "&Help");
    SetMenuBar(menuBar);
    CreateStatusBar();
    SetStatusText("Welcome to wxWidgets!");

    wxGLAttributes vAttrs;
    vAttrs.PlatformDefaults().Defaults().EndList();
    wxSplitterWindow *splitter = new wxSplitterWindow(this);
    m_3DView = new GLPanel(splitter, vAttrs);

    m_leftPanel = new wxPanel(splitter);
    wxStaticText *SimplificationLabel = new wxStaticText(m_leftPanel, wxID_ANY, "Simplification level", wxPoint(20, 50));
    m_slider = new wxSlider(m_leftPanel, SLIDER_Simplification, 0, 0, 10,
                            wxPoint(230, 50), wxSize(140, -1));

    m_render = new wxButton(m_leftPanel, BUTTON_Render, _T("Render"),
            // shows a button on this window
                            wxPoint(40, 5), wxSize(50, -1), 0);

    wxStaticText *laplaceTimesLabel = new wxStaticText(m_leftPanel, wxID_ANY, "Times", wxPoint(20, 90));
    m_laplaceTimes = new wxTextCtrl(m_leftPanel, TEXT_Field,
                                                wxT("12"), wxPoint(100, 90), wxDefaultSize,
                                                wxTE_RICH , wxDefaultValidator, wxTextCtrlNameStr);
    wxStaticText *wHLabel = new wxStaticText(m_leftPanel, wxID_ANY, "Initial W_H", wxPoint(20, 120));
    m_wH = new wxTextCtrl(m_leftPanel, TEXT_Field,
                                      wxT("1.0"), wxPoint(100, 120), wxDefaultSize,
                                      wxTE_RICH , wxDefaultValidator, wxTextCtrlNameStr);
    wxStaticText *wLLabel = new wxStaticText(m_leftPanel, wxID_ANY, "W_L factor", wxPoint(20, 150));
    m_wL = new wxTextCtrl(m_leftPanel, TEXT_Field,
                          wxT("4.5"), wxPoint(100, 150), wxDefaultSize,
                          wxTE_RICH , wxDefaultValidator, wxTextCtrlNameStr);

    m_skeleton = new wxButton(m_leftPanel, BUTTON_Skeleton, _T("Skeleton"),
            // shows a button on this window
                              wxPoint(250, 120), wxSize(80, -1), 0);

    wxStaticText *wALabel = new wxStaticText(m_leftPanel, wxID_ANY, "W_A", wxPoint(20, 220));
    m_wA = new wxTextCtrl(m_leftPanel, TEXT_Field,
                                      wxT("1.0"), wxPoint(100, 220), wxDefaultSize,
                                      wxTE_RICH , wxDefaultValidator, wxTextCtrlNameStr);
    wxStaticText *wBLabel = new wxStaticText(m_leftPanel, wxID_ANY, "W_B", wxPoint(20, 250));
    m_wB = new wxTextCtrl(m_leftPanel, TEXT_Field,
                                      wxT("0.1"), wxPoint(100, 250), wxDefaultSize,
                                      wxTE_RICH , wxDefaultValidator, wxTextCtrlNameStr);


    m_skeletonToLine = new wxButton(m_leftPanel, BUTTON_SkeletonToLine, _T("Skeleton To Line"),
            // shows a button on this window
                              wxPoint(250, 235), wxSize(130, -1), 0);

    m_log = new wxTextCtrl(m_leftPanel, wxID_ANY, wxString(), wxPoint(5, 350),
                           wxSize(400, 200), wxTE_MULTILINE);
    m_log->SetMinSize(wxSize(-1, 100));
#ifdef USE_WINDOW_LOG
    m_logOld = wxLog::SetActiveTarget(new wxLogTextCtrl(m_log));
#endif

#ifdef USE_FILE_LOG
    m_logFile.open("log.txt", std::ios_base::out);
    m_logFileStream = new wxLogStream(&m_logFile);
    wxLog::SetActiveTarget(m_logFileStream);
#endif

    wxLogMessage("This is the log window");


    splitter->SetSize(GetClientSize());
    splitter->SetSashGravity(0.5);
    splitter->SplitVertically(m_leftPanel, m_3DView);

    Context *context = Context::getInstance();
    Pipeline *newPipeline = new PhongShadingPipeline();
    context->setPipeline(newPipeline);
    newPipeline->setupEnvironment();
    newPipeline->setupPipeline();


    m_mesh = nullptr;

}

MyFrame::~MyFrame() {
#ifdef USE_WINDOW_LOG
    delete wxLog::SetActiveTarget(m_logOld);
#endif
#ifdef USE_FILE_LOG
    delete wxLog::SetActiveTarget(m_logFileStream);
    m_logFile.close();
#endif
}


SimplificationThread *MyFrame::CreateSimplificationThread() {
    SimplificationThread *thread = new SimplificationThread(m_meshManager, m_3DView, m_slider);

    if (thread->Create() != wxTHREAD_NO_ERROR) {
        wxLogError("Can't create thread!");
    }

    wxGetApp().m_threads.Add(thread);

    return thread;
}

SkeletonToLineThread *MyFrame::CreateSkeletonToLineThread() {
    SkeletonToLineThread *thread = new SkeletonToLineThread(m_skeletonExtraction, m_3DView, m_skeletonToLine);

    if (thread->Create() != wxTHREAD_NO_ERROR) {
        wxLogError("Can't create thread!");
    }

    wxGetApp().m_threads.Add(thread);

    return thread;
}

void MyFrame::OnExit(wxCommandEvent &event) {
    Close(true);
}

void MyFrame::OnAbout(wxCommandEvent &event) {
    wxMessageBox("This is a wxWidgets' Hello world sample",
                 "About Hello World", wxOK | wxICON_INFORMATION);
}

void MyFrame::OnHello(wxCommandEvent &event) {
    wxLogMessage("Hello world from wxWidgets!");
}

void MyFrame::OnSkeleton(wxCommandEvent &event) {
//    int times;
//    m_laplaceTimes->GetValue();
//    times = wxAtoi(m_laplaceTimes->GetValue());
//    wxLogMessage("times=%d", times);
// 14 for 4.5 0~13 14 1.1
// 12 for 5
// 11 for 6

// 9
    for (int i = 0;i < 9;++i) {
        m_skeletonExtraction->calculateSkeleton();
    }
    m_mesh = m_skeletonExtraction->getCurrentMesh();
    Context *context = Context::getInstance();
    context->getPipeline()->m_objectList[0]->m_mesh = m_mesh;

    m_mesh->updateVAO();
    m_3DView->updated();
}

void MyFrame::OnSkeletonToLine(wxCommandEvent &event) {

//    const wxArrayThread& threads = wxGetApp().m_threads;
//    size_t count = threads.GetCount();
//    if (!count) {
//        m_skeletonToLine->Enable(false);
//
//        SkeletonToLineThread *thread = CreateSkeletonToLineThread();
//
//        if (thread->Run() != wxTHREAD_NO_ERROR) {
//            wxLogError("Can't start thread!");
//        }
//    }

    m_skeletonExtraction->simplifyMesh();
    m_mesh = m_skeletonExtraction->getCurrentMesh();
    Context *context = Context::getInstance();
    context->getPipeline()->m_objectList[0]->m_mesh = m_mesh;

//    m_mesh->updateVAO();
    m_3DView->updated();
}

void MyFrame::OnSimplification(wxScrollEvent &event) {
    if (m_mesh) {
        wxLogMessage("Current slider: %d", event.GetInt());
//        // TODO create working thread
//        const wxArrayThread& threads = wxGetApp().m_threads;
//        size_t count = threads.GetCount();
//        if (!count) {
//            SimplificationThread *thread = CreateSimplificationThread();
//
//            if (thread->Run() != wxTHREAD_NO_ERROR) {
//                wxLogError("Can't start thread!");
//            }
//        }

        while (m_meshManager->m_currentMeshIdx < event.GetInt()) {
            m_meshManager->lowerMeshLevel();
        }
        while (m_meshManager->m_currentMeshIdx > event.GetInt()) {
            m_meshManager->higherMeshLevel();
        }
        m_mesh = m_meshManager->getCurrentMesh();
        Context *context = Context::getInstance();
        context->getPipeline()->m_objectList[0]->m_mesh = m_mesh;

        m_3DView->updated();
    }
}

void MyFrame::OnRender(wxCommandEvent &event) {
    const wxSize ClientSize = m_3DView->GetClientSize();
    glViewport(0, 0, ClientSize.x, ClientSize.y);
    Pipeline *pipeline = Context::getInstance()->getPipeline();
    if (pipeline) {
        pipeline->m_camera->m_width = ClientSize.x;
        pipeline->m_camera->m_height = ClientSize.y;
    }
    if (pipeline) {
        pipeline->pipelineRender();
    }
    m_3DView->updated();
}

void MyFrame::OpenFile(wxCommandEvent & WXUNUSED(event)) {
    wxFileDialog *OpenDialog = new wxFileDialog(
            this, _("Choose a file to open"), wxEmptyString, wxEmptyString,
            _("YAML files (*.yml)|*.yml"),
            wxFD_OPEN, wxDefaultPosition);

    // Creates a "open file" dialog with 4 file types
    if (OpenDialog->ShowModal() == wxID_OK) // if the user click "Open" instead of "cancel"
    {

    }
}

void MyFrame::OpenMeshFile(wxCommandEvent & WXUNUSED(event)) {
    wxFileDialog *OpenDialog = new wxFileDialog(
            this, _("Choose a file to open"), wxEmptyString, wxEmptyString,
            _("OBJ files (*.obj)|*.obj"),
            wxFD_OPEN, wxDefaultPosition);

    // Creates a "open file" dialog with 4 file types
    if (OpenDialog->ShowModal() == wxID_OK) // if the user click "Open" instead of "cancel"
    {
        std::string filepath(OpenDialog->GetPath());
        OpenMesh::IO::Options opt;
        if (m_mesh == nullptr) {
            m_mesh = new Tri_Mesh();
        } else {
            delete m_mesh;
            m_mesh = new Tri_Mesh();
        }
        if (OpenMesh::IO::read_mesh(*m_mesh, filepath, opt)) {
            // If the file did not provide vertex normals and mesh has vertex normal ,then calculate them
            m_mesh->request_vertex_status();
            m_mesh->request_edge_status();
            m_mesh->request_face_status();
            if (!opt.check(OpenMesh::IO::Options::VertexNormal) && m_mesh->has_vertex_normals()) {
                m_mesh->update_normals();
            }
            wxLogMessage("Loaded mesh.");
        }
        Context *context = Context::getInstance();
        context->m_mesh = m_mesh;
        m_mesh->createVAO();
        m_meshManager = new MeshManager(m_mesh);
        m_skeletonExtraction = new SkeletonExtraction(m_mesh);

        context->getPipeline()->m_objectList.push_back(new ShadeObject(m_mesh));

    }
}

SimplificationThread::SimplificationThread(MeshManager *meshManager, GLPanel *view, wxSlider *slider)
        : wxThread() {
    m_meshManager = meshManager;
    m_3DView = view;
    m_slider = slider;
}

SimplificationThread::~SimplificationThread() {

    wxArrayThread &threads = wxGetApp().m_threads;
    threads.Remove(this);
}

wxThread::ExitCode SimplificationThread::Entry() {
    {
        wxCriticalSectionLocker locker(wxGetApp().m_critsect);
        wxLogMessage("Thread started (priority = %u).", GetPriority());

        while (m_meshManager->m_currentMeshIdx != m_slider->GetValue()) {
            while (m_meshManager->m_currentMeshIdx < m_slider->GetValue()) {
                m_meshManager->lowerMeshLevel();
            }
            while (m_meshManager->m_currentMeshIdx > m_slider->GetValue()) {
                m_meshManager->higherMeshLevel();
            }
        }
        Tri_Mesh *mesh = m_meshManager->getCurrentMesh();
        Context *context = Context::getInstance();
        context->getPipeline()->m_objectList[0]->m_mesh = mesh;

        mesh->updateVAO();
        m_3DView->updated();

        wxLogMessage("Thread finished.");

    }

    return NULL;
}

SkeletonToLineThread::SkeletonToLineThread(SkeletonExtraction *skeletonExtractor, GLPanel *view, wxButton *button)
        : wxThread() {
    m_skeletonExtractor = skeletonExtractor;
    m_3DView = view;
    m_button = button;
}

SkeletonToLineThread::~SkeletonToLineThread() {

    wxArrayThread &threads = wxGetApp().m_threads;
    threads.Remove(this);
}

wxThread::ExitCode SkeletonToLineThread::Entry() {
    {
        wxCriticalSectionLocker locker(wxGetApp().m_critsect);
        wxLogMessage("Thread started (priority = %u).", GetPriority());

        m_skeletonExtractor->simplifyMesh();

        Tri_Mesh *mesh = m_skeletonExtractor->getCurrentMesh();
        Context *context = Context::getInstance();
        context->getPipeline()->m_objectList[0]->m_mesh = mesh;

        m_3DView->updated();
        m_button->Enable(true);

        wxLogMessage("Thread finished.");

    }

    return NULL;
}
