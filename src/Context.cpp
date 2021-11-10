//
// Created by Edge on 2021/7/12.
//

#include <wx/wx.h>
#include "Context.h"

Context *Context::m_instance = nullptr;

Context *Context::getInstance() {
    if (Context::m_instance == nullptr) {
        Context::m_instance = new Context();

    }
    return Context::m_instance;
}
