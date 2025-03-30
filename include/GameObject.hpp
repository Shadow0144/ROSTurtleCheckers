
#pragma once

#include "Render.hpp"

#include <QPainter>

class GameObject
{
protected:
    RenderPtr m_render;

public:
    void setRender(const RenderPtr &render)
    {
        m_render = render;
    }

    const RenderPtr &getRender() const
    {
        return render;
    }

    void paint(QPainter &painter)
    {
        if (render)
        {
            render->paint(painter);
        }
    }
};