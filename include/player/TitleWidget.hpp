#pragma once

#include <QWidget>
#include <QLabel>

#include <memory>
#include <string>
#include <vector>
#include <functional>

class TitleWidget : public QWidget
{
public:
    TitleWidget();

    void reloadStrings();

private:
    QLabel *m_titleLabel;
};

typedef std::shared_ptr<TitleWidget> TitleWidgetPtr;