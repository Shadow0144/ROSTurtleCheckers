#pragma once

#include <QWidget>

#include <memory>
#include <string>
#include <vector>
#include <functional>

class TitleWidget : public QWidget
{
public:
    TitleWidget();
};

typedef std::shared_ptr<TitleWidget> TitleWidgetPtr;