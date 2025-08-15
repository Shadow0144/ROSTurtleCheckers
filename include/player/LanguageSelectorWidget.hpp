#pragma once

#include <QWidget>

#include "shared/CheckersConsts.hpp"

class LanguageSelectorWidget : public QWidget
{
    Q_OBJECT
public:
    LanguageSelectorWidget(QWidget *parent);

private slots:
    void onLanguageSelect(int index);
};