#pragma once

#include <QWidget>
#include <QComboBox>

#include "shared/CheckersConsts.hpp"

class LanguageSelectorWidget : public QWidget
{
    Q_OBJECT
public:
    LanguageSelectorWidget(QWidget *parent);

    void setCurrentIndex(int index);

private slots:
    void onLanguageSelect(int index);

private:
    QComboBox *m_languageComboBox;
};