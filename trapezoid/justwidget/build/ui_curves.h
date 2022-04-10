/********************************************************************************
** Form generated from reading UI file 'curves.ui'
**
** Created by: Qt User Interface Compiler version 5.14.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CURVES_H
#define UI_CURVES_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_curves
{
public:

    void setupUi(QWidget *curves)
    {
        if (curves->objectName().isEmpty())
            curves->setObjectName(QString::fromUtf8("curves"));
        curves->resize(400, 300);

        retranslateUi(curves);

        QMetaObject::connectSlotsByName(curves);
    } // setupUi

    void retranslateUi(QWidget *curves)
    {
        curves->setWindowTitle(QCoreApplication::translate("curves", "Form", nullptr));
    } // retranslateUi

};

namespace Ui {
    class curves: public Ui_curves {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CURVES_H
