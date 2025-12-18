/********************************************************************************
** Form generated from reading UI file 'pagepayment.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGEPAYMENT_H
#define UI_PAGEPAYMENT_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PagePayment
{
public:
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;

    void setupUi(QWidget *PagePayment)
    {
        if (PagePayment->objectName().isEmpty())
            PagePayment->setObjectName("PagePayment");
        PagePayment->resize(400, 300);
        verticalLayoutWidget = new QWidget(PagePayment);
        verticalLayoutWidget->setObjectName("verticalLayoutWidget");
        verticalLayoutWidget->setGeometry(QRect(230, 140, 160, 80));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName("verticalLayout");
        verticalLayout->setContentsMargins(0, 0, 0, 0);

        retranslateUi(PagePayment);

        QMetaObject::connectSlotsByName(PagePayment);
    } // setupUi

    void retranslateUi(QWidget *PagePayment)
    {
        PagePayment->setWindowTitle(QCoreApplication::translate("PagePayment", "Form", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PagePayment: public Ui_PagePayment {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGEPAYMENT_H
