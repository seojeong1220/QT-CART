/********************************************************************************
** Form generated from reading UI file 'pagepay.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGEPAY_H
#define UI_PAGEPAY_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PagePay
{
public:
    QLabel *label;
    QLabel *label_2;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;

    void setupUi(QWidget *PagePay)
    {
        if (PagePay->objectName().isEmpty())
            PagePay->setObjectName("PagePay");
        PagePay->resize(400, 300);
        label = new QLabel(PagePay);
        label->setObjectName("label");
        label->setGeometry(QRect(20, 10, 67, 17));
        label_2 = new QLabel(PagePay);
        label_2->setObjectName("label_2");
        label_2->setGeometry(QRect(10, 40, 291, 17));
        pushButton = new QPushButton(PagePay);
        pushButton->setObjectName("pushButton");
        pushButton->setGeometry(QRect(20, 120, 131, 25));
        pushButton_2 = new QPushButton(PagePay);
        pushButton_2->setObjectName("pushButton_2");
        pushButton_2->setGeometry(QRect(20, 150, 141, 25));
        pushButton_3 = new QPushButton(PagePay);
        pushButton_3->setObjectName("pushButton_3");
        pushButton_3->setGeometry(QRect(20, 180, 131, 25));
        pushButton_4 = new QPushButton(PagePay);
        pushButton_4->setObjectName("pushButton_4");
        pushButton_4->setGeometry(QRect(290, 10, 101, 25));

        retranslateUi(PagePay);

        QMetaObject::connectSlotsByName(PagePay);
    } // setupUi

    void retranslateUi(QWidget *PagePay)
    {
        PagePay->setWindowTitle(QCoreApplication::translate("PagePay", "Form", nullptr));
        label->setText(QCoreApplication::translate("PagePay", "\352\262\260\354\240\234\355\225\230\352\270\260", nullptr));
        label_2->setText(QCoreApplication::translate("PagePay", "\354\233\220\355\225\230\354\213\234\353\212\224 \352\262\260\354\240\234 \354\210\230\353\213\250\354\235\204 \354\204\240\355\203\235\355\225\264\354\243\274\354\204\270\354\232\224", nullptr));
        pushButton->setText(QCoreApplication::translate("PagePay", "\354\213\240\354\232\251\354\271\264\353\223\234/\354\262\264\355\201\254\354\271\264\353\223\234", nullptr));
        pushButton_2->setText(QCoreApplication::translate("PagePay", "\354\202\274\354\204\261\355\216\230\354\235\264/\354\271\264\354\271\264\354\230\244\355\216\230\354\235\264", nullptr));
        pushButton_3->setText(QCoreApplication::translate("PagePay", "\353\251\244\353\262\204\354\213\255 \355\225\240\354\235\270 \354\240\201\354\232\251", nullptr));
        pushButton_4->setText(QCoreApplication::translate("PagePay", "\355\231\210\354\234\274\353\241\234 \353\217\214\354\225\204\352\260\200\352\270\260", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PagePay: public Ui_PagePay {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGEPAY_H
