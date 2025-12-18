/********************************************************************************
** Form generated from reading UI file 'pagepay_card.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGEPAY_CARD_H
#define UI_PAGEPAY_CARD_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_pagepay_card
{
public:
    QLabel *label;
    QLabel *label_2;
    QPushButton *pushButton;
    QTextEdit *textEdit;
    QLabel *label_3;
    QPushButton *pushButton_2;

    void setupUi(QWidget *pagepay_card)
    {
        if (pagepay_card->objectName().isEmpty())
            pagepay_card->setObjectName("pagepay_card");
        pagepay_card->resize(400, 300);
        label = new QLabel(pagepay_card);
        label->setObjectName("label");
        label->setGeometry(QRect(10, 10, 201, 17));
        label_2 = new QLabel(pagepay_card);
        label_2->setObjectName("label_2");
        label_2->setGeometry(QRect(10, 50, 211, 17));
        pushButton = new QPushButton(pagepay_card);
        pushButton->setObjectName("pushButton");
        pushButton->setGeometry(QRect(280, 240, 131, 21));
        textEdit = new QTextEdit(pagepay_card);
        textEdit->setObjectName("textEdit");
        textEdit->setGeometry(QRect(50, 120, 104, 70));
        label_3 = new QLabel(pagepay_card);
        label_3->setObjectName("label_3");
        label_3->setGeometry(QRect(290, 140, 121, 17));
        pushButton_2 = new QPushButton(pagepay_card);
        pushButton_2->setObjectName("pushButton_2");
        pushButton_2->setGeometry(QRect(300, 10, 171, 31));

        retranslateUi(pagepay_card);

        QMetaObject::connectSlotsByName(pagepay_card);
    } // setupUi

    void retranslateUi(QWidget *pagepay_card)
    {
        pagepay_card->setWindowTitle(QCoreApplication::translate("pagepay_card", "Form", nullptr));
        label->setText(QCoreApplication::translate("pagepay_card", "\354\213\240\354\232\251\354\271\264\353\223\234/\354\262\264\355\201\254\354\271\264\353\223\234 \352\262\260\354\240\234\355\225\230\352\270\260", nullptr));
        label_2->setText(QCoreApplication::translate("pagepay_card", "\354\225\204\353\236\230 \352\265\254\353\247\244 \353\202\264\354\227\255\354\235\204 \355\231\225\354\235\270\355\225\264\354\243\274\354\204\270\354\232\224", nullptr));
        pushButton->setText(QCoreApplication::translate("pagepay_card", "\353\213\244\354\235\214\354\234\274\353\241\234 \353\204\230\354\226\264\352\260\200\352\270\260", nullptr));
        textEdit->setHtml(QCoreApplication::translate("pagepay_card", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><meta charset=\"utf-8\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"hr { height: 1px; border-width: 0; }\n"
"li.unchecked::marker { content: \"\\2610\"; }\n"
"li.checked::marker { content: \"\\2612\"; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">\354\230\201\354\210\230\354\246\235 \353\247\271\355\202\244\353\241\234 \352\265\254\353\247\244\353\202\264\354\227\255</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", nullptr));
        label_3->setText(QCoreApplication::translate("pagepay_card", "\353\214\200\354\266\251 \354\271\264\353\223\234 \352\267\270\353\246\274", nullptr));
        pushButton_2->setText(QCoreApplication::translate("pagepay_card", "\354\240\204 \355\231\224\354\202\264\355\221\234 \355\221\234\354\213\234 \354\240\204\355\231\224\353\251\264 \353\217\214\354\225\204\352\260\220", nullptr));
    } // retranslateUi

};

namespace Ui {
    class pagepay_card: public Ui_pagepay_card {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGEPAY_CARD_H
