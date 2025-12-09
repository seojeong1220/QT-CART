/********************************************************************************
** Form generated from reading UI file 'pagecart.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGECART_H
#define UI_PAGECART_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PageCart
{
public:
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *labelTitle;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *btnGuideMode;
    QHBoxLayout *horizontalLayout_2;
    QTableWidget *tableCart;
    QHBoxLayout *horizontalLayout_3;
    QLabel *labelTotalPrice;
    QLabel *labelTotalPriceValue;
    QSpacerItem *horizontalSpacer;
    QPushButton *btnPay;

    void setupUi(QWidget *PageCart)
    {
        if (PageCart->objectName().isEmpty())
            PageCart->setObjectName(QString::fromUtf8("PageCart"));
        PageCart->resize(624, 469);
        verticalLayout_2 = new QVBoxLayout(PageCart);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));

        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        labelTitle = new QLabel(PageCart);
        labelTitle->setObjectName(QString::fromUtf8("labelTitle"));

        horizontalLayout_4->addWidget(labelTitle);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_4->addItem(horizontalSpacer_2);

        btnGuideMode = new QPushButton(PageCart);
        btnGuideMode->setObjectName(QString::fromUtf8("btnGuideMode"));

        horizontalLayout_4->addWidget(btnGuideMode);

        horizontalLayout_4->setStretch(0, 1);
        horizontalLayout_4->setStretch(1, 4);
        horizontalLayout_4->setStretch(2, 1);

        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        tableCart = new QTableWidget(PageCart);
        if (tableCart->columnCount() < 6)
            tableCart->setColumnCount(6);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(4, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        tableCart->setHorizontalHeaderItem(5, __qtablewidgetitem5);
        tableCart->setObjectName(QString::fromUtf8("tableCart"));

        horizontalLayout_2->addWidget(tableCart);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        labelTotalPrice = new QLabel(PageCart);
        labelTotalPrice->setObjectName(QString::fromUtf8("labelTotalPrice"));

        horizontalLayout_3->addWidget(labelTotalPrice);

        labelTotalPriceValue = new QLabel(PageCart);
        labelTotalPriceValue->setObjectName(QString::fromUtf8("labelTotalPriceValue"));

        horizontalLayout_3->addWidget(labelTotalPriceValue);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        btnPay = new QPushButton(PageCart);
        btnPay->setObjectName(QString::fromUtf8("btnPay"));

        horizontalLayout_3->addWidget(btnPay);

        horizontalLayout_3->setStretch(0, 2);
        horizontalLayout_3->setStretch(1, 2);
        horizontalLayout_3->setStretch(2, 5);
        horizontalLayout_3->setStretch(3, 3);

        verticalLayout->addLayout(horizontalLayout_3);


        verticalLayout_2->addLayout(verticalLayout);


        retranslateUi(PageCart);

        QMetaObject::connectSlotsByName(PageCart);
    } // setupUi

    void retranslateUi(QWidget *PageCart)
    {
        PageCart->setWindowTitle(QCoreApplication::translate("PageCart", "Form", nullptr));
        labelTitle->setText(QCoreApplication::translate("PageCart", "Shopping List", nullptr));
        btnGuideMode->setText(QCoreApplication::translate("PageCart", "\354\225\210\353\202\264\353\252\250\353\223\234", nullptr));
        QTableWidgetItem *___qtablewidgetitem = tableCart->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("PageCart", "\354\203\201\355\222\210\353\252\205", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = tableCart->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("PageCart", "\352\260\234\354\210\230", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = tableCart->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("PageCart", "+", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = tableCart->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("PageCart", "-", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = tableCart->horizontalHeaderItem(4);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("PageCart", "\352\270\210\354\225\241", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = tableCart->horizontalHeaderItem(5);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("PageCart", "\354\202\255\354\240\234", nullptr));
        labelTotalPrice->setText(QCoreApplication::translate("PageCart", "\354\264\235\354\225\241 : ", nullptr));
        labelTotalPriceValue->setText(QCoreApplication::translate("PageCart", "0\354\233\220", nullptr));
        btnPay->setText(QCoreApplication::translate("PageCart", "\352\262\260\354\240\234", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PageCart: public Ui_PageCart {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGECART_H
