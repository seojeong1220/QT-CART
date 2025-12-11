/********************************************************************************
** Form generated from reading UI file 'pageguide.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PAGEGUIDE_H
#define UI_PAGEGUIDE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PageGuide
{
public:
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *labelGuideTitle;
    QSpacerItem *horizontalSpacer;
    QPushButton *btnBackToCart;
    QLabel *labelMap;

    void setupUi(QWidget *PageGuide)
    {
        if (PageGuide->objectName().isEmpty())
            PageGuide->setObjectName("PageGuide");
        PageGuide->resize(552, 393);
        horizontalLayout_2 = new QHBoxLayout(PageGuide);
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        labelGuideTitle = new QLabel(PageGuide);
        labelGuideTitle->setObjectName("labelGuideTitle");

        horizontalLayout->addWidget(labelGuideTitle);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        btnBackToCart = new QPushButton(PageGuide);
        btnBackToCart->setObjectName("btnBackToCart");
        btnBackToCart->setCheckable(true);

        horizontalLayout->addWidget(btnBackToCart);


        verticalLayout->addLayout(horizontalLayout);

        labelMap = new QLabel(PageGuide);
        labelMap->setObjectName("labelMap");
        labelMap->setPixmap(QPixmap(QString::fromUtf8("../../house.pgm")));
        labelMap->setScaledContents(true);

        verticalLayout->addWidget(labelMap);

        verticalLayout->setStretch(0, 2);
        verticalLayout->setStretch(1, 8);

        horizontalLayout_2->addLayout(verticalLayout);


        retranslateUi(PageGuide);

        QMetaObject::connectSlotsByName(PageGuide);
    } // setupUi

    void retranslateUi(QWidget *PageGuide)
    {
        PageGuide->setWindowTitle(QCoreApplication::translate("PageGuide", "Form", nullptr));
        labelGuideTitle->setText(QCoreApplication::translate("PageGuide", "Instruction Mode", nullptr));
        btnBackToCart->setText(QCoreApplication::translate("PageGuide", "\354\236\245\353\260\224\352\265\254\353\213\210\353\241\234", nullptr));
        labelMap->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class PageGuide: public Ui_PageGuide {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PAGEGUIDE_H
