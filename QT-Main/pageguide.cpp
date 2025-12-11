#include "pageguide.h"
#include "ui_pageguide.h"

PageGuide::PageGuide(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::PageGuide)
{
    ui->setupUi(this);
    connect(ui->btnBackToCart, SIGNAL(clicked()), this, SLOT(on_btnBackToCart_clicked()));
}

PageGuide::~PageGuide()
{
    delete ui;
}


void PageGuide::on_btnBackToCart_clicked()
{
    emit backToCartClicked();
}

