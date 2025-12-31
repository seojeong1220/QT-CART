#include "pagepay_card.h"
#include "ui_pagepay_card.h"

#include <QDialog>
#include <QFrame>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QStackedWidget>
#include <QTimer>
#include <QShowEvent>
#include <QResizeEvent>

pagepay_card::pagepay_card(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::pagepay_card)
{
    ui->setupUi(this);
}

pagepay_card::~pagepay_card()
{
    delete ui;
}

void pagepay_card::showEvent(QShowEvent *e)
{
    QWidget::showEvent(e);

    // 페이지가 화면에 뜬 뒤에 모달 1번만 띄우기
    QTimer::singleShot(0, this, [this]() { showPaymentModal(); });

}

void pagepay_card::showPaymentModal()
{
    QWidget *overlayParent = this->window();
    if (!overlayParent) overlayParent = this;

    // ✅ 프레임리스/모달 다이얼로그
    QDialog *dlg = new QDialog(overlayParent);
    dlg->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    dlg->setModal(true);
    dlg->setAttribute(Qt::WA_TranslucentBackground);

    // ✅ 부모창을 덮도록 위치/크기 맞추기
    QPoint topLeft = overlayParent->mapToGlobal(QPoint(0,0));
    dlg->setGeometry(QRect(topLeft, overlayParent->size()));

    // ===== 배경(반투명) =====
    QFrame *bg = new QFrame(dlg);
    bg->setStyleSheet("QFrame { background-color: rgba(0,0,0,128); }");
    bg->setGeometry(dlg->rect());

    QVBoxLayout *rootLay = new QVBoxLayout(bg);
    rootLay->setContentsMargins(0,0,0,0);

    // ===== 가운데 카드 =====
    QFrame *card = new QFrame(bg);
    card->setFixedSize(420, 360);
    card->setStyleSheet("QFrame { background: white; border-radius: 16px; }");

    rootLay->addStretch();
    rootLay->addWidget(card, 0, Qt::AlignHCenter);
    rootLay->addStretch();

    // ===== 카드 내부: processing/complete 전환 =====
    QStackedWidget *stack = new QStackedWidget(card);
    QVBoxLayout *cardLay = new QVBoxLayout(card);
    cardLay->setContentsMargins(24,24,24,24);
    cardLay->addWidget(stack);

    // --- (1) processing 위젯 ---
    QWidget *wProc = new QWidget(card);
    QVBoxLayout *pLay = new QVBoxLayout(wProc);
    pLay->setContentsMargins(0,0,0,0);
    pLay->setSpacing(12);

    QLabel *spinner = new QLabel("⏳", wProc);
    spinner->setAlignment(Qt::AlignCenter);
    spinner->setStyleSheet("font-size: 52px;");

    QLabel *title = new QLabel("카드를 뽑지말아주세요", wProc);
    title->setAlignment(Qt::AlignCenter);
    title->setStyleSheet("font-size: 22px; font-weight: 800; color: #111827;");

    QLabel *desc = new QLabel("결제가 진행중입니다", wProc);
    desc->setAlignment(Qt::AlignCenter);
    desc->setStyleSheet("font-size: 14px; color: #6B7280;");

    QLabel *countLbl = new QLabel("5", wProc);
    countLbl->setAlignment(Qt::AlignCenter);
    countLbl->setStyleSheet("font-size: 64px; font-weight: 900; color: #2563EB; margin-top: 6px;");

    pLay->addStretch();
    pLay->addWidget(spinner);
    pLay->addWidget(title);
    pLay->addWidget(desc);
    pLay->addWidget(countLbl);
    pLay->addStretch();

    // --- (2) complete 위젯 ---
    QWidget *wDone = new QWidget(card);
    QVBoxLayout *dLay = new QVBoxLayout(wDone);
    dLay->setContentsMargins(0,0,0,0);
    dLay->setSpacing(16);

    QLabel *check = new QLabel("✓", wDone);
    check->setAlignment(Qt::AlignCenter);
    check->setStyleSheet("font-size: 72px; font-weight: 900; color: #22C55E;");

    QLabel *doneTxt = new QLabel("결제가 완료되었습니다", wDone);
    doneTxt->setAlignment(Qt::AlignCenter);
    doneTxt->setStyleSheet("font-size: 22px; font-weight: 800; color: #111827;");

    QPushButton *btnOk = new QPushButton("확인", wDone);
    btnOk->setFixedHeight(44);
    btnOk->setStyleSheet(
        "QPushButton{background:#2563EB;color:white;border:none;border-radius:12px;font-size:16px;font-weight:800;}"
        "QPushButton:hover{background:#1D4ED8;}"
        "QPushButton:pressed{background:#1E40AF;}"
        );

    dLay->addStretch();
    dLay->addWidget(check);
    dLay->addWidget(doneTxt);
    dLay->addWidget(btnOk, 0, Qt::AlignHCenter);
    dLay->addStretch();

    stack->addWidget(wProc);
    stack->addWidget(wDone);
    stack->setCurrentWidget(wProc);

    // ===== 5초 카운트다운 =====
    int *count = new int(5);            // dlg 종료까지 살아있게 heap
    QTimer *t = new QTimer(dlg);
    t->setInterval(1000);

    QObject::connect(t, &QTimer::timeout, dlg, [=]() mutable {
        (*count)--;
        if (*count >= 0) countLbl->setText(QString::number(*count));

        if (*count <= 0) {
            t->stop();
            stack->setCurrentWidget(wDone);
        }
    });

    // ===== 확인 누르면 totalpay로 =====
    QObject::connect(btnOk, &QPushButton::clicked, dlg, [this, dlg, count]() {
        delete count;
        dlg->accept();
        emit goTotalPayClicked();   // ✅ MainWidget에서 pagetotalpay로 이동
    });

    t->start();
    dlg->exec(); // 모달 실행
}
void pagepay_card::resizeEvent(QResizeEvent *e)
{
    QWidget::resizeEvent(e);
    // (필요하면 여기서 모달 크기 맞추는 로직 추가 가능)
}
