#include "ccart.h"
#include "ui_ccart.h"

#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QPixmap>
#include <QLocale>
#include <QSpacerItem>

ccart::ccart(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ccart)
{
    ui->setupUi(this);

    setupScrollCart();

    // ✅ 실행 시 기본으로 3개 올라오게
    addCartRow("헬로키티 인형", ":/new/prefix1/kitty.jpg", 12000, 1);
    addCartRow("불닭볶음면 컵", ":/new/prefix1/buldak.jpg", 2000, 1);
    addCartRow("바나나",        ":/new/prefix1/banana.jpg", 1500, 1);
}

ccart::~ccart()
{
    delete ui;
}

QString ccart::moneyKR(int v)
{
    QLocale ko("ko_KR");
    return ko.toString(v) + "원";
}

void ccart::setupScrollCart()
{
    // ✅ product_items = QScrollArea
    ui->product_items->setWidgetResizable(true);

    // ScrollArea 안에 들어갈 컨테이너 위젯 + 세로 레이아웃
    m_container = new QWidget(ui->product_items);
    m_container->setObjectName("cart_container");

    m_vbox = new QVBoxLayout(m_container);
    m_vbox->setContentsMargins(0, 0, 0, 0);
    m_vbox->setSpacing(10);

    // 맨 아래로 몰리게 스페이서(선택이지만 보기 좋음)
    m_vbox->addStretch();

    ui->product_items->setWidget(m_container);
}

void ccart::addCartRow(const QString& name, const QString& imgPath, int unitPrice, int qty)
{
    QWidget *row = buildRowWidget(name, imgPath, unitPrice, qty);

    // addStretch() 바로 위에 끼워넣기
    int insertPos = m_vbox->count() - 1; // 마지막은 stretch
    m_vbox->insertWidget(insertPos, row);
}

QWidget* ccart::buildRowWidget(const QString& name, const QString& imgPath, int unitPrice, int qty)
{
    QWidget *row = new QWidget(m_container);
    row->setObjectName("cart_row");
    row->setStyleSheet(
        "QWidget#cart_row {"
        " background: #ffffff;"
        " border: 1px solid #e5e7eb;"
        " border-radius: 14px;"
        "}"
        );

    QHBoxLayout *h = new QHBoxLayout(row);
    h->setContentsMargins(12, 10, 12, 10);
    h->setSpacing(12);

    // 1) 이미지
    QLabel *img = new QLabel(row);
    img->setFixedSize(44, 44);
    img->setStyleSheet("background:#f3f4f6; border-radius:12px;");
    QPixmap px(imgPath);
    if (!px.isNull()) {
        img->setPixmap(px.scaled(44, 44, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
    }

    // 2) 상품명
    QLabel *lblName = new QLabel(name, row);
    lblName->setStyleSheet("font-size:13px; font-weight:800; color:#111827;");

    // 3) - 버튼
    QPushButton *btnMinus = new QPushButton("-", row);
    btnMinus->setFixedSize(32, 32);
    btnMinus->setStyleSheet(
        "QPushButton{border:1px solid #d1d5db; border-radius:8px; background:white; font-weight:900;}"
        "QPushButton:pressed{background:#f3f4f6;}"
        );

    // 4) 수량 라벨
    QLabel *lblQty = new QLabel(QString::number(qty), row);
    lblQty->setFixedWidth(28);
    lblQty->setAlignment(Qt::AlignCenter);
    lblQty->setStyleSheet("font-size:13px; font-weight:900; color:#111827;");

    // 5) + 버튼
    QPushButton *btnPlus = new QPushButton("+", row);
    btnPlus->setFixedSize(32, 32);
    btnPlus->setStyleSheet(
        "QPushButton{border:1px solid #d1d5db; border-radius:8px; background:white; font-weight:900;}"
        "QPushButton:pressed{background:#f3f4f6;}"
        );

    // 6) 가격 라벨 (단가 * 수량)
    QLabel *lblPrice = new QLabel(moneyKR(unitPrice * qty), row);
    lblPrice->setMinimumWidth(90);
    lblPrice->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    lblPrice->setStyleSheet("font-size:13px; font-weight:900; color:#111827;");

    // 7) 취소 버튼
    QPushButton *btnCancel = new QPushButton("취소", row);
    btnCancel->setFixedHeight(32);
    btnCancel->setStyleSheet(
        "QPushButton{border:none; border-radius:10px; background:#111827; color:white; font-weight:900; padding:0 14px;}"
        "QPushButton:pressed{background:#374151;}"
        );

    // 레이아웃 배치
    h->addWidget(img);
    h->addWidget(lblName);
    h->addStretch();
    h->addWidget(btnMinus);
    h->addWidget(lblQty);
    h->addWidget(btnPlus);
    h->addSpacing(8);
    h->addWidget(lblPrice);
    h->addSpacing(8);
    h->addWidget(btnCancel);

    // ✅ 여기서 제일 중요한 포인트:
    // row 내부에 상태를 "property"로 저장해두면, 찾기/배열 관리 없이도 안정적으로 갱신 가능
    row->setProperty("unitPrice", unitPrice);

    // + 클릭: qty 증가 후 price 갱신
    connect(btnPlus, &QPushButton::clicked, this, [row, lblQty, lblPrice]() {
        int unit = row->property("unitPrice").toInt();
        int q = lblQty->text().toInt();
        q++;
        lblQty->setText(QString::number(q));
        lblPrice->setText(ccart::moneyKR(unit * q));
    });

    // - 클릭: qty 감소(0 아래로 안 내려가게) 후 price 갱신
    connect(btnMinus, &QPushButton::clicked, this, [row, lblQty, lblPrice]() {
        int unit = row->property("unitPrice").toInt();
        int q = lblQty->text().toInt();
        if (q > 0) q--;
        lblQty->setText(QString::number(q));
        lblPrice->setText(ccart::moneyKR(unit * q));
    });

    // 취소 클릭: row 제거
    connect(btnCancel, &QPushButton::clicked, this, [row]() {
        row->deleteLater(); // ✅ 안전 삭제
    });

    return row;
}
