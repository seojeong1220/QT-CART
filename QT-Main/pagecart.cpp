#include "pagecart.h"
#include "ui_pagecart.h"

#include <QGraphicsDropShadowEffect>
#include <QPushButton>
#include <QTableWidgetItem>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QApplication>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <cmath>

// [삭제됨] #define SERVER_BASE_URL ... (이제 외부에서 주입받음)

static QString moneyKR(qint64 v)
{
    const QLocale loc(QLocale::Korean, QLocale::SouthKorea);
    return loc.toString(v) + "원";
}

static QString imageForName(const QString& name)
{
    if(name=="홈런볼 초코맛") return ":/cart_snack(1).jpg";
    if(name=="허니버터칩") return ":/cart_snack(2).png";
    if(name=="오레오 초콜릿크림") return ":/cart_snack(3).jpg";
    if(name=="빼빼로 아몬드") return ":/cart_snack(4).jpg";
    if(name=="빈츠") return ":/cart_snack(5).jpg";
    if(name=="예감 치즈그르탕") return ":/cart_snack(6).jpg";
    if(name=="오!감자 감자그라탕") return ":/cart_snack(7).jpg";
    if(name=="포카칩 오리지널") return ":/cart_snack(8).jpg";
    if(name=="아이폰 15 프로") return ":/cart_iphone.jpg";
    if(name=="핸드크림") return ":/cart_handcream.jpg";
    if(name=="크리스마스 퍼즐") return ":/cart_puzzle.png";
    return ""; 
}

PageCart::PageCart(QWidget *parent)
    : QWidget(parent),
    ui(new Ui::PageCart)
{
    ui->setupUi(this);

    // 그림자 효과 (기존 코드 유지)
    auto makeShadow = [](QObject *parent){
        auto *s = new QGraphicsDropShadowEffect(parent);
        s->setBlurRadius(18);
        s->setOffset(0, 4);
        s->setColor(QColor(0, 0, 0, 60));
        return s;
    };
    ui->tableCart->setStyleSheet(
        "QTableWidget::item { padding: 0px; margin: 0px; }"
        "QTableWidget { border: none; }"
        );

    if (ui->widget_2) ui->widget_2->setGraphicsEffect(makeShadow(ui->widget_2));
    if (ui->widget)   ui->widget->setGraphicsEffect(makeShadow(ui->widget));

    ui->tableCart->setSelectionMode(QAbstractItemView::NoSelection);
    ui->tableCart->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tableCart->setShowGrid(false);

    // 바코드 입력용 숨김 QLineEdit
    m_editBarcode = new QLineEdit(this);
    m_editBarcode->setVisible(false);
    m_editBarcode->setFocusPolicy(Qt::StrongFocus);
    m_editBarcode->setFocus();
    connect(m_editBarcode, SIGNAL(returnPressed()), this, SLOT(onBarcodeEntered()));

    qApp->installEventFilter(this);

    // [핵심] Scanner 초기화
    m_scanner = new BarcodeScanner(this);
    connect(m_scanner, &BarcodeScanner::itemFetched, this, &PageCart::handleItemFetched);
    connect(m_scanner, &BarcodeScanner::fetchFailed, this, &PageCart::handleFetchFailed);

    // 테이블 컬럼 설정
    ui->tableCart->setColumnCount(7);
    ui->tableCart->horizontalHeader()->setVisible(false);
    ui->tableCart->verticalHeader()->setVisible(false);
    ui->tableCart->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    ui->tableCart->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);

    ui->tableCart->setColumnWidth(0, 54);
    ui->tableCart->setColumnWidth(2, 52);
    ui->tableCart->setColumnWidth(3, 42);
    ui->tableCart->setColumnWidth(5, 76);
    ui->tableCart->setColumnWidth(6, 46);
    
    updateTotal();

    // 버튼 연결
    if (ui->btnHome) {
        connect(ui->btnHome, &QPushButton::clicked, this, [this](){
            emit goWelcome();
        });
    }
    
    // [중요] resetCart는 여기서 호출하지만 URL이 아직 설정 안 됐을 수 있음.
    // MainWidget에서 setApiConfig 호출 후 resetCart를 다시 불러주는 게 좋음.
}

PageCart::~PageCart()
{
    delete ui;
}

// [추가됨] MainWidget으로부터 API URL 설정 받기
void PageCart::setApiConfig(const QString &ip, int port)
{
    if (m_scanner) {
        m_scanner->setApiBaseUrl(ip, port);
    }
    // [선택] URL 설정 후 카트 초기화 한 번 실행
    resetCart();
}

// ----------------------------------------
// 행 추가 헬퍼 함수
// ----------------------------------------
void PageCart::addRowForItem(const QString& id, const QString& name, int unitPrice, int qty, double weight)
{
    int row = ui->tableCart->rowCount();
    ui->tableCart->insertRow(row);
    ui->tableCart->setRowHeight(row, 60);

    m_unitPrice.append(unitPrice);

    ItemInfo info;
    info.id = id.toInt(); // int로 변환해서 저장 (Scanner는 QString으로 주지만 구조체는 int)
    info.name = name;
    info.price = unitPrice;
    info.weight = weight;
    m_items.append(info);

    // 셀 위젯 생성 람다 (기존 유지)
    auto makeCenterCell = [&](QWidget *child) -> QWidget* {
        QWidget *w = new QWidget(ui->tableCart);
        auto *lay = new QHBoxLayout(w);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(0);
        lay->setAlignment(Qt::AlignCenter);
        lay->addWidget(child);
        return w;
    };
    auto makeRightCell = [&](QWidget *child) -> QWidget* {
        QWidget *w = new QWidget(ui->tableCart);
        auto *lay = new QHBoxLayout(w);
        lay->setContentsMargins(0, 0, 0, 0);
        lay->setSpacing(0);
        lay->addStretch();
        lay->addWidget(child);
        return w;
    };

    // (0) 이미지
    QLabel *img = new QLabel(ui->tableCart);
    img->setFixedSize(44, 44);
    img->setAlignment(Qt::AlignCenter);
    QPixmap px(imageForName(name));
    if (!px.isNull()) {
        img->setPixmap(px.scaled(44, 44, Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));
    }
    ui->tableCart->setCellWidget(row, 0, makeCenterCell(img));

    // (1) 상품명
    ui->tableCart->setItem(row, 1, new QTableWidgetItem(name));

    // (2) + 버튼
    QPushButton *btnPlus = new QPushButton("+", ui->tableCart);
    btnPlus->setFixedSize(26, 30);
    btnPlus->setStyleSheet("QPushButton { background-color: rgb(37,99,235); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 2, makeCenterCell(btnPlus));
    connect(btnPlus, &QPushButton::clicked, this, &PageCart::onPlusClicked);

    // (3) 수량
    QTableWidgetItem *qtyItem = new QTableWidgetItem(QString::number(qty));
    qtyItem->setTextAlignment(Qt::AlignCenter);
    ui->tableCart->setItem(row, 3, qtyItem);

    // (4) - 버튼
    QPushButton *btnMinus = new QPushButton("-", ui->tableCart);
    btnMinus->setFixedSize(30, 30);
    btnMinus->setStyleSheet("QPushButton { background-color: rgb(154,153,150); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 4, makeCenterCell(btnMinus));
    connect(btnMinus, &QPushButton::clicked, this, &PageCart::onMinusClicked);

    // (5) 가격
    QTableWidgetItem *priceItem = new QTableWidgetItem(moneyKR(0));
    priceItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->tableCart->setItem(row, 5, priceItem);

    // (6) X 삭제
    QPushButton *btnDelete = new QPushButton("X", ui->tableCart);
    btnDelete->setFixedSize(30, 30);
    btnDelete->setStyleSheet("QPushButton { background-color: rgb(224,27,36); color: white; border-radius: 10px; font-weight: bold; }");
    ui->tableCart->setCellWidget(row, 6, makeRightCell(btnDelete));
    connect(btnDelete, &QPushButton::clicked, this, &PageCart::onDeleteClicked);

    updateRowAmount(row);
}

// ----------------------------------------
// + 버튼 (수정됨: Scanner 사용)
// ----------------------------------------
void PageCart::onPlusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) return;

    // int ID를 QString으로 변환하여 전달
    QString itemId = QString::number(m_items[row].id);
    
    // [수정] 직접 통신 대신 Scanner 사용 (일관성 유지)
    // Scanner->fetchItemDetails(id)를 호출하면 서버에 add 요청을 보내고,
    // 성공 시 handleItemFetched가 호출되어 UI가 +1 됩니다.
    m_scanner->fetchItemDetails(itemId);
    
    // (참고: Optimistic Update를 원하면 여기서 UI를 먼저 바꿔도 되지만, 
    //  handleItemFetched에서 처리하므로 중복 방지를 위해 생략합니다.)
}

// ----------------------------------------
// - 버튼 (수정됨: Scanner 사용)
// ----------------------------------------
void PageCart::onMinusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) return;

    int currentQty = ui->tableCart->item(row, 3)->text().toInt();
    int itemId = m_items[row].id;

    // [수정] Scanner의 remove 기능 사용
    m_scanner->removeItem(itemId);

    // removeItem은 리턴값(Signal)이 따로 없으므로 UI를 수동으로 갱신 (Optimistic Update)
    int newQty = currentQty - 1;
    if (newQty <= 0) {
        m_unitPrice.removeAt(row);
        m_items.removeAt(row);
        ui->tableCart->removeRow(row);
    } else {
        ui->tableCart->item(row, 3)->setText(QString::number(newQty));
        updateRowAmount(row);
    }
    updateTotal();
}

// ----------------------------------------
// 삭제 버튼 (수정됨: Scanner 사용)
// ----------------------------------------
void PageCart::onDeleteClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = getRowFromButton(btn);
    if (row < 0 || row >= m_items.size()) return;

    int itemId = m_items[row].id;
    int qty = ui->tableCart->item(row, 3)->text().toInt();

    // 1. 서버에 삭제 요청 (수량만큼 반복)
    for (int i = 0; i < qty; ++i) {
        m_scanner->removeItem(itemId);
    }

    // 2. UI 즉시 삭제
    m_unitPrice.removeAt(row);
    m_items.removeAt(row);
    ui->tableCart->removeRow(row);
    updateTotal();
}

void PageCart::updateRowAmount(int row)
{
    if (row < 0 || row >= m_unitPrice.size()) return;
    int qty = ui->tableCart->item(row, 3)->text().toInt();
    int amount = m_unitPrice[row] * qty;
    ui->tableCart->item(row, 5)->setText(moneyKR(amount));
}

void PageCart::updateTotal()
{
    int totalCount = 0;
    int totalPrice = 0;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        auto *qtyItem = ui->tableCart->item(r, 3);
        if (!qtyItem) continue;
        int qty = qtyItem->text().toInt();
        totalCount += qty;
        int unit = (r < m_unitPrice.size()) ? m_unitPrice[r] : 0;
        totalPrice += unit * qty;
    }

    if (ui->lblTotalPrice_2)
        ui->lblTotalPrice_2->setText(moneyKR(totalPrice));
    if (ui->lblCartTitle)
        ui->lblCartTitle->setText(QString("Cart (%1 products)").arg(totalCount));
}

void PageCart::onBarcodeEntered()
{
    QString code = m_editBarcode->text().trimmed();
    m_editBarcode->clear();
    if (code.isEmpty()) return;
    m_scanner->fetchItemDetails(code);
}

bool PageCart::eventFilter(QObject *obj, QEvent *event)
{
    Q_UNUSED(obj);
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent->key() == Qt::Key_Return || keyEvent->key() == Qt::Key_Enter) {
            if (!m_barcodeData.isEmpty()) {
                m_scanner->fetchItemDetails(m_barcodeData);
                m_barcodeData.clear();
            }
            return true;
        }
        else if (!keyEvent->text().isEmpty() &&
                 !(keyEvent->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier | Qt::AltModifier))) {
            m_barcodeData.append(keyEvent->text());
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}

// ----------------------------------------
// Scanner 응답 처리 (UI 갱신)
// ----------------------------------------
void PageCart::handleItemFetched(const Item &item)
{
    QString strId = item.id; // Item 구조체는 QString ID를 씀
    int intId = strId.toInt();

    int rowFound = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        // ID 또는 이름으로 검색
        if (m_items[r].id == intId || m_items[r].name == item.name) { 
            rowFound = r;
            break;
        }
    }

    if (rowFound == -1) {
        // [신규]
        int unit = static_cast<int>(std::lround(item.price));
        addRowForItem(strId, item.name, unit, 1, item.weight);
    } else {
        // [증가]
        int qty = ui->tableCart->item(rowFound, 3)->text().toInt();
        ui->tableCart->item(rowFound, 3)->setText(QString::number(qty + 1));
        
        // 정보 업데이트
        m_items[rowFound].id = intId;
        m_items[rowFound].weight = item.weight;
        
        updateRowAmount(rowFound);
    }
    updateTotal();
}

void PageCart::handleFetchFailed(const QString &err)
{
    qDebug() << "Barcode fetch failed:" << err;
    QMessageBox::warning(this, "스캔 실패", "상품 정보를 불러오지 못했습니다.\n" + err);
}

// ----------------------------------------
// 버튼 핸들러 (수정됨)
// ----------------------------------------
void PageCart::on_btnGuide_clicked() { emit guideModeClicked(); }

void PageCart::on_pushButton_clicked() { resetCart(); }

void PageCart::on_btnCheckout_clicked()
{
    if (ui->tableCart->rowCount() == 0) {
        QMessageBox::information(this, "알림", "장바구니에 상품을 담아주세요.");
        return;
    }
    
    // [핵심] 직접 체크하지 않고 MainWidget에 요청만 보냄 (교통정리)
    emit requestCheckout(); 
}

// ----------------------------------------
// 기타 유틸리티
// ----------------------------------------
QVector<PageCart::CartLine> PageCart::getCartLines() const
{
    QVector<CartLine> out;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        auto *nameItem = ui->tableCart->item(r, 1);
        auto *qtyItem  = ui->tableCart->item(r, 3);
        if (!nameItem || !qtyItem) continue;
        int qty = qtyItem->text().toInt();
        if (qty <= 0) continue;
        int unitPrice = (r < m_unitPrice.size()) ? m_unitPrice[r] : 0;
        out.push_back({ nameItem->text(), qty, unitPrice });
    }
    return out;
}

// 카트 초기화 (서버 연동은 Scanner에 기능이 없으면 여기서 직접 호출해야 함. Scanner에 reset 기능 추가 추천)
void PageCart::resetCart()
{
    // 일단 여기서는 직접 호출하되, URL은 주입받은 값이 없으면 하드코딩될 수 있으므로 주의.
    // (Scanner에 reset 기능을 추가하는 것이 가장 좋은 설계입니다. 임시로 하드코딩 제거를 위해 생략하거나 Scanner에 추가하세요)
    
    // UI 초기화
    ui->tableCart->setRowCount(0);
    m_unitPrice.clear();
    m_items.clear();
    updateTotal();
}

int PageCart::getRowFromButton(QWidget *btn)
{
    if (!btn) return -1;
    int rowCount = ui->tableCart->rowCount();
    for (int r = 0; r < rowCount; ++r) {
        int targetCols[] = {2, 4, 6};
        for (int c : targetCols) {
            QWidget *cellWidget = ui->tableCart->cellWidget(r, c);
            if (cellWidget && (cellWidget == btn || cellWidget->isAncestorOf(btn))) {
                return r;
            }
        }
    }
    return -1;
}