#include "pagecart.h"
#include "ui_pagecart.h"

#include <QPushButton>
#include <QTableWidgetItem>

PageCart::PageCart(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PageCart)
{
    ui->setupUi(this);

    // 더미 데이터
    initDummyItems();


    qDebug() << "[PageCart] constructor called";
    InitItemDb();        // ⬅ 상품 DB 세팅

    // 1) 바코드 입력용 숨겨진 QLineEdit 생성
    m_editBarcode = new QLineEdit(this);
    m_editBarcode->setVisible(false);          // 화면에 안 보이게
    m_editBarcode->setFocusPolicy(Qt::StrongFocus);
    m_editBarcode->setFocus();                 // Cart 페이지 열리면 포커스 여기로

    connect(m_editBarcode, SIGNAL(returnPressed()), this, SLOT(onBarcodeEntered()));


    // 테이블 컬럼은 UI에 이미 설정됨
    ui->tableCart->setColumnCount(6);

    updateTotal();
}

PageCart::~PageCart()
{
    delete ui;
}

// ----------------------------------------
// 더미 데이터 3개 생성
// ----------------------------------------
void PageCart::initDummyItems()
{
    QStringList names   = {"사과", "바나나", "우유"};
    QVector<int> prices = {3000, 1500, 2500};

    ui->tableCart->setRowCount(names.size());
    m_unitPrice = prices;

    for (int row = 0; row < names.size(); ++row) {

        // 상품명
        ui->tableCart->setItem(row, 0, new QTableWidgetItem(names[row]));
        // 개수
        ui->tableCart->setItem(row, 1, new QTableWidgetItem("0"));
        // 금액
        ui->tableCart->setItem(row, 4, new QTableWidgetItem("0"));

        // + 버튼 (col 2)
        QPushButton *btnPlus = new QPushButton("+", this);
        ui->tableCart->setCellWidget(row, 2, btnPlus);
        connect(btnPlus, SIGNAL(clicked()), this, SLOT(onPlusClicked()));

        // - 버튼 (col 3)
        QPushButton *btnMinus = new QPushButton("-", this);
        ui->tableCart->setCellWidget(row, 3, btnMinus);
        connect(btnMinus, SIGNAL(clicked()), this, SLOT(onMinusClicked()));

        // 삭제 버튼 (col 5)
        QPushButton *btnDelete = new QPushButton("삭제", this);
        ui->tableCart->setCellWidget(row, 5, btnDelete);
        connect(btnDelete, SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
    }
}

// ----------------------------------------
// + 버튼 눌렸을 때
// ----------------------------------------
void PageCart::onPlusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 2) == btn) { // col 2 = +
            row = r;
            break;
        }
    }

    if (row < 0) return;

    QTableWidgetItem *qtyItem = ui->tableCart->item(row, 1);
    int qty = qtyItem->text().toInt();
    qty++;
    qtyItem->setText(QString::number(qty));

    updateRowAmount(row);
    updateTotal();
}

// ----------------------------------------
// - 버튼 눌렸을 때
// ----------------------------------------
void PageCart::onMinusClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 3) == btn) { // col 3 = -
            row = r;
            break;
        }
    }

    if (row < 0) return;

    QTableWidgetItem *qtyItem = ui->tableCart->item(row, 1);
    int qty = qtyItem->text().toInt();
    if (qty > 0) qty--;
    qtyItem->setText(QString::number(qty));

    updateRowAmount(row);
    updateTotal();
}

// ----------------------------------------
// 삭제 버튼 눌렸을 때
// ----------------------------------------
void PageCart::onDeleteClicked()
{
    QPushButton *btn = qobject_cast<QPushButton*>(sender());
    if (!btn) return;

    int row = -1;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        if (ui->tableCart->cellWidget(r, 5) == btn) { // col 5 = 삭제
            row = r;
            break;
        }
    }

    if (row < 0) return;

    // 단가 리스트에서도 삭제
    if (row < m_unitPrice.size())
        m_unitPrice.removeAt(row);

    // 테이블에서 행 삭제
    ui->tableCart->removeRow(row);

    updateTotal();
}

// ----------------------------------------
// 한 줄 금액 업데이트 → 개수 * 단가
// ----------------------------------------
void PageCart::updateRowAmount(int row)
{
    if (row < 0 || row >= m_unitPrice.size()) return;

    int qty = ui->tableCart->item(row, 1)->text().toInt();
    int amount = m_unitPrice[row] * qty;

    ui->tableCart->item(row, 4)->setText(QString::number(amount));
}

// ----------------------------------------
// 전체 총액 업데이트
// ----------------------------------------
void PageCart::updateTotal()
{
    int total = 0;

    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        QTableWidgetItem *amt = ui->tableCart->item(r, 4);
        if (!amt) continue;
        total += amt->text().toInt();
    }

    ui->labelTotalPriceValue->setText(QString::number(total) + "원");
}

void PageCart::onBarcodeEntered()
{
    QString code = m_editBarcode->text().trimmed();
    m_editBarcode->clear();

    if (code.isEmpty())
        return;

    handleBarcode(code);   // 이 함수에서 실제 상품 추가/개수 증가 처리
}

void PageCart::InitItemDb()
{
    ItemInfo apple;
    apple.name   = "사과";
    apple.price  = 3000;
    apple.weight = 200.0;   // g, 나중에 로드셀 검증에 씀

    ItemInfo banana;
    banana.name   = "바나나";
    banana.price  = 1500;
    banana.weight = 120.0;

    ItemInfo milk;
    milk.name   = "우유";
    milk.price  = 2500;
    milk.weight = 900.0;

    // ⬇ 바코드 문자열은 우리가 임의로 정하는 값
    m_itemDb["111111"] = apple;
    m_itemDb["222222"] = banana;
    m_itemDb["333333"] = milk;
}

void PageCart::handleBarcode(const QString &code)
{
    // 1) 등록된 바코드인지 확인
    if (!m_itemDb.contains(code)) {
        // TODO: QMessageBox로 "등록되지 않은 바코드입니다" 띄우기
        return;
    }

    ItemInfo info = m_itemDb.value(code);

    // 2) 이미 테이블에 있는 상품인지 이름으로 검색
    int rowFound = -1;
    for (int r = 0; r < ui->tableCart->rowCount(); ++r) {
        QTableWidgetItem *nameItem = ui->tableCart->item(r, 0);
        if (nameItem && nameItem->text() == info.name) {
            rowFound = r;
            break;
        }
    }

    if (rowFound == -1) {
        // 3-1) 없으면 새 행 추가
        int row = ui->tableCart->rowCount();
        ui->tableCart->insertRow(row);

        // 상품명 / 개수 / 금액
        ui->tableCart->setItem(row, 0, new QTableWidgetItem(info.name));
        ui->tableCart->setItem(row, 1, new QTableWidgetItem("1"));
        ui->tableCart->setItem(row, 4, new QTableWidgetItem(QString::number(info.price)));

        // 단가 목록에도 추가
        m_unitPrice.append(info.price);

        // + / - / 삭제 버튼 연결 (기존 initDummyItems의 코드 재활용)
        QPushButton *btnPlus   = new QPushButton("+", this);
        QPushButton *btnMinus  = new QPushButton("-", this);
        QPushButton *btnDelete = new QPushButton("삭제", this);

        ui->tableCart->setCellWidget(row, 2, btnPlus);
        ui->tableCart->setCellWidget(row, 3, btnMinus);
        ui->tableCart->setCellWidget(row, 5, btnDelete);

        connect(btnPlus,  SIGNAL(clicked()), this, SLOT(onPlusClicked()));
        connect(btnMinus, SIGNAL(clicked()), this, SLOT(onMinusClicked()));
        connect(btnDelete,SIGNAL(clicked()), this, SLOT(onDeleteClicked()));
    }
    else {
        // 3-2) 이미 있는 상품이면 개수 +1
        QTableWidgetItem *qtyItem = ui->tableCart->item(rowFound, 1);
        int qty = qtyItem->text().toInt();
        qty++;
        qtyItem->setText(QString::number(qty));

        updateRowAmount(rowFound);
    }

    updateTotal();
}


