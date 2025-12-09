#ifndef PAGECART_H
#define PAGECART_H

#include <QWidget>
#include <QVector>
#include <QLineEdit>
#include <QMap>

struct ItemInfo {
    QString name;
    int price;
    double weight;   // 나중에 로드셀 검증용, 지금은 0으로 둬도 됨
};

namespace Ui {
class PageCart;
}

class PageCart : public QWidget
{
    Q_OBJECT

private slots:
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();

    void onBarcodeEntered();

public:
    explicit PageCart(QWidget *parent = nullptr);
    ~PageCart();

private:
    Ui::PageCart *ui;

    void initDummyItems();
    void updateRowAmount(int row);
    void InitItemDb();
    void updateTotal();
    void handleBarcode(const QString &code);   // ⬅ 바코드 처리용

    QVector<int> m_unitPrice;
    QLineEdit *m_editBarcode;         // ⬅ 스캐너 입력용 숨김 edit
    QMap<QString, ItemInfo> m_itemDb; // ⬅ 바코드 → 상품정보
};

#endif // PAGECART_H
