#ifndef CCART_H
#define CCART_H

#include <QWidget>
#include <QVector>
#include <QString>
#include <QVBoxLayout>


namespace Ui {
class ccart;
}
class QVBoxLayout;
class QWidget;

class ccart : public QWidget
{
    Q_OBJECT

public:
    explicit ccart(QWidget *parent = nullptr);
    ~ccart();

private slots:
    void onPlusClicked();
    void onMinusClicked();
    void onDeleteClicked();

private:
    Ui::ccart *ui;

    QWidget*     m_container = nullptr;  // ScrollArea 안에 들어갈 content widget
    QVBoxLayout* m_vbox      = nullptr;  // 아이템 행들을 쌓는 레이아웃

    void setupScrollCart();
    void addCartRow(const QString& name, const QString& imgPath, int unitPrice, int qty = 1);

    // row 위젯 만들기
    QWidget* buildRowWidget(const QString& name, const QString& imgPath, int unitPrice, int qty);

    static QString moneyKR(int v);
};

#endif // CCART_H
