#ifndef PAGEPAY_CARD_H
#define PAGEPAY_CARD_H

#include <QWidget>

namespace Ui {
class pagepay_card;
}

class pagepay_card : public QWidget
{
    Q_OBJECT
public:
    explicit pagepay_card(QWidget *parent = nullptr);
    ~pagepay_card();

signals:
    void goTotalPayClicked();   // ✅ 확인 누르면 pagetotalpay로 이동

protected:
    void showEvent(QShowEvent *e) override;
    void resizeEvent(QResizeEvent *e) override;

private:
    void showPaymentModal();

private:
    Ui::pagepay_card *ui;
    bool m_modalShown = false;
};

#endif // PAGEPAY_CARD_H
