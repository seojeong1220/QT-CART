#ifndef PAGEWELCOME_H
#define PAGEWELCOME_H

#include <QWidget>
#include <QPoint>

class QPropertyAnimation;
class QShowEvent;
class QHideEvent;
class QResizeEvent;

namespace Ui {
class PageWelcome;
}

class PageWelcome : public QWidget
{
    Q_OBJECT

public:
    explicit PageWelcome(QWidget *parent = nullptr);
    ~PageWelcome();

signals:
    // 너 MainWidget에서 어떤 시그널을 connect 했는지 몰라서 둘 다 유지
    void startClicked();
    void startRequested();

protected:
    void showEvent(QShowEvent *e) override;
    void hideEvent(QHideEvent *e) override;
    void resizeEvent(QResizeEvent *e) override;

private slots:
    void onStartClicked();

private:
    void loadPixmapsOnce();
    void centerStartImage();     // start_image를 가운데로
    void resetWelcome();         // ✅ 돌아왔을 때 초기화(버튼+위치+애니메이션 정리)

private:
    Ui::PageWelcome *ui;

    bool m_leaving = false;
    bool m_pixLoaded = false;

    int  m_baseY = -1;                // start_image의 기준 Y(처음 UI 배치값)
    QPropertyAnimation *m_anim = nullptr;
};

#endif // PAGEWELCOME_H
