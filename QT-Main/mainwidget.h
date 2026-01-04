#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <QtNetwork/QUdpSocket>

#define ROS_SERVER_IP   "192.168.123.42"
#define ROS_SERVER_PORT 55555

#define API_SERVER_IP   "192.168.123.43"
#define API_SERVER_PORT 8000

#define QT_LISTEN_PORT  55555

#include "pagewelcome.h"
#include "pagecart.h"
#include "pageguide.h"
#include "pagepay.h"
#include "pagepay_card.h"
#include "pagetotalpay.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWidget; }
QT_END_NAMESPACE

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

signals:
    void robotArrivedSignal(); 

private:
    Ui::MainWidget *ui;
    QUdpSocket *m_udpSocket; 

    PageWelcome *pPageWelcome;
    PageCart *pPageCart;
    PageGuide *pPageGuide;
    PagePay *pPagePay;
    pagepay_card *pPageCard;
    PageTotalPay *pPageTotalPay; 

    void sendUdpData(const QString &cmd);   
    void sendRobotMode(int mode); // 모드 전송 (0:정지, 1:따라가기, 2:안내)

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void onUdpReadyRead();
    void onGoalRequestReceived(double x, double y); 
    void onStopRequestReceived();                   
    
    void onPageChanged(int index);
    void slotShowCartPage();
    void slotShowGuidePage();
    void slotShowWelcomePage();
    void slotShowPayPage();
    void slotShowPayCardPage();
    void slotShowTotalPayPage_2();
    void on_pPBStartClicked();
};

#endif // MAINWIDGET_H