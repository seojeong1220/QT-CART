#include "mainwidget.h"
#include "ui_mainwidget.h"
#include <QDebug>
#include <QtNetwork/QHostAddress>
#include <QtNetwork/QNetworkDatagram>
#include <QApplication>
#include <QEvent>
#include <QKeyEvent>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);
    qApp->installEventFilter(this);

    m_udpSocket = new QUdpSocket(this);
    m_udpSocket->bind(QHostAddress::Any, QT_LISTEN_PORT, QUdpSocket::ShareAddress);
    connect(m_udpSocket, &QUdpSocket::readyRead, this, &MainWidget::onUdpReadyRead);

    pPageWelcome = new PageWelcome(this);
    pPageCart    = new PageCart(this);
    pPageCart->setApiConfig(API_SERVER_IP, API_SERVER_PORT);
    pPageGuide   = new PageGuide(this);
    pPagePay     = new PagePay(this);
    pPageCard    = new pagepay_card(this);
    pPageTotalPay= new PageTotalPay(this);

    ui->pstackedWidget->addWidget(pPageWelcome);
    ui->pstackedWidget->addWidget(pPageCart);
    ui->pstackedWidget->addWidget(pPageGuide);
    ui->pstackedWidget->addWidget(pPagePay);
    ui->pstackedWidget->addWidget(pPageCard);
    ui->pstackedWidget->addWidget(pPageTotalPay);
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);

    connect(pPageWelcome, &PageWelcome::startRequested, this, &MainWidget::slotShowCartPage);

    connect(pPageGuide, &PageGuide::requestGoal, this, &MainWidget::onGoalRequestReceived); 
    connect(pPageGuide, &PageGuide::requestStop, this, &MainWidget::onStopRequestReceived); 
    connect(this, SIGNAL(robotArrivedSignal()), pPageGuide, SLOT(onRobotArrived()));        
    connect(pPageGuide, SIGNAL(backToCartClicked()), this, SLOT(slotShowCartPage()));

    connect(pPageCart, SIGNAL(guideModeClicked()), this, SLOT(slotShowGuidePage()));
    connect(pPageCart, SIGNAL(goPay()),            this, SLOT(slotShowPayPage()));
    connect(pPageCart, SIGNAL(goWelcome()),        this, SLOT(slotShowWelcomePage()));

    connect(pPagePay, SIGNAL(creditCardClicked()), this, SLOT(slotShowPayCardPage()));
    connect(pPagePay, SIGNAL(backCartClicked()),   this, SLOT(slotShowCartPage()));

    connect(pPageCard, SIGNAL(goTotalPayClicked()), this, SLOT(slotShowTotalPayPage_2()));
    connect(pPageTotalPay, &PageTotalPay::backToStartClicked, this, &MainWidget::slotShowWelcomePage);

    connect(ui->pstackedWidget, &QStackedWidget::currentChanged, this, &MainWidget::onPageChanged);
    onPageChanged(ui->pstackedWidget->currentIndex());
}

MainWidget::~MainWidget()
{
    delete ui;
}

void MainWidget::sendUdpData(const QString &cmd)
{
    QByteArray data = cmd.toUtf8();
    m_udpSocket->writeDatagram(data, QHostAddress(ROS_SERVER_IP), ROS_SERVER_PORT);
    qDebug() << "[UDP Send]" << cmd << "to" << ROS_SERVER_IP;
}

// ROS에서 온 UDP 수신
void MainWidget::onUdpReadyRead()
{
    while (m_udpSocket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_udpSocket->receiveDatagram();
        QString msg = QString::fromUtf8(datagram.data()).trimmed();
        
        qDebug() << "[UDP Recv]" << msg;

        if (msg == "STATUS:ARRIVED") {
            emit robotArrivedSignal(); 
        }
        else if (msg == "ERROR:WEIGHT_MISMATCH") {  // 무게 불일치 시 
            // TODO: 무게 불일치 팝업 띄우기
        }
    }
}

void MainWidget::onGoalRequestReceived(double x, double y)
{
    QString cmd = QString("GOAL:%1,%2").arg(x).arg(y);
    sendUdpData(cmd);
}

void MainWidget::onStopRequestReceived()
{
    sendUdpData("STOP");
}

// 페이지 변경 시 모드 변경
void MainWidget::onPageChanged(int index)
{
    QWidget *currentWidget = ui->pstackedWidget->widget(index);
    int mode = 0;

    if (currentWidget == pPageWelcome) mode = 0;      // 정지
    else if (currentWidget == pPageCart) mode = 1;    // 따라가기
    else if (currentWidget == pPageGuide) mode = 2;   // 안내모드
    else if (currentWidget == pPagePay) mode = 0;     // 결제 중 정지 
    else mode = 0;

    sendRobotMode(mode);
}

void MainWidget::sendRobotMode(int mode)
{
    sendUdpData(QString("MODE:%1").arg(mode));
}

void MainWidget::on_pPBStartClicked() { ui->pstackedWidget->setCurrentWidget(pPageCart); }
void MainWidget::slotShowCartPage()   { ui->pstackedWidget->setCurrentWidget(pPageCart); }
void MainWidget::slotShowGuidePage()  { ui->pstackedWidget->setCurrentWidget(pPageGuide); }
void MainWidget::slotShowPayCardPage(){ ui->pstackedWidget->setCurrentWidget(pPageCard); }
void MainWidget::slotShowTotalPayPage_2() { ui->pstackedWidget->setCurrentWidget(pPageTotalPay); }

void MainWidget::slotShowWelcomePage()
{
    ui->pstackedWidget->setCurrentWidget(pPageWelcome);
    pPageCart->resetCart();
}

void MainWidget::slotShowPayPage()
{
    auto cartLines = pPageCart->getCartLines();
    QVector<PagePay::PayLine> payLines;
    payLines.reserve(cartLines.size());

    for (const auto &c : cartLines) {
        PagePay::PayLine p;
        p.name = c.name;
        p.qty = c.qty;
        p.unitPrice = c.unitPrice;
        payLines.push_back(p);
    }

    pPagePay->setPayItems(payLines);
    ui->pstackedWidget->setCurrentWidget(pPagePay);
}

bool MainWidget::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        auto *ke = static_cast<QKeyEvent*>(event);
        if (ke->key() == Qt::Key_F12) {
            qApp->quit();
            return true;
        }
    }
    return QWidget::eventFilter(obj, event);
}