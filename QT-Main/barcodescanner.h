#ifndef BARCODESCANNER_H
#define BARCODESCANNER_H

#include <QObject>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include "item.h" 

class BarcodeScanner : public QObject
{
    Q_OBJECT
public:
    explicit BarcodeScanner(QObject *parent = nullptr);

    void setApiBaseUrl(const QString &ip, int port);
    void fetchItemDetails(const QString& itemId); 
    void removeItem(int itemId);

signals:
    void itemFetched(const Item &item, double cartWeight); 
    void fetchFailed(const QString &error);

private slots:
    void onNetworkReply(QNetworkReply *reply);

private:
    QNetworkAccessManager *manager;
    QString m_apiBaseUrl; 
};

#endif // BARCODESCANNER_H