#include "barcodescanner.h"
#include <QNetworkRequest>
#include <QUrl>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>

BarcodeScanner::BarcodeScanner(QObject *parent)
    : QObject(parent),
      manager(new QNetworkAccessManager(this))
{
    connect(manager, &QNetworkAccessManager::finished,
            this, &BarcodeScanner::onNetworkReply);
}

void BarcodeScanner::setApiBaseUrl(const QString &ip, int port)
{
    m_apiBaseUrl = QString("http://%1:%2").arg(ip).arg(port);
}

// 상품 추가
void BarcodeScanner::fetchItemDetails(const QString& itemId)
{
    QString path = QString("%1/cart/add/%2").arg(m_apiBaseUrl).arg(itemId);
    QUrl url(path);
    
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    
    manager->post(request, QByteArray());
    qDebug() << "[Scanner] Requesting ADD:" << url.toString();
}

// 상품 제거
void BarcodeScanner::removeItem(int itemId)
{
    QString path = QString("%1/cart/remove/%2").arg(m_apiBaseUrl).arg(itemId);
    QUrl url(path);
    
    QNetworkRequest request(url);
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");

    manager->post(request, QByteArray());
    qDebug() << "[Scanner] Requesting REMOVE:" << url.toString();
}

// 서버 응답 처리
void BarcodeScanner::onNetworkReply(QNetworkReply *reply)
{
    // 에러 체크
    if (reply->error() != QNetworkReply::NoError) {
        qDebug() << "[Scanner] Error:" << reply->errorString();
        emit fetchFailed(reply->errorString());
        reply->deleteLater();
        return;
    }

    // 데이터 수신 및 JSON 파싱
    QByteArray data = reply->readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    
    if (!doc.isObject()) {
        qDebug() << "[Scanner] Invalid JSON response";
        reply->deleteLater();
        return;
    }

    QJsonObject obj = doc.object();
    QString action = obj["action"].toString(); 

    // 데이터 처리 
    if (action == "add") {
        Item item;

        if (obj.contains("item")) {
            item.name = obj["item"].toString();
        } else if (obj.contains("name")) {
            item.name = obj["name"].toString();
        } else {
            item.name = "알수없음";
        }

        if (obj.contains("id")) {
            item.id = obj["id"].toInt();
        } else {
            item.id = 0; 
        }

        // 가격
        if (obj.contains("price")) {
            item.price = obj["price"].toInt();
        } else {
            qDebug() << "[Warning] Server response missing 'price' field!";
            item.price = 0; 
        }

        double expectedWeight = obj["expected_weight"].toDouble();

        qDebug() << "[Scanner] Fetched from DB -> Name:" << item.name 
                 << ", Price:" << item.price 
                 << ", ID:" << item.id;

        emit itemFetched(item, expectedWeight);
    }
    else if (action == "remove") {
        qDebug() << "[Scanner] Remove confirmed.";
    }

    reply->deleteLater();
}