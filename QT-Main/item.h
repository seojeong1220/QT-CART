#ifndef ITEM_H
#define ITEM_H

#include <QString>

class Item {
public:
    int id = 0;
    QString name = "";
    double price = 0.0;
    int stock = 0;
    double weight = 0.0;
};

#endif // ITEM_H
